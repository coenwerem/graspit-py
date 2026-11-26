"""Export static contact-planner records as NumPy arrays and JSON metadata.

Run ``python -m graspit_py.dataset --help`` for the JSON-to-NPZ command.
See ``docs/datasets.md`` for the versioned field and coordinate contracts.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


SCHEMA_VERSION = "1.0"
SCHEMA_NAME = "graspit-py.static-grasps"
_RECORD_KIND = "static_grasp_configuration"
_UNITS = {"length": "m", "dof_angle": "rad", "roll_angle": "deg"}
_FRAMES = {
    "T_object_hand_m": "maps hand-base coordinates into the object frame",
    "contact_positions_m": "object", "contact_normals": "object, inward, unit length",
    "approach_dir": "object, hand direction of travel",
    "object_pointcloud_m": "object, caller supplied",
}
_SCORES = ("native_epsilon", "nominal_epsilon", "expected_epsilon", "epsilon_cvar", "score")
SCORE_MEANINGS = {
    "native_epsilon": "Unmodified native GraspIt! epsilon at nominal closure; native conventions apply.",
    "nominal_epsilon": "Recomputed GWS epsilon at nominal closure, middle friction column.",
    "expected_epsilon": "Arithmetic mean of all recomputed epsilon samples.",
    "epsilon_cvar": "Lower-tail mean of recomputed epsilon samples at planner.tail_mass.",
    "score": "Planner ranking score: epsilon_cvar or lower-tail task wrench capacity, as score_kind states.",
    "nominal_task": "Task wrench capacity at nominal closure, middle friction column, when present.",
    "expected_task": "Arithmetic mean of task wrench capacity samples, when present.",
    "task_samples": "Recomputed task wrench capacity under the source task loads; capacity is not a success label.",
    "epsilon_samples": "Recomputed GWS epsilon; row 0 nominal, subsequent rows execution-error closures; columns source friction_samples.",
}


@dataclass(frozen=True)
class GraspDataset:
    """Validated in-memory arrays and their JSON metadata."""

    arrays: dict[str, np.ndarray]
    metadata: dict


def _numeric(value, name, shape=None):
    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in "iuf":
            raise ValueError("expected real numbers")
        out = np.asarray(value, dtype=np.float64)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError(f"{name} must contain finite real numbers") from exc
    if not np.isfinite(out).all():
        raise ValueError(f"{name} must contain finite values")
    if shape is not None and out.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {out.shape}")
    return out


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")
    return value


def _names(values, name):
    if isinstance(values, str):
        raise ValueError(f"{name} must be an ordered sequence of names")
    result = [_text(v, name) for v in values]
    if not result or len(set(result)) != len(result):
        raise ValueError(f"{name} must contain unique names")
    return result


def _transform(value, name):
    out = _numeric(value, f"{name} transform", (4, 4))
    if not np.allclose(out[3], [0, 0, 0, 1], atol=1e-9, rtol=0):
        raise ValueError(f"{name} transform must have homogeneous last row [0,0,0,1]")
    rotation = out[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6, rtol=0)):
        raise ValueError(f"{name} rotation must be a proper orthonormal matrix")
    return out


def _unit_vectors(value, name):
    if not np.allclose(np.linalg.norm(value, axis=-1), 1, atol=1e-5, rtol=0):
        raise ValueError(f"{name} must contain unit vectors")


def _sample(value, name):
    out = _numeric(value, name)
    if out.ndim != 2 or 0 in out.shape:
        raise ValueError(f"{name} must be a nonempty pose-by-friction matrix")
    return out


def _json_bytes(value):
    try:
        return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    except (ValueError, TypeError) as exc:
        raise ValueError("source metadata must be finite JSON data") from exc


def _sha256(data):
    return hashlib.sha256(data).hexdigest()


def export_dataset(source: Mapping, out_npz, *, hand_id: str, object_id: str,
                   dof_names: Sequence[str], mesh_scale=1.0, model_files=None,
                   object_pointcloud_m=None, source_path=None) -> tuple[Path, Path]:
    """Write one hand/object archive and a same-stem JSON metadata file.

    ``source`` is a planning CLI payload containing ``candidates`` produced by
    ``GraspCandidate.to_dict()``. ``dof_names`` labels its independent DOFs in
    their existing order. ``mesh_scale`` records already-applied mesh scaling;
    it does not scale records again. Only explicitly supplied ``model_files``
    are read and hashed. ``source_path`` optionally records an exact file hash.
    """
    hand_id, object_id = _text(hand_id, "hand_id"), _text(object_id, "object_id")
    names = _names(dof_names, "dof_names")
    scale = _numeric(mesh_scale, "mesh_scale")
    if scale.shape == ():
        scale = np.repeat(scale, 3)
    if scale.shape != (3,) or np.any(scale <= 0):
        raise ValueError("mesh_scale must be a positive scalar or three-vector")
    if not isinstance(source, Mapping) or not isinstance(source.get("candidates"), list):
        raise ValueError("source must contain a candidates list")
    units = source.get("units", {})
    if not isinstance(units, Mapping) or units.get("length", "mm") != "mm" or units.get("angle", "rad") != "rad":
        raise ValueError("source units must be mm and rad")
    planner = source.get("planner", {})
    if not isinstance(planner, Mapping):
        raise ValueError("source planner must be a JSON object")
    friction = _numeric(planner.get("friction_samples", []), "friction_samples")
    if friction.ndim != 1 or np.any(friction < 0):
        raise ValueError("friction_samples must be a nonnegative vector")

    records, ndof = source["candidates"], len(names)
    n = len(records)
    a = {
        "schema_version": np.asarray(SCHEMA_VERSION),
        "hand_id": np.asarray(hand_id), "object_id": np.asarray(object_id),
        "dof_names": np.asarray(names), "mesh_scale": scale,
        "friction_samples": friction,
        "T_object_hand_m": np.empty((n, 4, 4), dtype=np.float64),
        "dofs_rad": np.empty((n, ndof), dtype=np.float64),
        "preshape_rad": np.empty((n, ndof), dtype=np.float64),
        "closing_mask": np.zeros((n, ndof), dtype=np.bool_),
        "approach_dir": np.empty((n, 3), dtype=np.float64),
        "roll_deg": np.empty(n, dtype=np.float64),
        "task_present": np.zeros(n, dtype=np.bool_),
    }
    for key in (*_SCORES, "nominal_task", "expected_task"):
        a[key] = np.zeros(n, dtype=np.float64)
    for key in ("nominal_task_present", "expected_task_present"):
        a[key] = np.zeros(n, dtype=np.bool_)
    ids, kinds, positions, normals = [], [], [], []
    contact_offsets, eps_offsets, task_offsets = [0], [0], [0]
    eps_shapes, task_shapes, eps_values, task_values = [], [], [], []
    for i, candidate in enumerate(records):
        if not isinstance(candidate, Mapping):
            raise ValueError(f"candidate {i} must be a JSON object")
        try:
            transform = _transform(candidate["T_object_hand_mm"], f"candidate {i}").copy()
            transform[:3, 3] /= 1000.0
            a["T_object_hand_m"][i] = transform
            a["dofs_rad"][i] = _numeric(candidate["dofs_rad"], "dofs_rad", (ndof,))
            a["preshape_rad"][i] = _numeric(candidate["preshape"], "preshape", (ndof,))
            a["approach_dir"][i] = _numeric(candidate["approach_dir"], "approach_dir", (3,))
            _unit_vectors(a["approach_dir"][i], "approach_dir")
            a["roll_deg"][i] = _numeric(candidate["roll_deg"], "roll_deg", ())
            for key in _SCORES:
                a[key][i] = _numeric(candidate[key], key, ())
            contacts = candidate["contacts"]
            if not isinstance(contacts, list):
                raise ValueError("contacts must be a list")
            for contact in contacts:
                positions.append(_numeric(contact["position_mm"], "contact position_mm", (3,)) / 1000.0)
                normal = _numeric(contact["normal"], "contact normal", (3,))
                _unit_vectors(normal, "contact normal")
                normals.append(normal)
            contact_offsets.append(len(positions))
            closing = candidate.get("closing")
            if closing is None:
                a["closing_mask"][i] = True
            else:
                if (not isinstance(closing, list) or any(type(j) is not int or not 0 <= j < ndof for j in closing)
                        or len(set(closing)) != len(closing)):
                    raise ValueError("closing must contain unique independent DOF indices")
                a["closing_mask"][i, closing] = True
            eps = _sample(candidate["epsilon_samples"], "epsilon_samples")
            if friction.size and eps.shape[1] != friction.size:
                raise ValueError("epsilon_samples columns must match friction_samples")
            eps_values.extend(eps.ravel())
            eps_shapes.append(eps.shape)
            eps_offsets.append(len(eps_values))
            task = candidate.get("task_samples")
            if task is None:
                task_shapes.append((0, 0))
                if candidate.get("nominal_task") is not None or candidate.get("expected_task") is not None:
                    raise ValueError("task_samples required when task summaries are present")
            else:
                task = _sample(task, "task_samples")
                if task.shape != eps.shape:
                    raise ValueError("task_samples shape must match epsilon_samples")
                a["task_present"][i] = True
                task_values.extend(task.ravel())
                task_shapes.append(task.shape)
            task_offsets.append(len(task_values))
            for key in ("nominal_task", "expected_task"):
                if candidate.get(key) is not None:
                    a[key][i] = _numeric(candidate[key], key, ())
                    a[key + "_present"][i] = True
            kinds.append("task_wrench_capacity" if task is not None else "epsilon")
            ids.append(_text(candidate.get("grasp_id", f"{hand_id}/{object_id}/{i:06d}"), "grasp_id"))
        except (KeyError, TypeError) as exc:
            raise ValueError(f"candidate {i} is missing or has a malformed field: {exc}") from exc
    if len(set(ids)) != len(ids):
        raise ValueError("grasp_ids must be unique within an archive")
    a.update(grasp_ids=np.asarray(ids, dtype=np.str_), score_kind=np.asarray(kinds, dtype=np.str_),
             contact_positions_m=np.asarray(positions, dtype=np.float64).reshape(-1, 3),
             contact_normals=np.asarray(normals, dtype=np.float64).reshape(-1, 3),
             contact_offsets=np.asarray(contact_offsets, dtype=np.int64),
             epsilon_samples=np.asarray(eps_values, dtype=np.float64),
             epsilon_sample_offsets=np.asarray(eps_offsets, dtype=np.int64),
             epsilon_sample_shapes=np.asarray(eps_shapes, dtype=np.int64).reshape(n, 2),
             task_samples=np.asarray(task_values, dtype=np.float64),
             task_sample_offsets=np.asarray(task_offsets, dtype=np.int64),
             task_sample_shapes=np.asarray(task_shapes, dtype=np.int64).reshape(n, 2))
    if object_pointcloud_m is not None:
        points = _numeric(object_pointcloud_m, "object_pointcloud_m")
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("object_pointcloud_m must have shape (P,3)")
        a["object_pointcloud_m"] = points
    provenance = {"source_sha256": _sha256(_json_bytes(source)),
                  "config_sha256": _sha256(_json_bytes({k: source.get(k) for k in ("hand", "planner", "units")})),
                  "model_files": {}}
    for label, path in (model_files or {}).items():
        label, path = _text(label, "model file label"), Path(path)
        provenance["model_files"][label] = {"path": str(path), "sha256": _sha256(path.read_bytes())}
    if source_path is not None:
        path = Path(source_path)
        provenance.update(source_path=str(path), source_file_sha256=_sha256(path.read_bytes()))
    metadata = {
        "schema_name": SCHEMA_NAME, "schema_version": SCHEMA_VERSION,
        "hand_id": hand_id, "object_id": object_id, "dof_names": names,
        "record_kind": _RECORD_KIND, "mesh_scale": scale.tolist(),
        "units": _UNITS, "frames": _FRAMES,
        "source": {k: v for k, v in source.items() if k != "candidates"},
        "provenance": provenance, "score_meanings": SCORE_MEANINGS,
        "sample_layout": "C-order pose-by-friction matrices; row 0 nominal, later rows perturbed execution closures; actual perturbation transforms are not in source records",
        "fields": {k: {"shape": list(v.shape), "dtype": str(v.dtype)} for k, v in a.items()},
    }
    _validate(a, metadata)
    out = Path(out_npz)
    if out.suffix != ".npz":
        raise ValueError("out_npz must end in .npz")
    meta = out.with_suffix(".json")
    if source_path is not None and Path(source_path).resolve() in (out.resolve(), meta.resolve()):
        raise ValueError("output paths must not replace the source file; use a different stem or directory")
    encoded_metadata = json.dumps(metadata, indent=2, allow_nan=False) + "\n"
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, **a)
    meta.write_text(encoded_metadata, encoding="utf-8")
    return out, meta


def _validate(a, metadata):
    _json_bytes(metadata)
    if (metadata.get("schema_name") != SCHEMA_NAME or metadata.get("schema_version") != SCHEMA_VERSION
            or "schema_version" not in a or a["schema_version"].shape != ()
            or a["schema_version"].item() != SCHEMA_VERSION):
        raise ValueError("unsupported or mismatched dataset schema")
    for key, expected in (("record_kind", _RECORD_KIND), ("units", _UNITS), ("frames", _FRAMES)):
        if metadata.get(key) != expected:
            raise ValueError(f"{key} disagrees with the schema {SCHEMA_VERSION} coordinate contract")
    for key in ("grasp_ids", "dof_names"):
        if key not in a or a[key].ndim != 1:
            raise ValueError(f"{key} must be a vector")
    n, ndof = len(a.get("grasp_ids", [])), len(a.get("dof_names", []))
    specs = {
        "schema_version": ((), "U"), "hand_id": ((), "U"), "object_id": ((), "U"),
        "dof_names": ((ndof,), "U"), "grasp_ids": ((n,), "U"), "score_kind": ((n,), "U"),
        "mesh_scale": ((3,), "f"), "friction_samples": ((None,), "f"),
        "T_object_hand_m": ((n, 4, 4), "f"), "dofs_rad": ((n, ndof), "f"),
        "preshape_rad": ((n, ndof), "f"), "closing_mask": ((n, ndof), "b"),
        "approach_dir": ((n, 3), "f"), "roll_deg": ((n,), "f"),
        "contact_positions_m": ((None, 3), "f"), "contact_normals": ((None, 3), "f"),
        "contact_offsets": ((n + 1,), "i"),
    }
    for key in (*_SCORES, "nominal_task", "expected_task"):
        specs[key] = ((n,), "f")
    for key in ("task_present", "nominal_task_present", "expected_task_present"):
        specs[key] = ((n,), "b")
    for prefix in ("epsilon", "task"):
        specs[prefix + "_samples"] = ((None,), "f")
        specs[prefix + "_sample_offsets"] = ((n + 1,), "i")
        specs[prefix + "_sample_shapes"] = ((n, 2), "i")
    if "object_pointcloud_m" in a:
        specs["object_pointcloud_m"] = ((None, 3), "f")
    if (set(a) != set(specs) or not isinstance(metadata.get("fields"), dict)
            or set(metadata["fields"]) != set(specs)):
        raise ValueError("schema fields are missing or unexpected")
    for key, (shape, kind) in specs.items():
        value = a[key]
        if (value.dtype.kind != kind or value.ndim != len(shape)
                or any(want is not None and want != got for want, got in zip(shape, value.shape))):
            raise ValueError(f"{key} has invalid shape or dtype")
        if kind == "f" and (value.dtype != np.float64 or not np.isfinite(value).all()):
            raise ValueError(f"{key} must contain finite float64 values")
        if kind == "i" and value.dtype != np.int64:
            raise ValueError(f"{key} must contain int64 offsets or shapes")
        if metadata["fields"][key] != {"shape": list(value.shape), "dtype": str(value.dtype)}:
            raise ValueError(f"{key} disagrees with metadata schema")
    _names(a["dof_names"].tolist(), "dof_names")
    if len(set(a["grasp_ids"].tolist())) != n or any(not s.strip() for s in a["grasp_ids"].tolist()):
        raise ValueError("grasp_ids must be unique nonempty strings")
    for key in ("hand_id", "object_id"):
        _text(a[key].item(), key)
        if metadata.get(key) != a[key].item():
            raise ValueError(f"{key} disagrees with metadata")
    if metadata.get("dof_names") != a["dof_names"].tolist():
        raise ValueError("dof_names disagrees with metadata")
    if metadata.get("mesh_scale") != a["mesh_scale"].tolist():
        raise ValueError("mesh_scale disagrees with metadata")
    if np.any(a["mesh_scale"] <= 0) or np.any(a["friction_samples"] < 0):
        raise ValueError("invalid mesh_scale or friction_samples")
    for transform in a["T_object_hand_m"]:
        _transform(transform, "T_object_hand_m")
    _unit_vectors(a["contact_normals"], "contact_normals")
    _unit_vectors(a["approach_dir"], "approach_dir")
    if len(a["contact_normals"]) != len(a["contact_positions_m"]):
        raise ValueError("contact_normals and contact_positions_m lengths differ")
    offsets = a["contact_offsets"]
    if offsets[0] != 0 or offsets[-1] != len(a["contact_positions_m"]) or np.any(np.diff(offsets) < 0):
        raise ValueError("contact_offsets do not delimit contacts")
    for prefix in ("epsilon", "task"):
        offsets, shapes = a[prefix + "_sample_offsets"], a[prefix + "_sample_shapes"]
        if (offsets[0] != 0 or offsets[-1] != len(a[prefix + "_samples"])
                or np.any(np.diff(offsets) < 0) or np.any(shapes < 0)
                or not np.array_equal(np.diff(offsets), np.prod(shapes, axis=1))):
            raise ValueError(f"{prefix}_sample_offsets do not delimit sample shapes")
    if np.any(a["epsilon_sample_shapes"] <= 0):
        raise ValueError("epsilon_sample_shapes must be nonempty")
    if a["friction_samples"].size and np.any(a["epsilon_sample_shapes"][:, 1] != a["friction_samples"].size):
        raise ValueError("epsilon_sample_shapes disagree with friction_samples")
    present = a["task_present"]
    if (np.any(a["task_sample_shapes"][~present] != 0)
            or not np.array_equal(a["task_sample_shapes"][present], a["epsilon_sample_shapes"][present])):
        raise ValueError("task_sample_shapes disagree with task_present or epsilon shapes")
    for key in ("nominal_task", "expected_task"):
        if np.any(a[key + "_present"] & ~present) or np.any(a[key][~a[key + "_present"]] != 0):
            raise ValueError(f"{key} disagrees with its presence mask")
    if not np.array_equal(a["score_kind"], np.where(present, "task_wrench_capacity", "epsilon")):
        raise ValueError("score_kind disagrees with task_present")


def load_dataset(npz_path, metadata_path=None) -> GraspDataset:
    """Load and validate schema 1.0, always with ``allow_pickle=False``."""
    path = Path(npz_path)
    meta = Path(metadata_path) if metadata_path is not None else path.with_suffix(".json")
    metadata = json.loads(meta.read_text(encoding="utf-8"))
    if not isinstance(metadata, dict):
        raise ValueError("dataset metadata must be a JSON object")
    try:
        with np.load(path, allow_pickle=False) as archive:
            arrays = {key: archive[key] for key in archive.files}
    except (ValueError, TypeError) as exc:
        raise ValueError(f"invalid non-pickled NPZ dataset: {exc}") from exc
    _validate(arrays, metadata)
    return GraspDataset(arrays, metadata)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="planner JSON payload")
    parser.add_argument("--out", type=Path, required=True, help="output .npz; metadata uses same stem .json")
    parser.add_argument("--hand-id", required=True)
    parser.add_argument("--object-id", required=True)
    parser.add_argument("--dof-names", required=True, help="comma-separated independent DOF names in source order")
    parser.add_argument("--mesh-scale", type=float, nargs="+", default=[1.0], help="one or three already-applied mesh scale factors")
    parser.add_argument("--model-file", action="append", default=[], metavar="LABEL=PATH", help="repeat for model and mesh dependencies to hash")
    parser.add_argument("--object-pointcloud-m", type=Path, help="caller-supplied object-frame (P,3) .npy in meters")
    args = parser.parse_args(argv)
    try:
        models = {}
        for item in args.model_file:
            label, separator, path = item.partition("=")
            if not separator or not label or not path or label in models:
                raise ValueError("--model-file requires unique LABEL=PATH entries")
            models[label] = Path(path)
        points = None
        if args.object_pointcloud_m:
            points = np.load(args.object_pointcloud_m, allow_pickle=False)
        scale = args.mesh_scale[0] if len(args.mesh_scale) == 1 else args.mesh_scale
        paths = export_dataset(json.loads(args.input.read_text(encoding="utf-8")), args.out,
                               hand_id=args.hand_id, object_id=args.object_id,
                               dof_names=args.dof_names.split(","), mesh_scale=scale,
                               model_files=models, object_pointcloud_m=points, source_path=args.input)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print(f"Wrote {paths[0]} and {paths[1]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
