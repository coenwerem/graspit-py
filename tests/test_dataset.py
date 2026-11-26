"""Portable static-grasp data contracts, independent of native GraspIt."""
from __future__ import annotations

import copy
import hashlib
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest


def source():
    # A 90-degree rotation catches accidental inversion or rotation of contacts.
    return {
        "robot": "hand.xml", "object": "mug.xml",
        "units": {"length": "mm", "angle": "rad", "quaternion": "wxyz"},
        "hand": {"approach_axis": [1, 0, 0]},
        "planner": {"friction_samples": [0.2, 0.8], "n_pose_samples": 1,
                    "tail_mass": 0.2, "seed": 7},
        "candidates": [{
            "grasp_id": "closed-17",
            "T_object_hand_mm": [[0, -1, 0, 123.456789012345],
                                 [1, 0, 0, -250], [0, 0, 1, 50], [0, 0, 0, 1]],
            "dofs_rad": [0.123456789012345, -0.4], "preshape": [0.1, -0.1],
            "contacts": [{"position_mm": [10, 20, -30], "normal": [0, -1, 0]}],
            "native_epsilon": -0.02, "nominal_epsilon": 0.4,
            "epsilon_samples": [[0.1, 0.4], [0, 0.2]],
            "score": 0.01, "expected_epsilon": 0.175, "epsilon_cvar": 0.01,
            "approach_dir": [0, 0, -1], "roll_deg": 90, "closing": [1],
            "task_samples": None, "nominal_task": None, "expected_task": None,
        }],
    }


def export(tmp_path, payload=None, **kwargs):
    api = importlib.import_module("graspit_py.dataset")
    paths = api.export_dataset(
        source() if payload is None else payload, tmp_path / "mug.npz",
        hand_id="test_hand", object_id="mug", dof_names=["flexion", "spread"],
        mesh_scale=[1, 2, 1], **kwargs,
    )
    return paths, api.load_dataset(paths[0])


def test_si_export_preserves_rotation_dof_order_and_object_frame(tmp_path):
    # Break caught: scaling all transform entries, inversion, or DOF sorting.
    (npz, metadata), dataset = export(tmp_path)
    a = dataset.arrays
    np.testing.assert_array_equal(a["T_object_hand_m"][0, :3, :3],
                                  [[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    np.testing.assert_array_equal(a["T_object_hand_m"][0, :3, 3],
                                  [123.456789012345 / 1000, -0.25, 0.05])
    np.testing.assert_array_equal(a["T_object_hand_m"][0, 3], [0, 0, 0, 1])
    np.testing.assert_array_equal(a["contact_positions_m"], [[0.01, 0.02, -0.03]])
    np.testing.assert_array_equal(a["contact_normals"], [[0, -1, 0]])
    assert a["dof_names"].tolist() == ["flexion", "spread"]
    assert a["dofs_rad"][0, 0] == 0.123456789012345
    np.testing.assert_array_equal(a["preshape_rad"], [[0.1, -0.1]])
    np.testing.assert_array_equal(a["closing_mask"], [[False, True]])
    assert a["grasp_ids"].tolist() == ["closed-17"]
    assert a["object_id"].item() == "mug"
    np.testing.assert_array_equal(a["mesh_scale"], [1, 2, 1])
    assert metadata == npz.with_suffix(".json")
    with np.load(npz, allow_pickle=False) as raw:
        assert all(raw[key].dtype.kind != "O" for key in raw.files)
        assert raw["dofs_rad"].dtype == np.float64


def test_ragged_contacts_and_samples_round_trip_with_optional_tasks(tmp_path):
    # Break caught: reshaping samples globally or treating absent task as zero quality.
    payload = source()
    other = copy.deepcopy(payload["candidates"][0])
    other.pop("grasp_id")
    other.update(contacts=[], closing=None, epsilon_samples=[[0.3, 0.6]],
                 task_samples=[[2.0, 4.0]], nominal_task=4.0, expected_task=3.0,
                 score=2.0)
    payload["candidates"].append(other)
    (_, _), d = export(tmp_path, payload)
    a = d.arrays
    np.testing.assert_array_equal(a["contact_offsets"], [0, 1, 1])
    np.testing.assert_array_equal(a["epsilon_sample_offsets"], [0, 4, 6])
    np.testing.assert_array_equal(a["epsilon_sample_shapes"], [[2, 2], [1, 2]])
    np.testing.assert_array_equal(a["epsilon_samples"], [0.1, 0.4, 0, 0.2, 0.3, 0.6])
    np.testing.assert_array_equal(a["task_present"], [False, True])
    np.testing.assert_array_equal(a["task_sample_offsets"], [0, 0, 2])
    np.testing.assert_array_equal(a["task_sample_shapes"], [[0, 0], [1, 2]])
    np.testing.assert_array_equal(a["task_samples"], [2, 4])
    np.testing.assert_array_equal(a["nominal_task_present"], [False, True])
    np.testing.assert_array_equal(a["expected_task_present"], [False, True])
    assert a["nominal_task"][1] == 4
    assert a["expected_task"][1] == 3
    assert a["score_kind"].tolist() == ["epsilon", "task_wrench_capacity"]
    assert a["grasp_ids"][1] == "test_hand/mug/000001"
    np.testing.assert_array_equal(a["closing_mask"][1], [True, True])


def test_metadata_keeps_provenance_and_separate_score_meanings(tmp_path):
    hand = tmp_path / "hand.xml"
    hand.write_bytes(b"verified hand bytes\n")
    points = [[0.001, 0, 0], [0, 0.02, 0]]
    (_, _), d = export(tmp_path, model_files={"hand": hand}, object_pointcloud_m=points)
    m, a = d.metadata, d.arrays
    assert m["source"]["planner"] == source()["planner"]
    assert m["source"]["hand"] == source()["hand"]
    assert m["source"]["units"] == source()["units"]
    canonical = json.dumps(source(), sort_keys=True, separators=(",", ":"), allow_nan=False)
    assert m["provenance"]["source_sha256"] == hashlib.sha256(canonical.encode()).hexdigest()
    assert len(m["provenance"]["config_sha256"]) == 64
    assert m["provenance"]["model_files"]["hand"]["sha256"] == hashlib.sha256(hand.read_bytes()).hexdigest()
    assert "native_epsilon" in m["score_meanings"]
    assert "nominal_epsilon" in m["score_meanings"]
    assert a["native_epsilon"][0] == -0.02
    assert a["nominal_epsilon"][0] == 0.4
    np.testing.assert_array_equal(a["object_pointcloud_m"], points)


@pytest.mark.parametrize("mutation,match", [
    (lambda p: p["candidates"][0]["T_object_hand_mm"][3].__setitem__(0, 1), "transform"),
    (lambda p: p["candidates"][0]["T_object_hand_mm"][0].__setitem__(0, 1), "rotation"),
    (lambda p: p["candidates"][0].__setitem__("dofs_rad", [0.2]), "dofs_rad"),
    (lambda p: p["candidates"][0]["dofs_rad"].__setitem__(0, float("nan")), "finite"),
    (lambda p: p["candidates"][0]["contacts"][0].__setitem__("normal", [0, 0, 0]), "normal"),
    (lambda p: p["candidates"][0].__setitem__("epsilon_samples", [[1], [2, 3]]), "epsilon_samples"),
    (lambda p: p["candidates"][0].__setitem__("task_samples", [[1, 2]]), "task_samples"),
    (lambda p: p["candidates"][0].__setitem__("closing", [2]), "closing"),
    (lambda p: p["units"].__setitem__("length", "m"), "units"),
])
def test_malformed_records_fail_before_writing(tmp_path, mutation, match):
    # Break caught: malformed geometry or labels silently entering training tensors.
    payload = source()
    mutation(payload)
    with pytest.raises(ValueError, match=match):
        export(tmp_path, payload)
    assert not (tmp_path / "mug.npz").exists()


def test_duplicate_dof_names_fail(tmp_path):
    api = importlib.import_module("graspit_py.dataset")
    with pytest.raises(ValueError, match="dof_names"):
        api.export_dataset(source(), tmp_path / "bad.npz", hand_id="hand", object_id="mug",
                           dof_names=["same", "same"])


def test_loader_rejects_schema_mismatch_and_bad_offsets(tmp_path):
    # Break caught: accepting a new or corrupt archive with the old tensor contract.
    (npz, meta), d = export(tmp_path)
    metadata = json.loads(meta.read_text())
    metadata["schema_version"] = "999.0"
    meta.write_text(json.dumps(metadata))
    api = importlib.import_module("graspit_py.dataset")
    with pytest.raises(ValueError, match="schema"):
        api.load_dataset(npz)
    metadata["schema_version"] = "1.0"
    meta.write_text(json.dumps(metadata))
    arrays = dict(d.arrays)
    arrays["contact_offsets"] = np.array([0, 7], dtype=np.int64)
    np.savez_compressed(npz, **arrays)
    with pytest.raises(ValueError, match="contact_offsets"):
        api.load_dataset(npz)


def test_public_cli_exports_without_native_graspit(tmp_path):
    # Break caught: public alias execution or dataset imports requiring native code.
    src, out = tmp_path / "input.json", tmp_path / "cli.npz"
    src.write_text(json.dumps(source()))
    command = [sys.executable, "-W", "error::RuntimeWarning", "-m", "graspit_py.dataset",
               "--input", str(src), "--out", str(out), "--hand-id", "test_hand",
               "--object-id", "mug", "--dof-names", "flexion,spread"]
    result = subprocess.run(command, capture_output=True, text=True, env=os.environ.copy())
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stderr == ""
    api = importlib.import_module("graspit_py.dataset")
    d = api.load_dataset(out)
    assert d.arrays["dofs_rad"].shape == (1, 2)
    assert d.metadata["provenance"]["source_file_sha256"] == hashlib.sha256(src.read_bytes()).hexdigest()
    check = subprocess.run([sys.executable, "-c", "import graspit_py.dataset, sys; "
                            "assert 'graspit_python_wrapper.core._bindings' not in sys.modules; "
                            "assert 'scipy' not in sys.modules"], capture_output=True, text=True)
    assert check.returncode == 0, check.stderr


@pytest.mark.parametrize("field,value", [
    ("grasp_ids", np.asarray("not-a-vector")),
    ("dof_names", np.asarray("not-a-vector")),
    ("dofs_rad", np.array([[np.inf, 0.2]])),
    ("T_object_hand_m", np.zeros((1, 4, 4))),
    ("contact_positions_m", np.array([[0.1, 0.2, 0.3]], dtype=object)),
])
def test_loader_rejects_malformed_archive_values(tmp_path, field, value):
    # Break caught: accepting invalid tensors, or crashing before schema validation.
    (npz, _), d = export(tmp_path)
    arrays = dict(d.arrays)
    arrays[field] = value
    np.savez_compressed(npz, **arrays)
    with pytest.raises(ValueError):
        importlib.import_module("graspit_py.dataset").load_dataset(npz)


def test_loader_rejects_nonfinite_metadata(tmp_path):
    # Break caught: loading a NaN source configuration despite finite tensor checks.
    (npz, meta), _ = export(tmp_path)
    metadata = json.loads(meta.read_text())
    metadata["source"]["planner"]["tail_mass"] = float("nan")
    meta.write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match="finite"):
        importlib.import_module("graspit_py.dataset").load_dataset(npz)


def test_empty_dataset_has_valid_ragged_boundaries(tmp_path):
    # Break caught: collapsing the zero-record shapes to one-dimensional arrays.
    payload = source()
    payload["candidates"] = []
    (_, _), d = export(tmp_path, payload)
    assert d.arrays["dofs_rad"].shape == (0, 2)
    assert d.arrays["epsilon_sample_shapes"].shape == (0, 2)
    assert d.arrays["contact_positions_m"].shape == (0, 3)
    np.testing.assert_array_equal(d.arrays["contact_offsets"], [0])


@pytest.mark.parametrize("kwargs,match", [
    ({"mesh_scale": 0}, "mesh_scale"),
    ({"object_pointcloud_m": [[1, 2]]}, "object_pointcloud_m"),
])
def test_bad_geometry_arguments_fail(tmp_path, kwargs, match):
    api = importlib.import_module("graspit_py.dataset")
    with pytest.raises(ValueError, match=match):
        api.export_dataset(source(), tmp_path / "bad.npz", hand_id="h", object_id="o",
                           dof_names=["a", "b"], **kwargs)


def test_cli_does_not_replace_its_source_with_metadata(tmp_path):
    # Break caught: same-stem sidecar overwriting the planner's original JSON.
    src = tmp_path / "original.json"
    original = json.dumps(source())
    src.write_text(original)
    result = subprocess.run(
        [sys.executable, "-m", "graspit_py.dataset", "--input", str(src),
         "--out", str(src.with_suffix(".npz")), "--hand-id", "h", "--object-id", "o",
         "--dof-names", "a,b"], capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert src.read_text() == original
    assert not src.with_suffix(".npz").exists()


def test_loader_rejects_malformed_metadata_field_schema(tmp_path):
    # Break caught: malformed field declarations escaping schema validation.
    (npz, meta), _ = export(tmp_path)
    metadata = json.loads(meta.read_text())
    metadata["fields"] = None
    meta.write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match="schema"):
        importlib.import_module("graspit_py.dataset").load_dataset(npz)


@pytest.mark.parametrize("section,key,value", [
    ("units", "length", "mm"),
    ("units", "dof_angle", "deg"),
    ("units", "roll_angle", "rad"),
    ("frames", "T_object_hand_m", "maps object coordinates into the hand frame"),
    ("frames", "contact_positions_m", "world"),
    ("frames", "contact_normals", "object, outward, unit length"),
    ("frames", "approach_dir", "world, hand direction of travel"),
    ("frames", "object_pointcloud_m", "camera, caller supplied"),
    (None, "record_kind", "trajectory"),
    (None, "mesh_scale", [1.0, 1.0, 1.0]),
])
def test_loader_rejects_contradictory_coordinate_metadata(tmp_path, section, key, value):
    # Break caught: correct tensors interpreted using conflicting units or frames.
    (npz, meta), _ = export(tmp_path)
    metadata = json.loads(meta.read_text())
    target = metadata if section is None else metadata[section]
    target[key] = value
    meta.write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match=section or key):
        importlib.import_module("graspit_py.dataset").load_dataset(npz)
