"""URDF helpers.

GraspIt does not natively load URDF.

For *objects*, we can convert a URDF to a GraspIt object XML by:
- loading the URDF as a scene
- exporting a combined mesh to PLY (.ply) (supported by GraspIt)
- writing a GraspIt object XML that references that PLY file

For *robots/hands*, GraspIt expects its own robot XML format; that conversion is
non-trivial and is not handled here.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import hashlib
import os
import tempfile
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class UrdfImportResult:
    graspit_object_xml: Path
    temp_dir: Path


def urdf_to_graspit_object_xml(
    urdf_path: Path,
    *,
    name: Optional[str] = None,
    mass: float = 300.0,
    scale: float = 1.0,
) -> Tuple[Path, Path]:
    """Convert a URDF into a GraspIt object XML + geometry file.

    Args:
        urdf_path: Path to URDF file.
        name: Optional name for the object (defaults to URDF stem).
        mass: Mass in grams (default 300).
        scale: Scale factor for mesh (default 1.0). Use to convert meters->mm (scale=1000) or vice versa.

    Returns:
        (xml_path, temp_dir)

    Notes:
        This is best-effort: URDFs with complex materials, multiple collision
        geometries, or missing mesh assets may fail.
    """

    urdf_path = Path(urdf_path)
    if urdf_path.suffix.lower() != ".urdf":
        raise ValueError(f"Expected a .urdf file, got: {urdf_path}")
    if not urdf_path.exists():
        raise FileNotFoundError(str(urdf_path))

    stem = name or urdf_path.stem

    # Use a stable temp directory name so repeated runs are easier to inspect.
    digest = hashlib.sha256(str(urdf_path.resolve()).encode("utf-8")).hexdigest()[:10]
    base = Path(tempfile.mkdtemp(prefix=f"graspit_urdf_{stem}_{digest}_"))

    rewritten_urdf = base / urdf_path.name
    _rewrite_urdf_mesh_filenames(urdf_path, rewritten_urdf)

    mesh_path = base / f"{stem}.off"
    _export_urdf_to_off(rewritten_urdf, mesh_path, scale=scale)

    xml_path = base / f"{stem}.xml"
    _write_graspit_object_xml(
        xml_path,
        geometry_filename=mesh_path.name,
        geometry_type="off",
        mass=mass,
    )

    return xml_path, base


def _rewrite_urdf_mesh_filenames(src_urdf: Path, dst_urdf: Path) -> None:
    """Rewrite package:// URIs to absolute paths (best-effort).

    Trimesh/URDF loaders often don't resolve ROS package URIs; rewriting them to
    concrete paths improves portability.
    """

    tree = ET.parse(str(src_urdf))
    root = tree.getroot()

    urdf_dir = src_urdf.parent

    for mesh in root.iter("mesh"):
        filename = mesh.attrib.get("filename") or mesh.attrib.get("name")
        if not filename:
            continue

        resolved = _resolve_urdf_uri(filename, urdf_dir)
        if resolved is None:
            continue

        mesh.attrib["filename"] = str(resolved)

    dst_urdf.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(dst_urdf), encoding="utf-8", xml_declaration=True)


def _resolve_urdf_uri(uri: str, base_dir: Path) -> Optional[Path]:
    # Absolute or relative file path.
    if uri.startswith("file://"):
        return Path(uri[len("file://") :])

    if uri.startswith("package://"):
        # package://<pkg>/<path>
        remainder = uri[len("package://") :]
        parts = remainder.split("/", 1)
        if not parts or not parts[0]:
            return None
        pkg = parts[0]
        rel = parts[1] if len(parts) > 1 else ""

        pkg_root = _find_package_dir(pkg, base_dir)
        if pkg_root is None:
            return None
        return (pkg_root / rel).resolve()

    p = Path(uri)
    if p.is_absolute():
        return p

    # Relative path (relative to the URDF file location).
    return (base_dir / p).resolve()


def _find_package_dir(package_name: str, start_dir: Path) -> Optional[Path]:
    # Common case in this workspace: the URDF lives inside the package dir.
    cur = start_dir.resolve()

    # If the current directory *is* the package.
    if cur.name == package_name:
        return cur

    # Walk upwards and check for a sibling directory named package.
    for _ in range(10):
        candidate = cur / package_name
        if candidate.exists() and candidate.is_dir():
            return candidate
        if cur.parent == cur:
            break
        cur = cur.parent

    # ROS-style search path.
    ros_paths = os.environ.get("ROS_PACKAGE_PATH")
    if ros_paths:
        for root in ros_paths.split(":"):
            candidate = Path(root) / package_name
            if candidate.exists() and candidate.is_dir():
                return candidate

    return None



def _export_urdf_to_off(urdf_path: Path, out_off: Path, scale: float = 1.0) -> None:
    """Load URDF and export combined visual geometry to OFF (.off).

    Args:
        urdf_path: Path to URDF file.
        out_off: Output OFF file path.
        scale: Scale factor to apply to the mesh (default 1.0).
    """

    # Preferred path: `yourdfpy` can load URDFs and produce a trimesh.Scene.
    try:
        from yourdfpy import URDF  # type: ignore
        import trimesh

        robot = URDF.load(str(urdf_path))
        scene = robot.scene
        meshes = scene.dump(concatenate=False)
        meshes = [m for m in meshes if m is not None and getattr(m, "vertices", None) is not None]
        if not meshes:
            raise RuntimeError("URDF produced no visual geometry meshes")

        combined = trimesh.util.concatenate(meshes)
        if scale != 1.0:
            combined.apply_scale(scale)
        out_off.parent.mkdir(parents=True, exist_ok=True)
        combined.export(str(out_off), file_type="off")
        return
    except Exception:
        # Fall back below.
        pass

    # Fallback: try trimesh's loader (many builds do NOT support URDF).
    try:
        import trimesh
    except Exception as e:  # pragma: no cover
        raise RuntimeError(
            "URDF import requires `yourdfpy` (recommended) or a trimesh build with URDF support. "
            "Install with `pip install yourdfpy trimesh`."
        ) from e

    try:
        scene_or_mesh = trimesh.load(str(urdf_path), force="scene")
    except Exception as e:
        raise RuntimeError(
            "Failed to load URDF. Install `yourdfpy` (recommended). "
            "Also check URDF mesh paths (package:// URIs require ROS_PACKAGE_PATH) and that referenced meshes exist."
        ) from e

    meshes = scene_or_mesh.dump(concatenate=False) if hasattr(scene_or_mesh, "dump") else [scene_or_mesh]
    meshes = [m for m in meshes if m is not None and getattr(m, "vertices", None) is not None]
    if not meshes:
        raise RuntimeError("URDF produced no geometry meshes")

    combined = trimesh.util.concatenate(meshes)
    if scale != 1.0:
        combined.apply_scale(scale)
    out_off.parent.mkdir(parents=True, exist_ok=True)
    combined.export(str(out_off), file_type="off")


def _write_graspit_object_xml(
    xml_path: Path,
    *,
    geometry_filename: str,
    geometry_type: str,
    mass: float,
) -> None:
    xml_path.parent.mkdir(parents=True, exist_ok=True)
    xml = f"""<?xml version=\"1.0\" ?>
<root>
\t<material>plastic</material>
\t<useFlockOfBirds>1.0</useFlockOfBirds>
\t<mass>{mass:g}</mass>
\t<cog>0 0 0</cog>
\t<inertia_matrix>416.64 0.0 0.0 0.0 416.64 0.0 0.0 0.0 416.64</inertia_matrix>
	<geometryFile type=\"{geometry_type}\">{geometry_filename}</geometryFile>
</root>
"""
    xml_path.write_text(xml, encoding="utf-8")
