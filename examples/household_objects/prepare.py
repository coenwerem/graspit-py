"""Prepare an OBJ object for GraspIt! and sample its surface in meters.

Supply the metric scale and source attribution. The output object keeps the
source axes, with its origin at the source axis-aligned bounding-box center.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from graspit_python_wrapper.convert.mjcf_hand import export_body


def prepare(source: Path, out: Path, object_id: str, points: int = 2048, *, scale_m: float = 1.0,
            dataset: str = "User-supplied mesh", license_name: str = "Unspecified",
            source_url: str | None = None, source_citation: str | None = None):
    vertices = np.array([[float(v) for v in line.split()[1:4]]
                         for line in source.read_text().splitlines() if line.startswith('v ')])
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all():
        raise ValueError('The OBJ must contain finite 3D vertices')
    if not np.isfinite(scale_m) or scale_m <= 0:
        raise ValueError("scale_m must be positive and finite")
    vertices *= scale_m
    center = (vertices.min(0) + vertices.max(0)) / 2
    root = ET.Element('mujoco', model=object_id)
    ET.SubElement(root, 'compiler', angle='radian')
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'mesh', name='payload_mesh', file=str(source.resolve()),
                  scale=' '.join([str(scale_m)]*3))
    body = ET.SubElement(ET.SubElement(root, 'worldbody'), 'body', name='payload')
    # Nominal mass permits model compilation. This workflow evaluates static
    # contact geometry and normalized wrench scores, not dynamic lift outcomes.
    ET.SubElement(body, 'geom', type='mesh', mesh='payload_mesh',
                  pos=' '.join(f'{x:.17g}' for x in -center), mass='0.1')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    out.mkdir(parents=True, exist_ok=True)
    xml = export_body(model, 'payload', out, name=object_id, mesh_collision='source')
    lines = (out / f'{object_id}.off').read_text().splitlines()
    nv, nf, _ = map(int, lines[1].split())
    verts = np.array([list(map(float, line.split())) for line in lines[2:2+nv]]) / 1000
    faces = np.array([list(map(int, line.split()[1:])) for line in lines[2+nv:2+nv+nf]])
    triangles = verts[faces]
    areas = np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0], triangles[:,2]-triangles[:,0]),axis=1)/2
    rng = np.random.default_rng(0)
    selected = triangles[rng.choice(len(faces), points, p=areas/areas.sum())]
    uv = rng.random((points,2)); uv[uv.sum(1)>1] = 1-uv[uv.sum(1)>1]
    cloud = selected[:,0] + uv[:,:1]*(selected[:,1]-selected[:,0]) + uv[:,1:]*(selected[:,2]-selected[:,0])
    np.save(out/'object_pointcloud_m.npy',cloud,allow_pickle=False)
    expected = np.ptp(vertices,axis=0)
    actual = np.ptp(verts,axis=0)
    if not np.allclose(actual,expected,atol=2e-7,rtol=0):
        raise ValueError('Object preparation changed source extents')
    report = dict(object_id=object_id, source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  source_length_unit='source mesh coordinates', mesh_scale=scale_m, source_to_object_translation_m=(-center).tolist(),
                  object_frame='source axes; origin at source axis-aligned bounding-box center',
                  extents_m=actual.tolist(), pointcloud_points=points, pointcloud_seed=0,
                  object_cog_m=(np.array(list(map(float,ET.parse(xml).findtext('cog').split())))/1000).tolist(),
                  source_vertices=nv, source_triangles=nf, nominal_mass_kg=0.1,
                  mass_role='model compilation only; no dynamic validation',
                  source_dataset=dataset, source_license=license_name,
                  source_citation=source_citation, source_url=source_url, object_xml=str(xml))
    (out/'object_metadata.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--object-id',required=True)
    p.add_argument('--scale-m',type=float,default=1.0,help='meters per source mesh coordinate unit')
    p.add_argument('--dataset',default='User-supplied mesh')
    p.add_argument('--license',default='Unspecified',dest='license_name')
    p.add_argument('--source-url')
    a=p.parse_args(); prepare(a.source,a.out,a.object_id,scale_m=a.scale_m,
        dataset=a.dataset,license_name=a.license_name,source_url=a.source_url)

if __name__=='__main__': main()
