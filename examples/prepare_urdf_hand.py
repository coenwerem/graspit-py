"""Compile a URDF into MJCF with explicit affine mimic-joint equalities.

This geometry-preparation example does not build actuators or tendon dynamics.
Use --geometry-only to derive inertias from collisions for static export checks.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def mimic_polynomial(offset, ratio, driven_reference, driver_reference):
    """Convert q1 = offset + ratio*q2 to MuJoCo reference-displacement form."""
    return offset - driven_reference + ratio * driver_reference, ratio


def apply_profile_mimics(root, couplings):
    """Apply explicitly declared kinematic couplings without replacing source values."""
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    for name, coupling in couplings.items():
        if name not in joints or coupling['driver'] not in joints:
            raise ValueError(f'{name}: profile joint is absent from source URDF')
        joint = joints[name]
        existing = joint.find('mimic')
        if existing is not None:
            values = (float(existing.get('multiplier', '1')), float(existing.get('offset', '0')))
            expected = (coupling['ratio'], coupling['offset_rad'])
            if existing.get('joint') != coupling['driver'] or any(
                    not math.isclose(a, b, abs_tol=1e-12, rel_tol=0)
                    for a, b in zip(values, expected)):
                raise ValueError(f'{name}: source mimic conflicts with the planning profile')
        else:
            ET.SubElement(joint, 'mimic', joint=coupling['driver'],
                          multiplier=str(coupling['ratio']), offset=str(coupling['offset_rad']))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--geometry-only', action='store_true',
                        help='discard source inertias for a static collision-geometry export')
    parser.add_argument('--package-root', action='append', default=[], metavar='NAME=PATH')
    parser.add_argument('--profile', type=Path,
                        help='apply explicitly declared affine planning couplings')
    args = parser.parse_args()
    import mujoco
    source = args.urdf.resolve()
    packages = dict(item.split('=', 1) for item in args.package_root)
    tree = ET.parse(source)
    root = tree.getroot()
    if args.profile:
        profile = json.loads(args.profile.read_text())
        apply_profile_mimics(root, profile.get('mimic_joints', {}))
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    mimics = []
    for name, joint in joints.items():
        mimic = joint.find('mimic')
        if mimic is None:
            continue
        driver = mimic.get('joint')
        if driver not in joints or joints[driver].find('mimic') is not None:
            raise ValueError(f'{name}: mimic driver must be an independent named joint')
        if joint.get('type') not in ('revolute', 'continuous') or joints[driver].get('type') not in ('revolute', 'continuous'):
            raise ValueError(f'{name}: only rotational mimic joints are supported')
        mimics.append(dict(driven=name, driver=driver,
                           ratio=float(mimic.get('multiplier', '1')),
                           offset_rad=float(mimic.get('offset', '0'))))
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            link.remove(visual)
        if args.geometry_only:
            for inertia in link.findall('inertial'):
                link.remove(inertia)
    for mesh in root.findall('.//mesh'):
        filename = mesh.get('filename')
        if filename.startswith('package://'):
            package, relative = filename[len('package://'):].split('/', 1)
            if package not in packages:
                raise ValueError(f'provide --package-root {package}=PATH')
            resolved = Path(packages[package]) / relative
        else:
            resolved = source.parent / filename
        if not resolved.is_file():
            raise FileNotFoundError(resolved)
        mesh.set('filename', str(resolved.resolve()))
    mj = root.find('mujoco')
    if mj is None:
        mj = ET.SubElement(root, 'mujoco')
    compiler = mj.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(mj, 'compiler')
    compiler.attrib.update(fusestatic='false', strippath='false', discardvisual='true')
    args.output.mkdir(parents=True, exist_ok=True)
    prepared = args.output / 'prepared.urdf'
    scene = args.output / 'scene.xml'
    tree.write(prepared, encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(prepared))
    mujoco.mj_saveLastXML(str(scene), model)
    tree = ET.parse(scene)
    equality = ET.SubElement(tree.getroot(), 'equality')
    for mimic in mimics:
        driven, driver = model.joint(mimic['driven']), model.joint(mimic['driver'])
        c0, c1 = mimic_polynomial(mimic['offset_rad'], mimic['ratio'],
                                 model.qpos0[driven.qposadr[0]], model.qpos0[driver.qposadr[0]])
        ET.SubElement(equality, 'joint', joint1=mimic['driven'], joint2=mimic['driver'],
                      polycoef=f'{c0:.17g} {c1:.17g} 0 0 0')
    tree.write(scene, encoding='unicode')
    mujoco.MjModel.from_xml_path(str(scene))
    record = dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
                  geometry_only=args.geometry_only, mimics=mimics, mujoco=mujoco.__version__,
                  simulation_steps=0)
    if args.profile:
        record['profile_sha256'] = hashlib.sha256(args.profile.read_bytes()).hexdigest()
    (args.output / 'preparation.json').write_text(json.dumps(record, indent=2) + '\n')
    print(scene)


if __name__ == '__main__':
    main()
