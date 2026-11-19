"""Export a MuJoCo hand as a GraspIt robot (palm + DH chains + OFF meshes).

The hand is read from a compiled ``mujoco.MjModel``, so anything MuJoCo can
load (MJCF, URDF, a whole scene with arms and objects) works; only the subtree
under ``palm_body`` is exported. Requirements on that subtree:

* every finger is a serial chain of bodies, each with exactly one hinge joint
  (bodies without joints are merged into their nearest jointed ancestor);
* joint couplings, if any, are MuJoCo joint equalities with a linear
  polynomial in displacements from the joints' reference angles.

Each hinge becomes a GraspIt revolute DH joint whose axis is the hinge axis
at the model's reference configuration (qpos0); link meshes are re-expressed
in the corresponding DH frames, so GraspIt link poses equal MuJoCo body poses
for every joint vector. Independent hinges become GraspIt DOFs in chain order;
coupled hinges use the driver's DOF and an offset in absolute joint coordinates.

Files: ``<out_dir>/<name>.xml`` (robot), ``<out_dir>/<name>.json`` (DOF and
link correspondence to MuJoCo names) and ``<out_dir>/iv/*.xml|off`` (links).

Geometry: collision geoms (nonzero contype or conaffinity) of type mesh, box,
sphere, capsule, cylinder or ellipsoid, written as OFF triangle meshes in mm.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    import mujoco
except ImportError as exc:  # pragma: no cover
    raise ImportError('graspit_python_wrapper.convert needs the `mujoco` package '
                      '(pip install graspit_python_wrapper[mjcf])') from exc

MM = 1000.0
_EPS = 1e-9


@dataclass
class ExportedJoint:
    mujoco_joint: str
    dof: int
    ratio: float
    offset_rad: float
    range_rad: Tuple[float, float]


@dataclass
class ExportedHand:
    robot_xml: Path
    dof_joints: List[str]                       # MuJoCo joint name of each GraspIt DOF
    chains: List[List[ExportedJoint]] = field(default_factory=list)
    links: Dict[Tuple[int, int], str] = field(default_factory=dict)  # (chain, link) -> MuJoCo body

    def to_json(self) -> dict:
        return dict(robot_xml=str(self.robot_xml), dof_joints=self.dof_joints,
                    chains=[[vars(j) for j in c] for c in self.chains],
                    links={f'{c},{l}': b for (c, l), b in self.links.items()})


# --------------------------------------------------------------- geometry
def _box(half):
    s = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float) * half
    f = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
         [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]
    return s, np.array(f)


def _uv_sphere(radii, n=16, z_shift=0.0, cap=None):
    th = np.linspace(0, np.pi, n + 1)
    ph = np.linspace(0, 2 * np.pi, 2 * n, endpoint=False)
    v = [[0, 0, radii[2]]]
    for t in th[1:-1]:
        for p in ph:
            v.append([radii[0] * np.sin(t) * np.cos(p), radii[1] * np.sin(t) * np.sin(p),
                      radii[2] * np.cos(t)])
    v.append([0, 0, -radii[2]])
    v = np.array(v)
    if cap is not None:                     # capsule: split hemispheres apart
        v[:, 2] += np.where(v[:, 2] >= 0, cap, -cap)
    v[:, 2] += z_shift
    f, m = [], len(ph)
    for j in range(m):
        f.append([0, 1 + j, 1 + (j + 1) % m])
    for i in range(n - 2):
        for j in range(m):
            a, b = 1 + i * m + j, 1 + i * m + (j + 1) % m
            f += [[a, a + m, b], [b, a + m, b + m]]
    last = len(v) - 1
    for j in range(m):
        f.append([last, 1 + (n - 2) * m + (j + 1) % m, 1 + (n - 2) * m + j])
    return v, np.array(f)


def _cylinder(r, h, n=32):
    ph = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.stack([r * np.cos(ph), r * np.sin(ph)], 1)
    v = np.vstack([np.c_[ring, np.full(n, -h)], np.c_[ring, np.full(n, h)], [[0, 0, -h], [0, 0, h]]])
    f = []
    for j in range(n):
        k = (j + 1) % n
        f += [[j, k, n + k], [j, n + k, n + j], [2 * n, k, j], [2 * n + 1, n + j, n + k]]
    return v, np.array(f)


def geom_mesh(model, g) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Vertices (geom frame, m) and triangles of geom ``g``, or None if unsupported."""
    T, size = mujoco.mjtGeom, model.geom_size[g]
    t = model.geom_type[g]
    if t == T.mjGEOM_MESH:
        mid = model.geom_dataid[g]
        va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        fa, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
        return model.mesh_vert[va:va + vn].astype(float), model.mesh_face[fa:fa + fn].astype(int)
    if t == T.mjGEOM_BOX:
        return _box(size)
    if t == T.mjGEOM_SPHERE:
        return _uv_sphere([size[0]] * 3)
    if t == T.mjGEOM_ELLIPSOID:
        return _uv_sphere(size)
    if t == T.mjGEOM_CAPSULE:
        return _uv_sphere([size[0]] * 3, cap=size[1])
    if t == T.mjGEOM_CYLINDER:
        return _cylinder(size[0], size[1])
    return None


def _write_off(path: Path, v: np.ndarray, f: np.ndarray) -> None:
    lines = ['OFF', f'{len(v)} {len(f)} 0']
    lines += [f'{x:.6f} {y:.6f} {z:.6f}' for x, y, z in v]
    lines += [f'3 {a} {b} {c}' for a, b, c in f]
    path.write_text('\n'.join(lines) + '\n')


# ------------------------------------------------------------------- DH
def _signed_angle(a, b, axis):
    return float(np.arctan2(np.dot(np.cross(a, b), axis), np.dot(a, b)))


def dh_from_axes(axes: Sequence[Tuple[np.ndarray, np.ndarray]], x_hint: np.ndarray,
                 parallel_tol: float = 1e-2):
    """DH parameters for a chain of revolute axes given as (point, unit direction).

    Returns (base 4x4, [(theta_offset, d, a, alpha)], [frame_k 4x4 at zero]).
    Frame k (after joint k) has its z along axis k+1; the last frame keeps
    the previous x and z. Axes whose directions differ by less than
    ``parallel_tol`` (sine of the angle) are treated as parallel.
    """
    p0, z0 = axes[0]
    x0 = x_hint - z0 * np.dot(x_hint, z0)
    if np.linalg.norm(x0) < 1e-6:
        x0 = np.cross(z0, [1.0, 0, 0]) if abs(z0[0]) < .9 else np.cross(z0, [0, 1.0, 0])
    x0 /= np.linalg.norm(x0)
    base = np.eye(4)
    base[:3, :3] = np.stack([x0, np.cross(z0, x0), z0], 1)
    base[:3, 3] = p0
    o, x, z = p0.copy(), x0, z0
    params, frames = [], []
    for k in range(len(axes)):
        if k + 1 < len(axes):
            p1, z1 = axes[k + 1]
            cz = np.cross(z, z1)
            if np.linalg.norm(cz) < parallel_tol:
                # (Nearly) parallel axes: the common normal's foot runs off to
                # infinity, so pin d = 0 at the current origin and drop the
                # out-of-plane residual of z1 (error ~ |cz| x link length).
                w = (p1 - o) - z * np.dot(p1 - o, z)
                d, a = 0.0, float(np.linalg.norm(w))
                xn = w / a if a > 1e-9 else x
                z1 = z1 - xn * np.dot(z1, xn)
                z1 = z1 / np.linalg.norm(z1)
            else:                                             # closest points of the two lines
                A = np.array([[z @ z, -z @ z1], [z @ z1, -z1 @ z1]])
                b = np.array([(p1 - o) @ z, (p1 - o) @ z1])
                t, s = np.linalg.solve(A, b)
                foot = o + t * z
                w = (p1 + s * z1) - foot
                d, a = float(t), float(np.linalg.norm(w))
                xn = w / a if a > 1e-9 else cz / np.linalg.norm(cz)
            alpha = _signed_angle(z, z1, xn)
        else:
            d = a = alpha = 0.0
            xn = x
            z1 = z
        theta = _signed_angle(x, xn, z)
        params.append((theta, d, a, alpha))
        o = o + d * z + a * xn
        x, z = xn, z1
        F = np.eye(4)
        F[:3, :3] = np.stack([x, np.cross(z, x), z], 1)
        F[:3, 3] = o
        frames.append(F)
    return base, params, frames


# ---------------------------------------------------------------- export
def _subtree(model, root):
    out = []
    for b in range(model.nbody):
        p = b
        while p not in (0, root):
            p = model.body_parentid[p]
        if p == root and b != root:
            out.append(b)
    return out


def _hinge(model, body):
    js = [j for j in range(model.njnt) if model.jnt_bodyid[j] == body]
    if not js:
        return None
    if len(js) > 1 or model.jnt_type[js[0]] != mujoco.mjtJoint.mjJNT_HINGE:
        raise ValueError(f'body {model.body(body).name!r} must have exactly one hinge joint')
    return js[0]


def _collision_geoms(model, bodies):
    """Collision geoms on ``bodies`` (zero-size geoms have no surface and are skipped)."""
    return [g for g in range(model.ngeom) if model.geom_bodyid[g] in bodies
            and (model.geom_contype[g] or model.geom_conaffinity[g])
            and model.geom_rbound[g] > 0.0]


def export_hand(model, palm_body: str, out_dir: str | Path, *, name: str = 'hand',
                material: str = 'rubber', palm_material: str = 'plastic',
                max_effort: float = 100.0,
                preshape_joints: Sequence[str] = ()) -> ExportedHand:
    """Write ``<out_dir>/<name>.xml`` plus link XML/OFF files; see the module docstring.

    DOFs driven by ``preshape_joints`` (MuJoCo joint names) get a zero default
    velocity, so GraspIt's autograsp holds them at their preshape value instead
    of sweeping them to a limit (e.g. a thumb opposition joint).
    """
    out_dir = Path(out_dir)
    link_dir = out_dir / 'iv'               # GraspIt resolves link files under <robot dir>/iv
    link_dir.mkdir(parents=True, exist_ok=True)
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    palm = model.body(palm_body).id
    Pp, PR = data.xpos[palm].copy(), data.xmat[palm].reshape(3, 3).copy()

    def to_palm_mm(points_world):
        return (points_world - Pp) @ PR * MM

    sub = _subtree(model, palm)
    jointed = [b for b in sub if _hinge(model, b) is not None]
    owner = {}                                     # body -> exported link body (or palm)
    for b in [palm] + sub:
        p = b
        while p != palm and p not in jointed:
            p = model.body_parentid[p]
        owner[b] = p
    roots = [b for b in jointed if owner[model.body_parentid[b]] == palm]
    chains = []
    for r in roots:
        chain, b = [r], r
        while True:
            kids = [c for c in jointed if owner[model.body_parentid[c]] == b]
            if not kids:
                break
            if len(kids) > 1:
                raise ValueError(f'body {model.body(b).name!r} branches; only serial fingers are supported')
            b = kids[0]
            chain.append(b)
        chains.append(chain)

    # couplings: driven joint -> (driver joint, c0, c1)
    coupled = {}
    for i in range(model.neq):
        if model.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT and model.eq_obj2id[i] >= 0:
            c0, c1 = model.eq_data[i, 0], model.eq_data[i, 1]
            if np.any(np.abs(model.eq_data[i, 2:5]) > _EPS):
                raise ValueError('only linear joint couplings are supported')
            coupled[int(model.eq_obj1id[i])] = (int(model.eq_obj2id[i]), float(c0), float(c1))
    dof_of, dof_joints = {}, []
    for chain in chains:
        for b in chain:
            j = _hinge(model, b)
            if j not in coupled:
                dof_of[j] = len(dof_joints)
                dof_joints.append(model.joint(j).name)
    for j, (parent, _, _) in coupled.items():
        if j in [_hinge(model, b) for c in chains for b in c] and parent not in dof_of:
            raise ValueError(f'joint {model.joint(j).name!r} is coupled to a non-DOF joint')

    hand = ExportedHand(out_dir / f'{name}.xml', dof_joints)

    def write_link(fname, bodies, frame_mm):
        verts, faces = [], []
        for g in _collision_geoms(model, bodies):
            res = geom_mesh(model, g)
            if res is None:
                continue
            v, f = res
            w = data.geom_xpos[g] + v @ data.geom_xmat[g].reshape(3, 3).T
            local = (to_palm_mm(w) - frame_mm[:3, 3]) @ frame_mm[:3, :3]
            faces.append(f + sum(len(x) for x in verts))
            verts.append(local)
        if not verts:
            raise ValueError(f'link {fname} has no collision geometry')
        _write_off(link_dir / f'{fname}.off', np.vstack(verts), np.vstack(faces))
        (link_dir / f'{fname}.xml').write_text(
            f'<?xml version="1.0" ?>\n<root>\n\t<material>{palm_material if fname == "palm" else material}'
            f'</material>\n\t<geometryFile type="off">{fname}.off</geometryFile>\n</root>\n')

    write_link('palm', [b for b in [palm] + sub if owner[b] == palm], np.eye(4))
    chain_xml = []
    x_hint = np.array([1.0, 0.0, 0.0])
    for ci, chain in enumerate(chains):
        axes = []
        for b in chain:
            j = _hinge(model, b)
            p = to_palm_mm(data.xanchor[j][None])[0]
            z = (data.xaxis[j] @ PR)
            axes.append((p, z / np.linalg.norm(z)))
        base, params, frames = dh_from_axes(axes, x_hint)
        exported, joints_xml, links_xml = [], [], []
        for li, (b, (theta, d, a, alpha), F) in enumerate(zip(chain, params, frames)):
            j = _hinge(model, b)
            lo, hi = model.jnt_range[j] if model.jnt_limited[j] else (-np.pi, np.pi)
            q0 = model.qpos0[model.jnt_qposadr[j]]
            if j in coupled:
                parent, c0, c1 = coupled[j]
                dof, ratio = dof_of[parent], c1
                parent_q0 = model.qpos0[model.jnt_qposadr[parent]]
                # MuJoCo constrains displacements from qpos0.
                off = float(q0 + c0 - ratio * parent_q0)
            else:
                dof, ratio, off = dof_of[j], 1.0, 0.0
            # theta(q) = q + theta_dh - q0 with q = ratio * dof + off
            const = np.degrees(theta + off - q0)
            jlo, jhi = lo - off, hi - off          # GraspIt limits apply to ratio * dof
            expr = f'd{dof}' + (f'*{ratio:.9f}' if abs(ratio - 1) > 1e-12 else '') + f'+{const:.9f}'
            joints_xml.append(
                f"\t\t<joint type='Revolute'>\n\t\t\t<theta>{expr}</theta>\n\t\t\t<d>{d:.9f}</d>\n"
                f"\t\t\t<a>{a:.9f}</a>\n\t\t\t<alpha>{np.degrees(alpha):.9f}</alpha>\n"
                f"\t\t\t<minValue>{np.degrees(jlo):.6f}</minValue>\n\t\t\t<maxValue>{np.degrees(jhi):.6f}</maxValue>\n"
                f"\t\t\t<viscousFriction>5.0e+7</viscousFriction>\n\t\t</joint>\n")
            fname = model.body(b).name or f'c{ci}l{li}'
            write_link(fname, [x for x in [palm] + sub if owner[x] == b], F)
            links_xml.append(f"\t\t<link dynamicJointType='Revolute'>{fname}.xml</link>\n")
            exported.append(ExportedJoint(model.joint(j).name, dof, ratio, off, (float(lo), float(hi))))
            hand.links[(ci, li)] = model.body(b).name
        hand.chains.append(exported)
        R = base[:3, :3].T.reshape(-1)         # GraspIt stores the transpose
        chain_xml.append(
            "\t<chain>\n\t\t<transform>\n"
            f"\t\t\t<translation>{' '.join(f'{x:.9f}' for x in base[:3, 3])}</translation>\n"
            f"\t\t\t<rotationMatrix>{' '.join(f'{x:.12f}' for x in R)}</rotationMatrix>\n"
            "\t\t</transform>\n" + ''.join(joints_xml) + ''.join(links_xml) + "\t</chain>\n")
    unknown = set(preshape_joints) - set(dof_joints)
    if unknown:
        raise ValueError(f'preshape joints {sorted(unknown)} are not independent DOFs')
    dof_xml = ''.join(
        f"\t<dof type='r'>\n\t\t<defaultVelocity>{0 if n in preshape_joints else 1}</defaultVelocity>\n"
        f"\t\t<maxEffort>{max_effort}</maxEffort>\n"
        "\t\t<Kp>1e+09</Kp>\n\t\t<Kd>1e+07</Kd>\n\t\t<draggerScale>20</draggerScale>\n\t</dof>\n"
        for n in dof_joints)
    hand.robot_xml.write_text('<?xml version="1.0" ?>\n<robot type="Hand">\n\t<palm>palm.xml</palm>\n'
                              + dof_xml + ''.join(chain_xml) + '</robot>\n')
    (out_dir / f'{name}.json').write_text(json.dumps(hand.to_json(), indent=1))
    return hand


def dofs_from_qpos(hand: ExportedHand, model, qpos: np.ndarray) -> np.ndarray:
    """GraspIt DOF vector for a MuJoCo ``qpos`` of ``model``."""
    return np.array([qpos[model.jnt_qposadr[model.joint(n).id]] for n in hand.dof_joints])


def export_body(model, body: str, out_dir: str | Path, *, name: Optional[str] = None,
                material: str = 'plastic', include_descendants: bool = True) -> Path:
    """Write a MuJoCo body's collision geometry as a GraspIt graspable object.

    Geometry is expressed in the body frame (mm), so a GraspIt object at the
    identity pose coincides with the MuJoCo body frame; mass and centre of mass
    are copied. Returns the object XML path.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    name = name or body
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    root = model.body(body).id
    bodies = [root] + (_subtree(model, root) if include_descendants else [])
    P, R = data.xpos[root], data.xmat[root].reshape(3, 3)
    verts, faces = [], []
    for g in _collision_geoms(model, bodies):
        res = geom_mesh(model, g)
        if res is None:
            continue
        v, f = res
        w = data.geom_xpos[g] + v @ data.geom_xmat[g].reshape(3, 3).T
        faces.append(f + sum(len(x) for x in verts))
        verts.append((w - P) @ R * MM)
    if not verts:
        raise ValueError(f'body {body!r} has no collision geometry')
    _write_off(out_dir / f'{name}.off', np.vstack(verts), np.vstack(faces))
    mass_g = float(sum(model.body_mass[b] for b in bodies)) * 1000.0
    com = sum(model.body_mass[b] * ((data.xipos[b] - P) @ R) for b in bodies) / max(
        1e-12, sum(model.body_mass[b] for b in bodies)) * MM
    xml = out_dir / f'{name}.xml'
    xml.write_text('<?xml version="1.0" ?>\n<root>\n'
                   f'\t<material>{material}</material>\n\t<mass>{mass_g:.6f}</mass>\n'
                   f'\t<cog>{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}</cog>\n'
                   f'\t<geometryFile type="off">{name}.off</geometryFile>\n</root>\n')
    return xml
