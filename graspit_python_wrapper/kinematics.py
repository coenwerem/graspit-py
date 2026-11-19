"""Forward kinematics of a GraspIt robot XML without the native library.

Reproduces GraspIt's link placement for hands whose chains use Denavit-
Hartenberg joints: each chain starts at its `<transform>` (the stored
`rotationMatrix` is the transpose of the column-vector rotation) and each
joint contributes Rz(theta) Tz(d) Tx(a) Rx(alpha), with theta = coupling
ratio x DOF value (degrees) + the constant in `<theta>` (`d<k>*<ratio>+<deg>`).
Each chain's i-th joint carries its i-th link. Units are those of the XML (mm).
Useful for comparing a GraspIt hand model with another simulator's model
of the same hand without running GraspIt.
"""
from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

_THETA = re.compile(r'^\s*d(\d+)(?:\*([0-9.-]+))?(?:\+(-?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?))?\s*$')


@dataclass(frozen=True)
class DHJoint:
    dof: int
    theta_offset_deg: float
    d: float
    a: float
    alpha_deg: float
    ratio: float = 1.0          # joint value = ratio * DOF value


@dataclass(frozen=True)
class Chain:
    base: np.ndarray            # 4x4, palm frame
    joints: Tuple[DHJoint, ...]


def _rot(axis: str, deg: float) -> np.ndarray:
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    R = np.eye(4)
    if axis == 'z':
        R[:2, :2] = [[c, -s], [s, c]]
    else:
        R[1:3, 1:3] = [[c, -s], [s, c]]
    return R


def _trans(x=0.0, z=0.0) -> np.ndarray:
    T = np.eye(4)
    T[0, 3], T[2, 3] = x, z
    return T


def load_chains(robot_xml: str | Path) -> List[Chain]:
    """Parse the DH chains of a GraspIt robot XML."""
    root = ET.parse(robot_xml).getroot()
    chains = []
    for c in root.findall('chain'):
        tr = c.find('transform')
        base = np.eye(4)
        if tr is not None:
            t = tr.find('translation')
            r = tr.find('rotationMatrix')
            if t is not None:
                base[:3, 3] = np.array(t.text.split(), float)
            if r is not None:
                base[:3, :3] = np.array(r.text.split(), float).reshape(3, 3).T
            if tr.find('rotation') is not None or tr.find('fullTransform') is not None:
                raise NotImplementedError('only translation/rotationMatrix chain transforms are supported')
        joints = []
        for j in c.findall('joint'):
            if j.get('type') != 'Revolute':
                raise NotImplementedError(f"joint type {j.get('type')!r} is not supported")
            m = _THETA.match(j.find('theta').text)
            if m is None:
                raise NotImplementedError(f"theta expression {j.find('theta').text!r} is not supported")
            joints.append(DHJoint(int(m.group(1)), float(m.group(3) or 0.0),
                                  float(j.find('d').text), float(j.find('a').text),
                                  float(j.find('alpha').text), float(m.group(2) or 1.0)))
        chains.append(Chain(base, tuple(joints)))
    return chains


def link_poses(chains: Sequence[Chain], dofs: Sequence[float]) -> Dict[Tuple[int, int], np.ndarray]:
    """4x4 pose of every chain link in the palm frame for DOF values in radians."""
    q = np.asarray(dofs, float)
    out = {}
    for ci, chain in enumerate(chains):
        T = chain.base.copy()
        for li, j in enumerate(chain.joints):
            theta = np.degrees(j.ratio * q[j.dof]) + j.theta_offset_deg
            T = T @ _rot('z', theta) @ _trans(z=j.d) @ _trans(x=j.a) @ _rot('x', j.alpha_deg)
            out[(ci, li)] = T.copy()
    return out
