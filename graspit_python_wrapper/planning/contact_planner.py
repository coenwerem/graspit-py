"""Risk-sensitive contact planning with GraspIt's quasi-static closure model.

A candidate grasp is a hand pose in the object frame together with the
contact set that GraspIt's closure produces from it. Each candidate is
scored on a joint distribution over

* execution error: Gaussian perturbations of the pre-closure hand pose,
  each followed by a fresh closure, so the contact set itself varies; and
* friction: coefficients drawn from a prior, applied to every contact set
  through a linearized friction-cone grasp wrench space.

The score is the lower-tail mean (CVaR) of a grasp quality over those
samples: the Ferrari-Canny epsilon by default, or, when the task's loads are
given as forces at points on the object, the task wrench quality (the largest
multiple of the worst task load the grasp resists). A tail mass of 1 recovers
the expected quality (risk-neutral planning). Candidates come from approach directions on a sphere around the
object, each approached until first contact and then closed with the hand's
autograsp. An optional support plane rejects grasps whose hand geometry
would pass below a table, and an optional contact region (an object-frame box)
rejects closures that touch the object anywhere else, e.g. to keep a tool's
working end free.

Units follow GraspIt: millimeters, radians, quaternions as (w, x, y, z).
"""
from __future__ import annotations

import zlib
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from ..core.types import ContactPoint
from ..grasp.quality import (analyze_gws, build_grasp_matrix_from_contacts,
                              compute_task_wrench_quality, task_wrenches_from_forces)
from ..risk import lower_tail_mean


@dataclass
class HandFrame:
    """Where the hand grasps, in the hand base frame (GraspIt units)."""

    approach_axis: Sequence[float] = (1.0, 0.0, 0.0)  # palm normal, toward the object
    finger_axis: Sequence[float] = (0.0, 0.0, 1.0)    # base-to-fingertip direction
    grasp_center_mm: Sequence[float] = (40.0, 10.0, 93.0)  # typical object centre at closure


@dataclass
class PlannerConfig:
    n_directions: int = 48
    n_rolls: int = 4
    standoff_mm: float = 120.0
    approach_step_mm: float = 2.0
    clearance_mm: float = 2.0
    min_contacts: int = 3
    n_pose_samples: int = 8
    pose_sigma_mm: float = 3.0
    pose_sigma_deg: float = 3.0
    friction_samples: Sequence[float] = field(default_factory=lambda: list(np.linspace(0.2, 1.0, 9)))
    tail_mass: float = 0.2
    support_normal: Optional[Sequence[float]] = None  # object frame, pointing away from the table
    support_offset_mm: float = 0.0                    # plane: n . x >= offset
    contact_region_mm: Optional[Sequence[Sequence[float]]] = None  # [[min xyz], [max xyz]], object frame
    keepout_boxes_mm: Optional[Sequence[Sequence[Sequence[float]]]] = None  # [[centre xyz], [half xyz]] per box, object frame
    autograsp_speed: float = 0.8
    preshapes: Optional[Sequence[Sequence[float]]] = None  # open DOF vectors to try (default: open_dofs)
    closing_sets: Optional[Sequence[Sequence[int]]] = None  # DOF index sets that close; the rest hold the preshape
    approaches: Optional[Sequence[Sequence[float]]] = None  # [dx, dy, dz, roll_deg] rows; overrides the sphere
    task_forces_mm: Optional[Sequence[Sequence[float]]] = None  # [px, py, pz, fx, fy, fz] rows, object frame;
                                                                # scores the task wrench quality instead of epsilon
    seed: int = 0


@dataclass
class GraspCandidate:
    T_object_hand: np.ndarray         # 4x4, mm
    dofs: np.ndarray                  # closed hand, radians, GraspIt DOF order
    contacts: List[ContactPoint]      # object frame, mm, inward normals
    native_epsilon: float
    nominal_epsilon: float            # at the friction prior mean, nominal pose
    epsilon_samples: np.ndarray       # (n_pose_samples + 1, n_friction)
    score: float
    expected_epsilon: float
    approach_dir: np.ndarray
    roll_deg: float
    preshape: np.ndarray
    closing: Optional[List[int]] = None   # DOFs that closed (None: every DOF)
    epsilon_cvar: float = 0.0             # CVaR of epsilon (equals score when no task is given)
    task_samples: Optional[np.ndarray] = None   # task wrench quality, same layout as epsilon_samples
    nominal_task: Optional[float] = None
    expected_task: Optional[float] = None

    def to_dict(self) -> dict:
        return dict(T_object_hand_mm=self.T_object_hand.tolist(), dofs_rad=self.dofs.tolist(),
                    contacts=[dict(position_mm=c.position.tolist(), normal=c.normal.tolist())
                              for c in self.contacts],
                    native_epsilon=self.native_epsilon, nominal_epsilon=self.nominal_epsilon,
                    epsilon_samples=self.epsilon_samples.tolist(), score=self.score,
                    expected_epsilon=self.expected_epsilon,
                    approach_dir=self.approach_dir.tolist(), roll_deg=self.roll_deg,
                    preshape=self.preshape.tolist(), closing=self.closing, epsilon_cvar=self.epsilon_cvar,
                    task_samples=None if self.task_samples is None else self.task_samples.tolist(),
                    nominal_task=self.nominal_task, expected_task=self.expected_task)


def fibonacci_directions(n: int) -> np.ndarray:
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    theta = np.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)], 1)


def _pose(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def palm_pose_for_approach(direction, roll_deg: float, standoff_mm: float, hand: HandFrame,
                           center_mm) -> np.ndarray:
    """T_object_hand (mm) for an approach along ``direction`` (object frame, the
    hand's direction of travel) with the hand's grasp centre on the approach line
    through ``center_mm``, ``standoff_mm`` short of it, rolled by ``roll_deg``
    about the approach. Pure NumPy, so arm reachability can be checked without
    GraspIt for exactly the poses the planner will use."""
    a = np.asarray(hand.approach_axis, float)
    f = np.asarray(hand.finger_axis, float)
    f = f - a * (f @ a) / (a @ a)
    basis = np.stack([a / np.linalg.norm(a), f / np.linalg.norm(f),
                      np.cross(a, f) / np.linalg.norm(np.cross(a, f))], 1)
    d = np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    ref = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(d, ref)
    u /= np.linalg.norm(u)
    u = Rotation.from_rotvec(np.radians(roll_deg) * d).apply(u)
    R = np.stack([d, u, np.cross(d, u)], 1) @ basis.T
    t = np.asarray(center_mm, float) - d * standoff_mm - R @ np.asarray(hand.grasp_center_mm, float)
    return _pose(R, t)


class ContactPlanner:
    """Plans grasps on graspable body ``body_index`` of a loaded ``world``.

    The object is assumed to sit at its loaded pose; candidate hand poses are
    expressed in that body's frame. ``open_dofs`` is the preshape applied
    before every approach (default: all zeros).
    """

    def __init__(self, world, body_index: int = 0, config: Optional[PlannerConfig] = None,
                 hand: Optional[HandFrame] = None, open_dofs: Optional[Sequence[float]] = None):
        self.world, self.body, self.cfg = world, body_index, config or PlannerConfig()
        self.hand = hand or HandFrame()
        self.open_dofs = (np.zeros(world.get_current_hand_num_dof()) if open_dofs is None
                          else np.asarray(open_dofs, float))
        pose = world.get_graspable_body_pose(body_index)
        self.T_world_object = _pose(Rotation.from_quat(pose['quat_wxyz'], scalar_first=True).as_matrix(),
                                    np.asarray(pose['translation'], float))
        self.center = np.asarray(world.get_graspable_body_cog_local(body_index), float)
        self.rng = np.random.default_rng(self.cfg.seed)
        self.rejections: dict = {}
        self.closing: Optional[List[int]] = None     # DOFs auto_grasp closes; None closes all
        self._velocities = list(world.get_dof_default_velocities())
        self.task = None
        if self.cfg.task_forces_mm:
            F = np.asarray(self.cfg.task_forces_mm, float)
            F[:, 3:] /= np.linalg.norm(F[:, 3:], axis=1, keepdims=True)
            self.task = task_wrenches_from_forces(F[:, :3], F[:, 3:], self.center, self.TORQUE_SCALE_MM)

    # ------------------------------------------------------------------ poses
    def hand_pose(self, direction: np.ndarray, roll_deg: float, standoff_mm: float) -> np.ndarray:
        """T_object_hand for an approach; see :func:`palm_pose_for_approach`."""
        return palm_pose_for_approach(direction, roll_deg, standoff_mm, self.hand, self.center)

    def _place(self, T_object_hand: np.ndarray) -> None:
        T = self.T_world_object @ T_object_hand
        q = Rotation.from_matrix(T[:3, :3]).as_quat(scalar_first=True)
        self.world.set_current_hand_pose(q.tolist(), T[:3, 3].tolist())

    def _collision_free(self, T_object_hand: np.ndarray) -> bool:
        self._place(T_object_hand)
        return bool(self.world.no_collision())

    def approach(self, T_start: np.ndarray, direction: np.ndarray) -> Optional[np.ndarray]:
        """Advance from ``T_start`` along ``direction`` until the open hand first
        touches the object; return the pose ``clearance_mm`` short of contact."""
        self.world.set_current_hand_dof_values(self.open_dofs.tolist())
        if not self._collision_free(T_start):
            return None
        step = self.cfg.approach_step_mm
        d = direction / np.linalg.norm(direction)
        travel, last_free = 0.0, 0.0
        limit = 2 * self.cfg.standoff_mm
        while travel < limit:
            travel += step
            T = T_start.copy()
            T[:3, 3] += d * travel
            if not self._collision_free(T):
                break
            last_free = travel
        else:
            return None
        T = T_start.copy()
        T[:3, 3] += d * max(0.0, last_free - self.cfg.clearance_mm)
        return T

    # ---------------------------------------------------------------- closure
    def _reject(self, why: str) -> None:
        self.rejections[why] = self.rejections.get(why, 0) + 1

    def close(self, T_object_hand: np.ndarray):
        """Close from ``T_object_hand``; return (dofs, contacts, native epsilon)
        or None if the start pose collides or the closure makes too few contacts."""
        from ..core.bindings import Grasp
        self.world.set_current_hand_dof_values(self.open_dofs.tolist())
        if not self._collision_free(T_object_hand):
            self._reject('closure_start_collides')
            return None
        if self.closing is not None:
            self.world.set_dof_default_velocities(
                [v if i in self.closing else 0.0 for i, v in enumerate(self._velocities)])
        try:
            self.world.auto_grasp(render_it=False, speed_factor=self.cfg.autograsp_speed,
                                  stop_at_contact=False)
        finally:
            if self.closing is not None:
                self.world.set_dof_default_velocities(self._velocities)
        grasp = Grasp(self.world, self.body)
        if not grasp.compute():
            self._reject('grasp_compute_failed')
            return None
        contacts = [ContactPoint(np.asarray(c['position_mm'], float), np.asarray(c['normal'], float))
                    for c in grasp.object_contact_geometry()]
        if len(contacts) < self.cfg.min_contacts:
            self._reject('too_few_contacts')
            return None
        if self.cfg.contact_region_mm is not None:
            lo, hi = (np.asarray(b, float) for b in self.cfg.contact_region_mm)
            if any(np.any(c.position < lo) or np.any(c.position > hi) for c in contacts):
                self._reject('contact_outside_region')
                return None
        return (np.asarray(self.world.get_current_hand_dof_values(), float), contacts,
                float(grasp.epsilon_quality()))

    def _hand_vertices_object(self) -> np.ndarray:
        """Every hand link vertex in the object frame (mm)."""
        R, t = self.T_world_object[:3, :3], self.T_world_object[:3, 3]
        V = [np.asarray(self.world.get_link_vertices_world(p['chain'], max(p['link'], 0)))
             for p in self.world.get_link_poses()]
        V = [v for v in V if v.size]
        return (np.vstack(V) - t) @ R if V else np.zeros((0, 3))

    def above_support(self, V: Optional[np.ndarray] = None) -> bool:
        """True if every hand vertex is on the free side of the support plane."""
        if self.cfg.support_normal is None:
            return True
        n_obj = np.asarray(self.cfg.support_normal, float)
        n_obj = n_obj / np.linalg.norm(n_obj)
        V = self._hand_vertices_object() if V is None else V
        return not V.size or float(np.min(V @ n_obj)) >= self.cfg.support_offset_mm

    def clear_of_keepout(self, V: Optional[np.ndarray] = None) -> bool:
        """True if no hand vertex lies inside a keep-out box (e.g. a fixture the
        object is later placed into, expressed in the object frame)."""
        if not self.cfg.keepout_boxes_mm:
            return True
        V = self._hand_vertices_object() if V is None else V
        for c, h in self.cfg.keepout_boxes_mm:
            if np.any(np.all(np.abs(V - np.asarray(c, float)) < np.asarray(h, float), axis=1)):
                return False
        return True

    # ---------------------------------------------------------------- scoring
    TORQUE_SCALE_MM = 100.0   # task quality is invariant to it; fixed so the hull and the loads agree

    def epsilons(self, contacts: List[ContactPoint]) -> np.ndarray:
        return np.array([analyze_gws(contacts, self.center, friction_coef=float(mu)).epsilon
                         for mu in self.cfg.friction_samples])

    def task_qualities(self, contacts: List[ContactPoint]) -> np.ndarray:
        if len(contacts) < 2:
            return np.zeros(len(self.cfg.friction_samples))
        return np.array([compute_task_wrench_quality(
            build_grasp_matrix_from_contacts(contacts, self.center, float(mu), max_radius=self.TORQUE_SCALE_MM),
            self.task) for mu in self.cfg.friction_samples])

    def _qualities(self, contacts: Optional[List[ContactPoint]]):
        n = len(self.cfg.friction_samples)
        if contacts is None:
            return np.zeros(n), (None if self.task is None else np.zeros(n))
        return self.epsilons(contacts), (None if self.task is None else self.task_qualities(contacts))

    def perturb(self, T: np.ndarray) -> np.ndarray:
        dR = Rotation.from_rotvec(np.radians(self.cfg.pose_sigma_deg) * self.rng.standard_normal(3))
        out = T.copy()
        out[:3, :3] = dR.as_matrix() @ T[:3, :3]
        out[:3, 3] = T[:3, 3] + self.cfg.pose_sigma_mm * self.rng.standard_normal(3)
        return out

    def evaluate(self, T_start: np.ndarray, direction: np.ndarray, nominal):
        """(epsilon, task) samples, task None without task loads: rows are the nominal closure then closures under execution
        error, columns the friction samples. Execution error perturbs the standoff
        pose and the approach direction; the hand then approaches until contact and
        closes, so an error changes where the hand meets the object rather than
        starting it inside the object. A closure that fails scores zero."""
        rows = [self._qualities(nominal[1])]
        counted = dict(self.rejections)     # robustness closures do not count as rejections
        for _ in range(self.cfg.n_pose_samples):
            T = self.perturb(T_start)
            d = T[:3, :3] @ T_start[:3, :3].T @ direction
            T_pre = self.approach(T, d)
            res = self.close(T_pre) if T_pre is not None else None
            rows.append(self._qualities(res[1] if res is not None else None))
        self.rejections = counted
        eps = np.array([r[0] for r in rows])
        return eps, (None if self.task is None else np.array([r[1] for r in rows]))

    # ------------------------------------------------------------------- plan
    def plan(self, top_k: Optional[int] = None, progress: Optional[Callable[[int, int], None]] = None
             ) -> List[GraspCandidate]:
        cfg = self.cfg
        if cfg.approaches is not None:
            pairs = [(np.asarray(a[:3], float) / np.linalg.norm(a[:3]), float(a[3])) for a in cfg.approaches]
        else:
            rolls = np.arange(cfg.n_rolls) * 360.0 / cfg.n_rolls
            pairs = [(d, r) for d in fibonacci_directions(cfg.n_directions) for r in rolls]
        preshapes = [np.asarray(p, float) for p in (cfg.preshapes or [self.open_dofs])]
        closings = [None] if cfg.closing_sets is None else [sorted(map(int, c)) for c in cfg.closing_sets]
        out: List[GraspCandidate] = []
        total, k = len(pairs) * len(preshapes) * len(closings), 0
        for (d, roll), pre, closing in ((pr, p, c) for pr in pairs for p in preshapes for c in closings):
            self.closing = closing
            k += 1
            self.open_dofs = pre
            if progress:
                progress(k, total)
            start = self.hand_pose(d, roll, cfg.standoff_mm)
            T_pre = self.approach(start, d)
            if T_pre is None:
                self._reject('approach_blocked_or_missed')
                continue
            nominal = self.close(T_pre)
            if nominal is None:
                continue
            V = self._hand_vertices_object()
            if not self.above_support(V):
                self._reject('below_support')
                continue
            if not self.clear_of_keepout(V):
                self._reject('in_keepout')
                continue
            dofs, contacts, native = nominal
            # Execution-error samples depend only on the seed and this candidate,
            # not on which candidates were planned before it (e.g. in a shard).
            ident = np.round(np.r_[d, roll, pre, closing or []], 6).tobytes()
            self.rng = np.random.default_rng([cfg.seed, zlib.crc32(ident)])
            eps, task = self.evaluate(start, d, nominal)
            flat = eps.ravel()
            mid = len(cfg.friction_samples) // 2
            eps_cvar = lower_tail_mean(flat, alpha=cfg.tail_mass)
            out.append(GraspCandidate(
                T_object_hand=T_pre, dofs=dofs, contacts=contacts, native_epsilon=native,
                nominal_epsilon=float(eps[0, mid]), epsilon_samples=eps,
                score=eps_cvar if task is None else lower_tail_mean(task.ravel(), alpha=cfg.tail_mass),
                expected_epsilon=float(flat.mean()), approach_dir=d, roll_deg=float(roll),
                preshape=pre.copy(), closing=closing, epsilon_cvar=eps_cvar, task_samples=task,
                nominal_task=None if task is None else float(task[0, mid]),
                expected_task=None if task is None else float(task.mean())))
        self.closing = None
        out.sort(key=lambda c: c.score, reverse=True)
        return out if top_k is None else out[:top_k]
