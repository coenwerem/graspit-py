"""Plan grasps from the command line and write ranked candidates as JSON.

    xvfb-run -a python -m graspit_python_wrapper.planning \
        --robot hand.xml --object object.xml --config problem.json --out candidates.json

``problem.json`` holds ``hand`` (HandFrame fields) and ``planner``
(PlannerConfig fields); both are optional. The object is placed at the
identity pose, so candidate poses are in the object's own frame (mm).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

from . import ContactPlanner, HandFrame, PlannerConfig


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robot', type=Path, required=True)
    ap.add_argument('--object', type=Path, required=True)
    ap.add_argument('--config', type=Path)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--top-k', type=int)
    a = ap.parse_args(argv)
    problem = json.loads(a.config.read_text()) if a.config else {}
    hand = HandFrame(**problem.get('hand', {}))
    cfg = PlannerConfig(**problem.get('planner', {}))

    from ..core import bindings as gp
    gp.initialize()
    world = gp.World()
    world.init()
    world.clear()
    if not world.load_robot(str(a.robot.resolve())):
        raise SystemExit(f'GraspIt could not load robot {a.robot}')
    if not world.load_graspable_body(str(a.object.resolve())):
        raise SystemExit(f'GraspIt could not load object {a.object}')

    t0 = time.perf_counter()
    last = [0.0]

    def progress(k, n):
        if time.perf_counter() - last[0] > 10 or k == n:
            last[0] = time.perf_counter()
            print(f'  {k}/{n} approaches, {last[0] - t0:.0f} s', file=sys.stderr, flush=True)

    planner = ContactPlanner(world, 0, cfg, hand=hand)
    cands = planner.plan(top_k=a.top_k, progress=progress)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(dict(
        robot=str(a.robot), object=str(a.object), hand=asdict(hand),
        planner={k: (list(v) if isinstance(v, tuple) else v) for k, v in asdict(cfg).items()},
        seconds=time.perf_counter() - t0, rejections=planner.rejections, units=dict(length='mm', angle='rad', quaternion='wxyz'),
        candidates=[c.to_dict() for c in cands]), indent=1, default=float))
    print(f'{len(cands)} candidates in {time.perf_counter() - t0:.1f} s -> {a.out}; '
          f'rejections {planner.rejections}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
