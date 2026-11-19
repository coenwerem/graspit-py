#!/usr/bin/env python3
"""Display a closure using a supplied GraspIt robot, object, and initial object position."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

def _die(msg: str) -> "None":
    print(f"ERROR: {msg}")
    raise SystemExit(2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', type=Path, required=True)
    parser.add_argument('--object', type=Path, required=True)
    parser.add_argument('--object-position-mm', type=float, nargs=3, required=True,
                        metavar=('X', 'Y', 'Z'), help='Object position in the hand frame')
    args = parser.parse_args()

    # GUI requires a real display.
    if os.environ.get("QT_QPA_PLATFORM") == "offscreen":
        _die(
            "QT_QPA_PLATFORM=offscreen is set. Unset it (or set to xcb) before running to see the GUI."
        )

    if not os.environ.get("DISPLAY"):
        _die(
            "DISPLAY is not set. Run under a graphical session, or use ssh -X / -Y to forward X11."
        )

    from graspit_python_wrapper.core import bindings as gp

    hand_path, obj_path = args.robot.resolve(), args.object.resolve()

    gp.initialize()
    if not gp.gui_available():
        _die(
            "GraspIt initialized without GUI support (no MainWindow/IVmgr). "
            "This usually means Qt/SoQt couldn't initialize a display."
        )

    w = gp.World()
    w.init()

    if not w.load_robot(str(hand_path)):
        parser.error(f'could not load robot {hand_path}')
    if not w.load_graspable_body(str(obj_path)):
        parser.error(f'could not load object {obj_path}')
    obj_id = w.num_graspable_bodies() - 1

    w.place_graspable_body_relative_to_hand(obj_id, *args.object_position_mm)

    w.quick_open(speed_factor=0.5)
    w.auto_grasp(render_it=False, speed_factor=0.8, stop_at_contact=False)

    g = gp.Grasp(w, obj_id)
    ok = g.compute()
    contacts = len(g.contact_points())
    eps = float(g.epsilon_quality()) if ok else float("nan")
    vol = float(g.volume_quality()) if ok else float("nan")

    print(f"compute={ok} contacts={contacts} eps={eps} vol={vol}")
    print("Launching GUI. Close the GraspIt window to exit.")

    gp.main_loop(-1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
