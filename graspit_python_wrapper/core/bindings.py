"""Core bindings for graspit-py.

Re-exports the compiled pybind11 extension (`._bindings`). Importing this
module fails with an explanatory error when the extension has not been built
(see docs/INSTALL.md).
"""

from __future__ import annotations

from typing import Any


try:
    from . import _bindings as _native  # type: ignore

    Grasp = _native.Grasp
    World = _native.World
    vec3_to_numpy = _native.vec3_to_numpy
    numpy_to_vec3 = _native.numpy_to_vec3
    process_events = _native.process_events
    spin = _native.spin
    gui_available = _native.gui_available
    show_gui = _native.show_gui
    view_all = _native.view_all
    main_window_xid = _native.main_window_xid
    save_screenshot = _native.save_screenshot
    start_main_loop = _native.start_main_loop
    main_loop = _native.main_loop
    event_loop = _native.event_loop
    exit_main_loop = _native.exit_main_loop
    resource_exists = _native.resource_exists

    def initialize() -> None:
        """Initialize the GraspIt core.

        Note: The native layer may set env vars via `setenv()` (e.g. `GRASPIT`,
        `QT_QPA_PLATFORM`). Python's `os.environ` does not automatically reflect
        such changes, so we mirror them in after initialization.
        """

        _native.initialize()

        try:
            import ctypes
            import os

            libc = ctypes.CDLL(None)
            libc.getenv.restype = ctypes.c_char_p

            for key in ("GRASPIT", "QT_QPA_PLATFORM"):
                val = libc.getenv(key.encode("utf-8"))
                if val is not None:
                    os.environ[key] = val.decode("utf-8")
        except Exception:
            # Best-effort only.
            pass


except Exception:  # pragma: no cover
    raise ImportError(
        "Failed to import the native GraspIt bindings (graspit_python_wrapper.core._bindings). "
        "This project no longer provides a placeholder simulator. Build/install the extension (see docs/INSTALL.md)."
    )


__all__ = [
    "initialize",
    "World",
    "Grasp",
    "vec3_to_numpy",
    "numpy_to_vec3",
    "process_events",
    "spin",
    "gui_available",
    "show_gui",
    "view_all",
    "main_window_xid",
    "save_screenshot",
    "start_main_loop",
    "main_loop",
    "event_loop",
    "exit_main_loop",
    "resource_exists",
]
