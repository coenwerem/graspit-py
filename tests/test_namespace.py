"""The public import name must share modules and native state with old imports."""
from __future__ import annotations

import importlib
import os
import subprocess
import sys

import pytest


def run_python(code):
    result = subprocess.run(
        [sys.executable, "-W", "error::RuntimeWarning", "-c", code],
        capture_output=True, text=True, env=os.environ.copy(),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return result


@pytest.mark.parametrize("first", ["graspit_py", "graspit_python_wrapper"])
def test_import_names_share_modules(first):
    # Loading source twice would split dataclass identity and module state.
    run_python(f"""
import importlib
import sys

first = {first!r}
second = "graspit_python_wrapper" if first == "graspit_py" else "graspit_py"
assert importlib.import_module(first) is importlib.import_module(second)
for suffix in (
    "risk", "kinematics", "core", "core.types", "grasp", "grasp.quality",
    "planning", "planning.contact_planner", "convert", "utils", "utils.geometry",
    "utils.urdf",
):
    a = importlib.import_module(first + "." + suffix)
    b = importlib.import_module(second + "." + suffix)
    assert a is b, suffix
    assert a.__spec__.name == "graspit_python_wrapper." + suffix, suffix
from graspit_py.core.types import ContactPoint
from graspit_python_wrapper.core.types import ContactPoint as OldContactPoint
from graspit_py.planning import PlannerConfig
from graspit_python_wrapper.planning import PlannerConfig as OldPlannerConfig
assert ContactPoint is OldContactPoint
assert PlannerConfig is OldPlannerConfig
assert "graspit_python_wrapper.core._bindings" not in sys.modules
assert "mujoco" not in sys.modules
""")


@pytest.mark.parametrize("first", ["graspit_py", "graspit_python_wrapper"])
def test_native_import_names_share_extension(first):
    try:
        importlib.import_module("graspit_python_wrapper.core.bindings")
    except ImportError:
        pytest.skip("native bindings have not been built")
    # A second extension load can duplicate the native singleton or pybind types.
    run_python(f"""
import importlib
import sys

first = {first!r}
second = "graspit_python_wrapper" if first == "graspit_py" else "graspit_py"
a = importlib.import_module(first + ".core.bindings")
b = importlib.import_module(second + ".core.bindings")
native_a = importlib.import_module(first + ".core._bindings")
native_b = importlib.import_module(second + ".core._bindings")
assert a is b
assert native_a is native_b
assert a.World is b.World is native_a.World
assert a.Grasp is b.Grasp is native_a.Grasp
assert native_a.__name__ == "graspit_python_wrapper.core._bindings"
assert native_a.__spec__.name == "graspit_python_wrapper.core._bindings"
from graspit_py import core
assert core.bindings is a
""")


def test_optional_export_module_is_shared():
    pytest.importorskip("mujoco")
    run_python("""
import importlib
new = importlib.import_module("graspit_py.convert.mjcf_hand")
old = importlib.import_module("graspit_python_wrapper.convert.mjcf_hand")
assert new is old
assert new.ExportedHand is old.ExportedHand
""")


def test_planning_module_cli():
    result = subprocess.run(
        [sys.executable, "-W", "error::RuntimeWarning", "-m",
         "graspit_py.planning", "--help"],
        check=True, capture_output=True, text=True,
    )
    assert "--robot" in result.stdout
    assert "--object" in result.stdout
    assert "--out" in result.stdout
    assert result.stderr == ""
