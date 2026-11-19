"""Shared fixtures: a small MJCF test hand exported to GraspIt."""
from pathlib import Path

import pytest

DATA = Path(__file__).parent / 'data'


@pytest.fixture(scope='session')
def gripper_mjcf():
    mujoco = pytest.importorskip('mujoco')
    return mujoco.MjModel.from_xml_path(str(DATA / 'gripper.xml'))


@pytest.fixture(scope='session')
def exported_gripper(gripper_mjcf, tmp_path_factory):
    from graspit_python_wrapper.convert.mjcf_hand import export_hand
    out = tmp_path_factory.mktemp('gripper')
    return export_hand(gripper_mjcf, 'palm', out, name='gripper')
