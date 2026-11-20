"""The upstream Barrett hand closes on a cube through the public interface."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(os.environ.get('GRASPIT', ''))
ROBOT = ROOT / 'models/robots/Barrett/Barrett.xml'
CUBE = ROOT / 'models/objects/cube.xml'


@pytest.mark.skipif(not ROBOT.is_file() or not CUBE.is_file(),
                    reason='set GRASPIT to the upstream native model directory')
def test_barrett_native_cube_contacts(tmp_path):
    pytest.importorskip('graspit_python_wrapper.core.bindings')
    example = Path(__file__).parents[1] / 'examples/contact_quality.py'
    output = tmp_path / 'barrett-cube.json'
    subprocess.run([sys.executable, str(example), '--robot', str(ROBOT.resolve()),
                    '--object', str(CUBE.resolve()), '--object-position-mm', '0', '0', '100',
                    '--out', str(output)], check=True, timeout=60, capture_output=True, text=True)
    record = json.loads(output.read_text())
    assert len(record['hand_dof_rad']) == 4
    assert len(record['contacts_object_frame']) >= 3
    assert record['epsilon'] > 0
