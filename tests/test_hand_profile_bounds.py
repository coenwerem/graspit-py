"""Configuration sampling must respect every affine-coupled hinge limit."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import unittest

spec = importlib.util.spec_from_file_location(
    'export_hand_profile', Path(__file__).parents[1] / 'examples/export_hand_profile.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def joint(dof, ratio, offset, limits):
    return SimpleNamespace(dof=dof, ratio=ratio, offset_rad=offset, range_rad=limits)


class CoupledBoundsTest(unittest.TestCase):
 def test_affine_follower_limits_intersect_driver_interval(self):
    chains = [[joint(0, 1., 0., (0., 2.)), joint(0, 2., -.2, (.2, 1.8))]]
    np.testing.assert_allclose(module.coupled_dof_bounds(chains, 1), [[.2, 1.]])


 def test_negative_ratio_reverses_follower_bounds(self):
    chains = [[joint(0, 1., 0., (-2., 2.)), joint(0, -2., .5, (-.5, 1.5))]]
    np.testing.assert_allclose(module.coupled_dof_bounds(chains, 1), [[-.5, .5]])


 def test_infeasible_coupled_limits_are_rejected(self):
    chains = [[joint(0, 1., 0., (0., 1.)), joint(0, 1., 2., (0., 1.))]]
    with self.assertRaisesRegex(ValueError, 'infeasible'):
        module.coupled_dof_bounds(chains, 1)

class MimicReferenceTest(unittest.TestCase):
 def test_nonzero_reference_angles_preserve_absolute_urdf_offset(self):
    path = Path(__file__).parents[1] / 'examples/prepare_urdf_hand.py'
    spec = importlib.util.spec_from_file_location('prepare_urdf_hand', path)
    preparation = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(preparation)
    c0, c1 = preparation.mimic_polynomial(.2, -1.5, .6, -.4)
    for driver in [-.3, 0., .7]:
        reconstructed = .6 + c0 + c1 * (driver + .4)
        self.assertAlmostEqual(reconstructed, .2 - 1.5 * driver)

class ProfileMimicTest(unittest.TestCase):
 def test_declared_planning_coupling_is_explicit_in_prepared_urdf(self):
    import xml.etree.ElementTree as ET
    path = Path(__file__).parents[1] / 'examples/prepare_urdf_hand.py'
    spec = importlib.util.spec_from_file_location('prepare_profile', path)
    preparation = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(preparation)
    root = ET.fromstring('<robot><joint name="driver" type="revolute"/><joint name="follower" type="revolute"/></robot>')
    preparation.apply_profile_mimics(root, {'follower': {'driver': 'driver', 'ratio': 1.2, 'offset_rad': -.1}})
    mimic = root.find("joint[@name='follower']/mimic")
    self.assertEqual(mimic.get('joint'), 'driver')
    self.assertAlmostEqual(float(mimic.get('offset')), -.1)

 def test_conflicting_source_coupling_is_rejected(self):
    import xml.etree.ElementTree as ET
    path = Path(__file__).parents[1] / 'examples/prepare_urdf_hand.py'
    spec = importlib.util.spec_from_file_location('prepare_profile', path)
    preparation = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(preparation)
    root = ET.fromstring('<robot><joint name="driver" type="revolute"/><joint name="follower" type="revolute"><mimic joint="driver" multiplier=".7"/></joint></robot>')
    with self.assertRaisesRegex(ValueError, 'conflicts'):
        preparation.apply_profile_mimics(root, {'follower': {'driver': 'driver', 'ratio': 1.2, 'offset_rad': 0.}})

class ProfileDriverTest(unittest.TestCase):
 def test_cross_wired_follower_with_equal_coefficients_is_rejected(self):
    follower = joint(1, 1.06399, -.04545, (0., 1.6))
    follower.mujoco_joint = 'index_intermediate_joint'
    index = joint(0, 1., 0., (0., 1.5))
    index.mujoco_joint = 'index_proximal_joint'
    middle = joint(1, 1., 0., (0., 1.5))
    middle.mujoco_joint = 'middle_proximal_joint'
    hand = SimpleNamespace(dof_joints=['index_proximal_joint', 'middle_proximal_joint'],
                           chains=[[index, follower], [middle]])
    profile = {'independent_dofs': 2, 'hinges': 3,
               'couplings': {'index_intermediate_joint': [1.06399, -.04545]},
               'mimic_joints': {'index_intermediate_joint': {'driver': 'index_proximal_joint',
                                                            'ratio': 1.06399, 'offset_rad': -.04545}}}
    with self.assertRaisesRegex(ValueError, 'driver'):
        module.validate_profile(hand, profile)

 def test_declared_driver_and_affine_map_are_accepted(self):
    driver = joint(0, 1., 0., (0., 1.5))
    driver.mujoco_joint = 'index_proximal_joint'
    follower = joint(0, 1.06399, -.04545, (0., 1.6))
    follower.mujoco_joint = 'index_intermediate_joint'
    hand = SimpleNamespace(dof_joints=['index_proximal_joint'], chains=[[driver, follower]])
    profile = {'independent_dofs': 1, 'hinges': 2,
               'mimic_joints': {'index_intermediate_joint': {'driver': 'index_proximal_joint',
                                                           'ratio': 1.06399, 'offset_rad': -.04545}}}
    module.validate_profile(hand, profile)
