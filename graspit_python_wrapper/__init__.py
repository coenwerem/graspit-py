"""Python bindings and contact-planning tools for the GraspIt! grasping simulator.

The native extension is imported lazily so that the pure-Python modules
(`risk`, `kinematics`, `grasp.quality`) work without a GraspIt build.
"""
__version__ = '0.2.0'

__all__ = ['__version__']
