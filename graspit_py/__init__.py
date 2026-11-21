"""Public import name for graspit-py.

Submodules are imported through the original implementation package so both
names share Python types, module state, and the native GraspIt singleton.
"""
from __future__ import annotations

import importlib
import importlib.abc
import importlib.util
import sys


_CANONICAL = "graspit_python_wrapper"


class _AliasLoader(importlib.abc.Loader):
    def __init__(self, canonical_name, canonical_spec):
        self.canonical_name = canonical_name
        self.canonical_spec = canonical_spec

    def create_module(self, spec):
        # In particular, never load the native extension under an alias name.
        return importlib.import_module(self.canonical_name)

    def exec_module(self, module):
        # Import machinery temporarily assigns the alias spec to the shared
        # module. Preserve its canonical metadata for reload and introspection.
        module.__spec__ = self.canonical_spec

    def get_code(self, fullname):
        # runpy uses this path for python -m graspit_py.planning.
        return self.canonical_spec.loader.get_code(self.canonical_name)


class _AliasFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if not fullname.startswith("graspit_py."):
            return None
        canonical_name = _CANONICAL + fullname[len("graspit_py"):]
        canonical_spec = importlib.util.find_spec(canonical_name)
        if canonical_spec is None:
            return None
        return importlib.util.spec_from_loader(
            fullname, _AliasLoader(canonical_name, canonical_spec),
            origin=canonical_spec.origin,
            is_package=canonical_spec.submodule_search_locations is not None,
        )


sys.meta_path.insert(0, _AliasFinder())
sys.modules[__name__] = importlib.import_module(_CANONICAL)
