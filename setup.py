from __future__ import annotations

import os
import shutil
import sys
import subprocess
from pathlib import Path

from setuptools import Extension, find_namespace_packages, setup
from setuptools.command.build_ext import build_ext


class CMakeBuild(build_ext):
    def build_extension(self, ext: Extension) -> None:
        # Only handle our CMake-built extension.
        if ext.name != "graspit_python_wrapper.core._bindings":
            super().build_extension(ext)
            return

        repo_root = Path(__file__).resolve().parent
        source_dir = repo_root / "bindings"
        if not source_dir.exists():
            raise RuntimeError(f"bindings/ directory not found at {source_dir}")

        # Use an isolated temp build dir to avoid stale CMake cache.
        build_temp = Path(self.build_temp) / "graspit_bindings"
        build_temp.mkdir(parents=True, exist_ok=True)

        cfg = "Debug" if self.debug else "Release"
        cmake_args = [
            f"-DCMAKE_BUILD_TYPE={cfg}",
            f"-DPython_EXECUTABLE={sys.executable}",
        ]

        # Allow overriding GraspIt paths via env vars.
        graspit_source = os.environ.get("GRASPIT_SOURCE_DIR")
        graspit_build = os.environ.get("GRASPIT_BUILD_DIR")
        if graspit_source:
            cmake_args.append(f"-DGRASPIT_SOURCE_DIR={graspit_source}")
        if graspit_build:
            cmake_args.append(f"-DGRASPIT_BUILD_DIR={graspit_build}")

        subprocess.check_call([
            "cmake",
            "-S",
            str(source_dir),
            "-B",
            str(build_temp),
            *cmake_args,
        ])
        subprocess.check_call([
            "cmake",
            "--build",
            str(build_temp),
            "-j",
            os.environ.get("CMAKE_BUILD_PARALLEL_LEVEL", "4"),
        ])

        # Find the produced extension module.
        candidates = list(build_temp.glob("_bindings*.so"))
        if not candidates:
            # Some generators may place it in a subdir.
            candidates = list(build_temp.rglob("_bindings*.so"))
        if not candidates:
            raise RuntimeError(f"Could not find built _bindings*.so under {build_temp}")

        built_so = candidates[0]

        if self.inplace:
            dest_dir = repo_root / "graspit_python_wrapper" / "core"
        else:
            dest_dir = Path(self.build_lib) / "graspit_python_wrapper" / "core"

        dest_dir.mkdir(parents=True, exist_ok=True)
        dest_path = dest_dir / built_so.name
        shutil.copy2(built_so, dest_path)


ext_modules = [
    Extension("graspit_python_wrapper.core._bindings", sources=[]),
]

setup(
    name="graspit_python_wrapper",
    version="0.2.0",
    author="Clinton Enwerem",
    author_email="me@clintonenwerem.com",
    description="Python bindings and risk-sensitive contact planning for the GraspIt! grasping simulator",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/coenwerem/graspit-python-wrapper",
    license="GPL-3.0-or-later",
    classifiers=[
        "License :: OSI Approved :: GNU General Public License v3 or later (GPLv3+)",
        "Programming Language :: Python :: 3",
        "Programming Language :: C++",
        "Operating System :: POSIX :: Linux",
        "Topic :: Scientific/Engineering",
    ],
    ext_modules=ext_modules,
    cmdclass={"build_ext": CMakeBuild},
    packages=find_namespace_packages(include=[
        "graspit_python_wrapper",
        "graspit_python_wrapper.core",
        "graspit_python_wrapper.grasp",
        "graspit_python_wrapper.planning",
        "graspit_python_wrapper.convert",
        "graspit_python_wrapper.utils",
    ]),
    python_requires=">=3.10",
    install_requires=["numpy", "scipy"],
    extras_require={"test": ["pytest"], "urdf": ["trimesh", "yourdfpy"], "mjcf": ["mujoco>=3.1"]},
)
