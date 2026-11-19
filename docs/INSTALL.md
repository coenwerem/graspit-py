# Installation

The reference environment is Ubuntu 24.04 with Python 3.12, GCC 13,
Qt 5, and a solver-free GraspIt! build. Use Docker for the packaged
environment or build the native extension against a local GraspIt! tree.

## Docker

From the wrapper repository root:

```bash
docker buildx build --load -f docker/Dockerfile -t graspit-py .
docker run --rm graspit-py
```

The Dockerfile builds the public GraspIt! engine at revision
`de64377541c4b4b2987c72d0a17c02464f9c7095` with the supplied
`docker/graspit-engine.patch`. The default container command runs the wrapper
tests under a virtual X display. To use a prepared engine source tree,
add `--build-context graspit=/absolute/path/to/graspit-source` to the
build command.

## Native build

Install the system dependencies on Ubuntu 24.04:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake pkg-config git python3-dev python3-venv \
  pybind11-dev qtbase5-dev libqt5opengl5-dev libcoin-dev libsoqt520-dev \
  libqhull-dev libeigen3-dev libblas-dev liblapack-dev xvfb xauth
```

In the wrapper repository, prepare and build the engine:

```bash
bash docker/prepare_graspit.sh ../graspit-source
export GRASPIT="$(cd ../graspit-source && pwd)"
export GRASPIT_SOURCE_DIR="$GRASPIT"
export GRASPIT_BUILD_DIR="$GRASPIT/build"
cmake -S "$GRASPIT_SOURCE_DIR" -B "$GRASPIT_BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release -DOPTIMIZATION_SOLVER=NONE
cmake --build "$GRASPIT_BUILD_DIR" --target graspit --parallel 4
```

Build the Python wheel and install it in a virtual environment:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install setuptools wheel
.venv/bin/python -m pip wheel --no-build-isolation --no-deps . --wheel-dir dist
.venv/bin/python -m pip install "dist/graspit_python_wrapper-0.2.0-"*.whl
.venv/bin/python -m pip install "mujoco>=3.1"
```

The last command adds MuJoCo for model export. The wheel loads
`libgraspit.so` from the build directory, so keep the engine build
there for subsequent sessions. Set `GRASPIT` to the source tree when
starting a new shell. GraspIt! uses this directory to locate its models
and other resources. Set `CMAKE_BUILD_PARALLEL_LEVEL` to limit the
number of compilation jobs when building the wrapper.

The solver-free build supports autograsp, collision queries, contact
geometry, and wrench-space scoring. Quasistatic contact force optimization
requires a GraspIt! build with an optimization solver.

## Headless execution

Run native GraspIt! commands under Xvfb when there is no X display:

```bash
xvfb-run -a .venv/bin/python your_program.py
```

The GraspIt! initialization uses Qt and SoQt. Use an Xvfb display for
headless execution. When creating a custom container, use
`docker run --init` or the image's `tini` entry point so
`xvfb-run` can receive its startup signal.
