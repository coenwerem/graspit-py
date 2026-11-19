# Installation

The reference environment uses Ubuntu 24.04, Python 3.12, GCC 13,
Qt 5, and GraspIt! built with `OPTIMIZATION_SOLVER=NONE`. The Docker
image includes the native engine, Python bindings, numerical libraries,
and MuJoCo. A native installation builds the Python extension against
a local GraspIt! tree.

Both installation paths start from the wrapper source.

```bash
git clone https://github.com/coenwerem/graspit-python-wrapper.git
cd graspit-python-wrapper
```

## Docker Installation

Build the image with Docker Buildx. The final command imports the
installed planner from outside the source tree and prints its
command-line options.

```bash
docker buildx build --load -f docker/Dockerfile -t graspit-py .
docker run --rm graspit-py python -m graspit_python_wrapper.planning --help
```

The Dockerfile builds the public GraspIt! engine at revision
`de64377541c4b4b2987c72d0a17c02464f9c7095` with the supplied
`docker/graspit-engine.patch`. The default container command runs the wrapper
test suite under a virtual X display. Pass
`--build-context graspit=/absolute/path/to/graspit-source` to build the
image with a prepared engine tree. Pass `--build-arg BUILD_JOBS=2` to
limit parallel compilation on hosts with less available memory.
The [RealHand L6 walkthrough](quickstart.md) uses the resulting image.

## Native Installation

Install the following system dependencies on Ubuntu 24.04.

```bash
sudo apt-get update
sudo apt-get install build-essential cmake pkg-config git python3-dev python3-venv \
  pybind11-dev qtbase5-dev libqt5opengl5-dev libcoin-dev libsoqt520-dev \
  libqhull-dev libeigen3-dev libblas-dev liblapack-dev xvfb xauth
```

From the wrapper repository root, prepare and build the native engine.

```bash
bash docker/prepare_graspit.sh ../graspit-source
export GRASPIT="$(cd ../graspit-source && pwd)"
export GRASPIT_SOURCE_DIR="$GRASPIT"
export GRASPIT_BUILD_DIR="$GRASPIT/build"
cmake -S "$GRASPIT_SOURCE_DIR" -B "$GRASPIT_BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release -DOPTIMIZATION_SOLVER=NONE
cmake --build "$GRASPIT_BUILD_DIR" --target graspit --parallel 4
```

Build and install the wrapper wheel in a virtual environment.

```bash
python3 -m venv .venv
.venv/bin/python -m pip install setuptools wheel
.venv/bin/python -m pip wheel --no-build-isolation --no-deps . --wheel-dir dist
.venv/bin/python -m pip install "dist/graspit_python_wrapper-0.2.0-"*.whl
.venv/bin/python -m pip install "mujoco>=3.1"
```

The last command installs MuJoCo for model export. The wheel loads
`libgraspit.so` from the engine build directory. Preserve the build
directory and set `GRASPIT` to the source tree in each new shell.
GraspIt! resolves its models and other resources relative to this
path. Set `CMAKE_BUILD_PARALLEL_LEVEL` before building the wheel to
limit parallel compilation.

The solver-free build supports autograsp, collision queries, contact
geometry, and wrench-space scoring. Quasistatic contact force optimization
requires a GraspIt! build with an optimization solver.

## Headless Execution

GraspIt! uses Qt and SoQt for initialization. On a host without an
X display, run native wrapper programs under Xvfb.

```bash
xvfb-run -a .venv/bin/python your_program.py
```

The reference image uses `tini` as its entry point. For a custom
container, use an init process such as `docker run --init` so
`xvfb-run` can receive its startup signal.
