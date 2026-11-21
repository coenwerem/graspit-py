# Installation

Choose **Docker** for a complete headless environment containing GraspIt!, the Python bindings, numerical libraries, and MuJoCo. Choose a **native installation** to work directly with the compiler, engine build tree, and Python extension on Ubuntu. Both paths compile GraspIt! from source; Docker also stores the operating-system image layers, while the native path keeps a source/build tree and virtual environment.

The reference environment uses Ubuntu 24.04, Python 3.12, GCC 13, and Qt 5.

```{important}
The reference engine uses `OPTIMIZATION_SOLVER=NONE`. It supports autograsp, collision queries, contact geometry, and wrench-space scoring. Native quasistatic contact-force optimization requires an engine built with an optimization solver. Select the engine configuration before building the wrapper; see the [GraspIt! project](https://graspit-simulator.github.io/) for engine build options.
```

## Obtain the Source

Both installation paths start from the public wrapper repository.

```bash
git clone https://github.com/coenwerem/graspit-py.git
cd graspit-py
```

The checkout contains `docs/`, `docker/`, `examples/`, and the Python packages. The distribution is named `graspit-py`; Python imports use `graspit_py`. Existing `graspit_python_wrapper` imports remain supported.

## Docker Installation

Build the image with Docker Buildx, then request the installed planner's help from outside the source directory.

```bash
docker buildx build --load -f docker/Dockerfile -t graspit-py .
docker run --rm graspit-py python -m graspit_py.planning --help
```

The help command exits successfully and prints a usage synopsis beginning with:

```text
usage: python -m graspit_py.planning [-h] --robot ROBOT --object OBJECT
```

The remaining options include `--config`, `--out`, and `--top-k`. Continue with the [bundled Barrett example](quickstart.md#first-closure-with-the-bundled-barrett-hand) to exercise native closure and contact queries.

The Dockerfile fetches the public GraspIt! engine at revision `de64377541c4b4b2987c72d0a17c02464f9c7095` and applies `docker/graspit-engine.patch`. Its default container command runs the repository checks under a virtual display. The build controls are:

| Build Option | Default | Purpose |
|---|---|---|
| `--build-arg BUILD_JOBS=2` | `3` compilation jobs | Reduce simultaneous compiler processes on a host with limited memory. |
| `--build-context graspit=/absolute/path/to/graspit-source` | Fetch the pinned public source | Supply an already prepared engine tree, including its required compatibility changes. |

Build time and disk use depend on compilation resources and existing Docker layers; the command reports its actual build progress.

## Native Installation

Install the system dependencies on Ubuntu 24.04:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake pkg-config git python3-dev python3-venv \
  pybind11-dev qtbase5-dev libqt5opengl5-dev libcoin-dev libsoqt520-dev \
  libqhull-dev libeigen3-dev libblas-dev liblapack-dev xvfb xauth
```

From the wrapper root, fetch and prepare the engine, then build its library:

```bash
bash docker/prepare_graspit.sh ../graspit-source
export GRASPIT="$(cd ../graspit-source && pwd)"
export GRASPIT_SOURCE_DIR="$GRASPIT"
export GRASPIT_BUILD_DIR="$GRASPIT/build"
cmake -S "$GRASPIT_SOURCE_DIR" -B "$GRASPIT_BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release -DOPTIMIZATION_SOLVER=NONE
cmake --build "$GRASPIT_BUILD_DIR" --target graspit --parallel 4
```

`prepare_graspit.sh` creates a **new** destination, fetches the pinned revision from `https://github.com/graspit-simulator/graspit.git`, and applies the compatibility patch. It does not require a pre-existing clone. Choose an unused destination name if `../graspit-source` already exists. After compilation, `$GRASPIT_BUILD_DIR/libgraspit.so` is the library linked by the wrapper.

Build and install the wheel with the same Python interpreter:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install setuptools wheel
.venv/bin/python -m pip wheel --no-build-isolation --no-deps . --wheel-dir dist
.venv/bin/python -m pip install dist/graspit_py-*.whl
.venv/bin/python -m pip install "mujoco>=3.1"
.venv/bin/python -I -c "from graspit_py.core import bindings; print('graspit-py ready')"
```

The final command prints:

```text
graspit-py ready
```

This confirms the installed native extension loaded. Python's `-I` option excludes the current source directory from the import path, so the check uses the installed wheel. If `dist/` contains wheels from several builds, install the new wheel by its printed filename. When replacing the previous distribution, first uninstall `graspit_python_wrapper` from the target environment. Set `CMAKE_BUILD_PARALLEL_LEVEL=2` before building the wheel to limit compilation parallelism.

Preserve the engine build directory: the extension's runtime library path points to it. Set `GRASPIT` to the engine source tree in each new shell so the engine can find its model collection and resources. With these dependencies and paths prepared, installation directly from GitHub is also available:

```bash
.venv/bin/python -m pip install "graspit-py[mjcf] @ git+https://github.com/coenwerem/graspit-py.git"
```

Successful installation makes `graspit_py` and the MuJoCo exporter available in that environment. The `mjcf` extra installs MuJoCo; the `urdf` extra supplies `trimesh` and `yourdfpy` for URDF object helpers. See the [API dependency table](api.md) for the operations requiring each dependency.

## Numerical Analysis Without the Native Engine

The numerical modules can be used directly from this source checkout with NumPy and SciPy:

```bash
python3 -m venv .venv-numerics
.venv-numerics/bin/python -m pip install numpy scipy
.venv-numerics/bin/python -c "from graspit_py.risk import lower_tail_mean; print(lower_tail_mean([1, 2, 3, 4], alpha=0.5))"
```

The final command prints `1.5`. This source-checkout route supports the [analytic contact example](api.md#numerical-utilities). Installing the complete distribution with `pip` also builds the native extension.

## Headless Execution

The reference GraspIt! build uses Qt and SoQt during initialization. Xvfb provides the virtual X display used by the documented headless native commands:

```bash
xvfb-run -a .venv/bin/python -I your_program.py
```

The program's own output appears in the terminal; Xvfb supplies the display without opening a desktop window. The Docker image uses `tini` as its entry point. A custom container can use `docker run --init` so `xvfb-run` receives its startup signal.

## Troubleshooting

| Symptom | Check and Remedy |
|---|---|
| CMake reports `GraspIt not found`, or import reports a missing `libgraspit.so` | Confirm `$GRASPIT_BUILD_DIR/libgraspit.so` exists and both engine path variables identify the intended source/build pair. Rebuild the wheel after moving an engine tree. For loader diagnostics on Linux, use `ldd` on the installed `_bindings*.so`; unresolved libraries appear as `not found`. |
| CMake cannot locate Qt 5, Coin, or SoQt, or Qt cannot connect to a display | Install the development packages listed above. Run headless native programs under `xvfb-run -a`; the reference image uses the `xcb` Qt platform. A Qt package-discovery error occurs during compilation, while a display error occurs during initialization. |
| Import reports that `_bindings` is unavailable after wheel installation | Verify that the wheel and invoking interpreter use the same environment and Python version. Run the isolated `python -I` import check above to avoid importing an unbuilt source checkout in place of the installed package. |

## Build the Manual

The documentation requires Python 3.11 or later. From the wrapper root:

```bash
python3 -m venv .venv-docs
.venv-docs/bin/python -m pip install -r docs/requirements.txt
.venv-docs/bin/python -m sphinx -nW -b html docs docs/_build/html
```

A successful build ends with `build succeeded.` and writes `docs/_build/html/index.html`. Open this file or serve the directory locally to inspect the Shibuya pages.
