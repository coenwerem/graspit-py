# Build with public GraspIt! source or supply a prepared source checkout:
# docker buildx build -f docker/Dockerfile -t graspit-py .
# docker buildx build -f docker/Dockerfile --build-context graspit=../graspit -t graspit-py .
FROM ubuntu:24.04 AS build-base

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake pkg-config git ca-certificates \
        python3-dev python3-venv pybind11-dev \
        qtbase5-dev libqt5opengl5-dev libcoin-dev libsoqt520-dev \
        libqhull-dev libeigen3-dev libblas-dev liblapack-dev \
        xvfb xauth tini \
    && rm -rf /var/lib/apt/lists/*

FROM build-base AS upstream
COPY docker/prepare_graspit.sh docker/graspit-engine.patch /tmp/engine/
RUN bash /tmp/engine/prepare_graspit.sh /graspit && rm -rf /graspit/.git

FROM scratch AS graspit
COPY --from=upstream /graspit/ /

FROM build-base
ARG BUILD_JOBS=3

ENV CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS} \
    GRASPIT=/opt/graspit \
    GRASPIT_SOURCE_DIR=/opt/graspit \
    GRASPIT_BUILD_DIR=/opt/graspit/build \
    VIRTUAL_ENV=/opt/venv \
    PATH=/opt/venv/bin:$PATH \
    QT_QPA_PLATFORM=xcb

COPY --from=graspit . /opt/graspit
RUN cmake -S /opt/graspit -B /opt/graspit/build \
        -DCMAKE_BUILD_TYPE=Release -DOPTIMIZATION_SOLVER=NONE \
    && cmake --build /opt/graspit/build --target graspit -j"${BUILD_JOBS}"

RUN python3 -m venv /opt/venv \
    && pip install --no-cache-dir setuptools wheel numpy scipy pytest "mujoco>=3.1"

WORKDIR /src
COPY . /src
RUN pip wheel --no-build-isolation --no-deps . --wheel-dir /dist \
    && pip install --no-deps /dist/graspit_py-*.whl

# Run from outside /src so tests import the installed wheel, not the source tree.
WORKDIR /work
# xvfb-run must not be PID 1 (it waits on a signal PID 1 never receives).
ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["xvfb-run", "-a", "python", "-m", "pytest", "-q", "/src/tests"]
