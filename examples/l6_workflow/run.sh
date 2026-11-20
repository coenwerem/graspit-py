#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" != 3 ]]; then
    echo "Usage: bash run.sh IMAGE DEXREACH_XARM7_L6_ASSETS OUTPUT_DIRECTORY" >&2
    exit 2
fi
example=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
assets=$(cd "$2" && pwd)
mkdir -p "$3"
output=$(cd "$3" && pwd)

docker run --rm --cpus=2 --memory=3g --user "$(id -u):$(id -g)" \
    -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
    -v "$example:/example:ro" -v "$assets:/assets:ro" -v "$output:/output" \
    --entrypoint sh "$1" -ec '
        Xvfb :79 -screen 0 1024x768x24 -nolisten tcp >/dev/null 2>&1 &
        display_pid=$!
        trap "kill $display_pid 2>/dev/null || true" EXIT
        export DISPLAY=:79
        retries=0
        while [ ! -S /tmp/.X11-unix/X79 ]; do
            retries=$((retries + 1))
            if [ "$retries" -ge 50 ]; then
                echo "Virtual X display failed to start" >&2
                exit 1
            fi
            sleep .1
        done
        python /example/run.py /example/l6-source.xml /output --assets /assets >/dev/null
    '
printf 'Saved %s/result.json\n' "$output"
