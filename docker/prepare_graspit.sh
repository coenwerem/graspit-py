#!/usr/bin/env bash
# Fetch the public engine and apply the wrapper's source compatibility changes.
set -euo pipefail

if [[ $# -ne 1 ]]; then
    printf 'Usage: %s <new-source-directory>\n' "$0" >&2
    exit 2
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
destination=$1
upstream=https://github.com/graspit-simulator/graspit.git
revision=de64377541c4b4b2987c72d0a17c02464f9c7095

mkdir -p -- "$(dirname -- "$destination")"
mkdir -- "$destination"
git -C "$destination" init --quiet
git -C "$destination" fetch --quiet --depth 1 "$upstream" "$revision"
git -C "$destination" checkout --quiet --detach FETCH_HEAD
git -C "$destination" apply --whitespace=fix "$script_dir/graspit-engine.patch"
