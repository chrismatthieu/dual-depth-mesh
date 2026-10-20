#!/bin/bash
# Launch the dual-camera shaded depth mesh.
# Usage: ./run-dual-depth.sh [--range 2.5] [other rs-multicam-mesh args]
set -euo pipefail

root=$(cd "$(dirname "$0")" && pwd)
bin=""
for candidate in \
    "$root/build/rs-multicam-mesh" \
    "$root/build/Release/rs-multicam-mesh"
do
    if [[ -x "$candidate" ]]; then
        bin=$candidate
        break
    fi
done

if [[ -z "$bin" ]]; then
    echo "rs-multicam-mesh is not built." >&2
    echo "  cmake -S \"$root\" -B \"$root/build\" && cmake --build \"$root/build\"" >&2
    exit 1
fi

exec "$bin" "$@"
