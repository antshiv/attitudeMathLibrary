#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${BUILD_DIR:-$project_root/build}"
samples="${SAMPLES:-2048}"

cmake -S "$project_root" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" --target attitude_shared -j"$(nproc)"
uv run --isolated --no-project \
    --with 'numpy>=2,<3' \
    --with 'scipy>=1.14,<2' \
    python "$project_root/tests/reference/test_scipy_quaternion_parity.py" \
    --library "$build_dir/libattitude.so" \
    --samples "$samples"
