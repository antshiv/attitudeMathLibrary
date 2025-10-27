#!/usr/bin/env bash

# generate_doxygen.sh
# Regenerates the API reference for the attitude math library.

set -euo pipefail

LIB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOXYFILE="${LIB_ROOT}/Doxyfile"
OUTPUT_DIR="${LIB_ROOT}/docs/api"

if ! command -v doxygen >/dev/null 2>&1; then
    echo "Error: doxygen is not installed or not in PATH." >&2
    exit 1
fi

if [[ ! -f "${DOXYFILE}" ]]; then
    echo "Error: Doxyfile not found at ${DOXYFILE}." >&2
    exit 1
fi

echo "Generating attitude math library documentation..."
echo "  Config : ${DOXYFILE}"
echo "  Output : ${OUTPUT_DIR}"

(
    cd "${LIB_ROOT}"
    doxygen "${DOXYFILE}"
)

echo "Done."
echo "Docs are available in ${OUTPUT_DIR}"
