#!/usr/bin/env bash
set -euo pipefail

# Test STARK wheel artifacts in fresh temporary conda environments.
#
# Usage:
#   scripts/test_wheels.sh path/to/artifact.zip
#   scripts/test_wheels.sh path/to/wheel.whl
#   scripts/test_wheels.sh path/to/wheelhouse/
#
# The script:
#   - extracts/downloads wheels into a temp folder
#   - detects the required Python version from the wheel tag, e.g. cp312 -> Python 3.12
#   - creates a temporary conda env with that Python version
#   - installs the wheel
#   - checks import pystark
#   - destroys the temporary env/folder afterwards

if [[ "$#" -ne 1 ]]; then
    echo "Usage: $0 <artifact.zip | wheel.whl | wheel-directory>"
    exit 1
fi

INPUT="$1"
WORKDIR="$(mktemp -d /tmp/stark-wheel-artifact-test-XXXXXX)"

cleanup() {
    echo
    echo "Cleaning temporary test directory..."
    rm -rf "${WORKDIR}"
}
trap cleanup EXIT

echo "== STARK wheel artifact test =="
echo "Input:   ${INPUT}"
echo "Workdir: ${WORKDIR}"
echo

if ! command -v conda >/dev/null 2>&1; then
    echo "Error: conda was not found in PATH."
    echo "Install/load Miniforge or Miniconda before running this script."
    exit 1
fi

# Make conda usable in this non-interactive shell.
eval "$(conda shell.bash hook)"

mkdir -p "${WORKDIR}/wheels"

case "${INPUT}" in
    *.zip)
        echo "Extracting artifact zip..."
        unzip -q "${INPUT}" -d "${WORKDIR}/wheels"
        ;;
    *.whl)
        cp "${INPUT}" "${WORKDIR}/wheels/"
        ;;
    *)
        if [[ -d "${INPUT}" ]]; then
            cp "${INPUT}"/*.whl "${WORKDIR}/wheels/"
        else
            echo "Error: input is not a .zip, .whl, or directory: ${INPUT}"
            exit 1
        fi
        ;;
esac

mapfile -t WHEELS < <(find "${WORKDIR}/wheels" -name "*.whl" | sort)

if [[ "${#WHEELS[@]}" -eq 0 ]]; then
    echo "Error: no .whl files found."
    exit 1
fi

echo "Found wheels:"
for whl in "${WHEELS[@]}"; do
    echo "  $(basename "${whl}")"
done
echo

FAILURES=0
TESTED=0

for WHL in "${WHEELS[@]}"; do
    NAME="$(basename "${WHL}")"

    echo "------------------------------------------------------------"
    echo "Testing: ${NAME}"

    # Extract Python tag: cp310, cp311, cp312, ...
    if [[ "${NAME}" =~ -cp([0-9]{2,3})- ]]; then
        CP="${BASH_REMATCH[1]}"

        if [[ "${#CP}" -eq 2 ]]; then
            PY_MAJOR="${CP:0:1}"
            PY_MINOR="${CP:1:1}"
        else
            PY_MAJOR="${CP:0:1}"
            PY_MINOR="${CP:1:2}"
        fi

        PY_VERSION="${PY_MAJOR}.${PY_MINOR}"
    else
        echo "ERROR: could not infer CPython version from wheel name."
        FAILURES=$((FAILURES + 1))
        continue
    fi

    ENV_PREFIX="${WORKDIR}/conda-py${PY_VERSION}"

    echo "Creating temporary conda env:"
    echo "  Python: ${PY_VERSION}"
    echo "  Prefix: ${ENV_PREFIX}"

    conda create -y -p "${ENV_PREFIX}" "python=${PY_VERSION}" pip

    conda activate "${ENV_PREFIX}"

    echo
    echo "Using Python:"
    which python
    python --version

    echo
    echo "Installing wheel with binary-only dependencies..."
    python -m pip install --upgrade pip
    python -m pip install --only-binary=:all: "${WHL}"

    echo
    echo "Running import smoke test..."
    python - <<'PY'
import sys
import numpy
import pystark

print("Python:", sys.version)
print("NumPy:", numpy.__version__)
print("pystark:", pystark)

# settings = pystark.Settings()
# print("Settings OK:", type(settings))

print("pystark import smoke test OK")
PY

    echo
    echo "Running pip check..."
    python -m pip check

    conda deactivate

    TESTED=$((TESTED + 1))

    echo
    echo "OK: ${NAME}"
    echo
done

echo "------------------------------------------------------------"

if [[ "${FAILURES}" -ne 0 ]]; then
    echo "FAILED: ${FAILURES} wheel(s) failed."
    exit 1
fi

if [[ "${TESTED}" -eq 0 ]]; then
    echo "FAILED: no wheels were tested."
    exit 1
fi

echo "All tested wheels passed."
