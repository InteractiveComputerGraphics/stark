#!/usr/bin/env bash
set -euo pipefail

# Clear existing packages
rm -f pystark/pystark/*.so
rm -rf build dist wheelhouse *.egg-info pystark/*.egg-info
rm -rf dist build _skbuild

# Install building tools
python -m pip install --upgrade build scikit-build-core cmake ninja

# Build
python -m build --wheel
ls -lh dist/*.whl

