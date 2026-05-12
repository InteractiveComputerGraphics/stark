#!/usr/bin/env bash
# Sphinx docs build helper for Stark
# Run from anywhere; the script resolves its own location.

# -- Environment Setup --
# If you don't already have Miniforge installed:
#   bash Miniforge3-Linux-x86_64.sh -b -p "$HOME/miniforge3"
#   eval "$($HOME/miniforge3/bin/conda shell.bash hook)"

# 1) Add conda-forge channel (recommended with Miniforge)
# conda config --add channels conda-forge
# conda config --set channel_priority strict

# 2) Create and activate the docs environment (one-time)
# conda create -n docs python=3.11 -y
# conda activate docs

# 3) Install Sphinx and required extensions (one-time)
# conda install -c conda-forge sphinx myst-parser furo sphinx-autobuild sphinxcontrib-mermaid -y

# -- Build Commands --

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="$SCRIPT_DIR/source"
BUILD_DIR="$SCRIPT_DIR/build/html"

# Live-reload development server
sphinx-autobuild "$SOURCE_DIR" "$BUILD_DIR" --fresh-env --write-all

# One-shot build (uncomment to use instead):
# sphinx-build -b html "$SOURCE_DIR" "$BUILD_DIR"
