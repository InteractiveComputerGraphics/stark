# 1) If you don't already have Miniforge installed (pick ONE)
# Linux/macOS (example name; adjust to your OS/arch from conda-forge/miniforge releases)
# bash Miniforge3-<OS>-<ARCH>.sh -b -p "$HOME/miniforge3"
# eval "$($HOME/miniforge3/bin/conda shell.bash hook)"

# 2) Make sure conda-forge is used (recommended with Miniforge)
conda config --add channels conda-forge
conda config --set channel_priority strict

# 3) Create the docs environment (choose python version if you want)
conda create -n docs python=3.11 -y


# 4) Activate it
conda activate docs

# 5) Install Sphinx + autobuild (and common extensions)
conda install -c conda-forge sphinx myst-parser furo sphinx-autobuild sphinxcontrib-mermaid -y

# Optional but common (uncomment what you need):

# conda install -y sphinxcontrib-bibtex sphinxcontrib-mermaid
# conda install -y doxygen breathe  # if you document C++ with Doxygen+Breathe

# 6) (Only once) create a Sphinx project skeleton in your docs folder
# mkdir -p docs && cd docs
# sphinx-quickstart

# 7) Run your live-reload server (your final commands)
sphinx-autobuild source build/html --fresh-env --write-all
