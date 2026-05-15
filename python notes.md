# Python notes for STARK

## Generate stubs
https://stackoverflow.com/questions/73879484/vscode-not-autocompleting-python-from-module-made-with-pybind11

pip install pybind11-stubgen
$env:PYTHONPATH = "$env:PYTHONPATH;$PWD"
pybind11-stubgen.exe pystark --ignore-all-errors

## CMake build from source
STARK_BUILD_PYTHON_BINDINGS ON
STARK_PYTHON_EXECUTABLE /home/fernandez/miniforge3/envs/notbase/bin/python

## Build wheels locally
Note that you have to be out of a conda env for this to work
> bash scripts/build_wheel_local_linux.sh --clean

This ends up in `dist/`

## Test local wheels
> scripts/test_wheels.sh dist/stark_sim-0.1.0-cp312-cp312-linux_x86_64.whl

### Manual test
Move the wheels somewhere isolated like `~/Desktop/wheels_test`

```bash
conda create -y -n stark-manual-test python=3.12 pip
conda activate stark-manual-test

python -m pip install --upgrade pip
python -m pip install ./stark_sim-0.1.0-*.whl

python - <<'PY'
import pystark
pystark.run_test_sim()
PY
```

## Build all the wheels combos in github
This is done by the script in .github/workflows/wheels.yaml
I need to push anything to github. Consider a `pre-release` branch



