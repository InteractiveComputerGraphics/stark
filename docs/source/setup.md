# Setup

STARK is a C++ library with optional Python bindings (`pystark`).
The core library, examples, tests, and bindings are all built with CMake.
STARK bundles its main third-party dependencies, including `SymX`, `Eigen`, `fmt`, `TriangleMeshCollisionDetection`, `vtkio`, `par_shapes`, and `tinyobjloader`.

For SymX-specific build options such as AVX2, compiler selection, code-generation folders, and Hessian storage precision, see the [SymX setup page](https://symx.physics-simulation.org/setup.html).

## Requirements

| Requirement | Notes |
|---|---|
| CMake | 3.18 or newer |
| C++ compiler | C++20-capable compiler |
| OpenMP | Required by the STARK C++ library |
| Python | Optional; Python 3.8+ with development-module support for `pystark` |
| NumPy | Optional; required when using `pystark` |

## Project structure

| Folder | Contents |
|---|---|
| `stark/` | Core C++ library |
| `examples/` | C++ example scenes |
| `tests/` | C++ unit tests |
| `pystark/` | Python package and nanobind bindings |
| `docs/` | Documentation source |

## Building the C++ library and examples

From the repository root:

```bash
cmake -S . -B build
cmake --build build --parallel
```

The examples are built by default:

```bash
./build/examples/examples
```

STARK defaults to a Release build when no build type is specified. To choose explicitly:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
```

## Optional tests

Tests are disabled by default. Enable them at configure time:

```bash
cmake -S . -B build -DSTARK_BUILD_TESTS=ON
cmake --build build --parallel --target stark_tests
./build/tests/stark_tests
```

## Optional Python bindings

The Python bindings are disabled by default. Enable them with:

```bash
cmake -S . -B build -DSTARK_BUILD_PYTHON_BINDINGS=ON
cmake --build build --parallel --target pystark
```

When using Conda, Miniforge, virtualenv, or multiple Python installations, explicitly select the Python executable:

```bash
cmake -S . -B build \
  -DSTARK_BUILD_PYTHON_BINDINGS=ON \
  -DSTARK_PYTHON_EXECUTABLE=$HOME/miniforge3/envs/ENVNAME/bin/python

cmake --build build --parallel --target pystark
```

`pystark` uses nanobind, which is fetched automatically by CMake.
The compiled extension module is written directly into the source-tree Python package:

```txt
pystark/pystark/
```

To import it from the source tree, add the package folder to `PYTHONPATH`:

```bash
export PYTHONPATH=/path/to/stark/pystark:$PYTHONPATH
```

You can also add the path to the built pystark manually from python by
```python
import sys
sys.path.append("path/to/stark/pystark")
```

Then verify:

```python
import pystark

settings = pystark.Settings()
print(settings.as_string())
```

If Python detection fails, either disable the bindings:

```bash
cmake -S . -B build -DSTARK_BUILD_PYTHON_BINDINGS=OFF
```

or point STARK to a Python environment with development-module support:

```bash
cmake -S . -B build \
  -DSTARK_BUILD_PYTHON_BINDINGS=ON \
  -DSTARK_PYTHON_EXECUTABLE=$HOME/miniforge3/envs/ENVNAME/bin/python
```

If CMake keeps finding the wrong Python, delete the build directory before reconfiguring:

```bash
rm -rf build
```

## CMake options

| Option | Default | Description |
|---|---:|---|
| `STARK_BUILD_TESTS` | `OFF` | Build the C++ test executable |
| `STARK_BUILD_PYTHON_BINDINGS` | `OFF` | Build the `pystark` Python extension module |
| `STARK_PYTHON_EXECUTABLE` | unset | Explicit Python executable used when building `pystark` |
| `CMAKE_BUILD_TYPE` | `Release` | Build type for single-config generators |

STARK also forwards or inherits relevant SymX options, including SIMD/JIT/compiler-related settings.
For those, use the SymX setup page as the reference.

