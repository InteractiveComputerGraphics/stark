# Setup

STARK requires **CMake 3.15+**, a **C++17** compiler, and **OpenMP**.
It bundles Eigen, fmt, and SymX as dependencies — nothing else needs to be installed for the C++ build.
Python bindings (pystark) require Python 3.8+ and are built separately.

## Project Structure

| Folder | Contents |
|---|---|
| `stark/` | The core C++ library |
| `stark/extern/symx/` | SymX symbolic differentiation engine (bundled) |
| `examples/` | Self-contained C++ example scenes |
| `pystark/` | Python bindings (nanobind) |
| `tests/` | C++ unit tests |
| `docs/` | This documentation |

## Building the C++ Library and Examples

Configure with CMake (Release by default):

```bash
cmake -B build
```

Build:

```bash
cmake --build build --parallel                   # Everything
cmake --build build --parallel --target examples # Only examples
cmake --build build --parallel --target tests    # Only tests
```

Run the examples:

```bash
./build/examples/examples
```

Output files (VTK/OBJ meshes per frame) and logs are written to the directory configured in `settings.output.output_directory`.
Generated C++ code from SymX is written to `settings.output.codegen_directory` and is cached between runs.

## CMake Options

| Option | Default | Description |
|---|---|---|
| `STARK_ENABLE_EXAMPLES` | `ON` | Build the example scenes |
| `STARK_ENABLE_TESTS` | `ON` | Build the test suite |
| `STARK_BUILD_PYTHON_BINDINGS` | `ON` | Build the pystark Python bindings |

STARK inherits SymX CMake options for AVX2, compiler path, and Hessian storage.
See the [SymX setup docs](https://github.com/InteractiveComputerGraphics/SymX) for details on those.

## Building pystark (Python Bindings)

pystark uses [nanobind](https://github.com/wjakob/nanobind) (fetched automatically at configure time) and is built as part of the main CMake project via the `STARK_BUILD_PYTHON_BINDINGS` option (enabled by default).

**Prerequisites:** Python 3.8+ with development headers and NumPy.

```bash
# Tell CMake which Python to use (required when using conda/virtualenv)
cmake -B build -DPython_EXECUTABLE=$(which python)
cmake --build build --parallel --target pystark
```

This produces a shared library and places it directly into `pystark/pystark/` so the Python package is immediately importable.

To make `import pystark` work, add the `pystark/` source directory to `PYTHONPATH`:

```bash
export PYTHONPATH=/path/to/stark/pystark:$PYTHONPATH
```

Then verify:

```python
import pystark
s = pystark.Settings()
print(s.as_string())
```

To disable pystark when building only the C++ library:

```bash
cmake -B build -DSTARK_BUILD_PYTHON_BINDINGS=OFF
```

The Python package at `pystark/pystark/__init__.py` re-exports everything from the native module and adds convenience aliases (e.g. `pystark.ZERO`, `pystark.UNITX`) and utilities (`pystark.blend`).

## Requirements Summary

| Component | Requirement |
|---|---|
| CMake | 3.15+ |
| C++ compiler | C++17, with OpenMP support |
| Python (pystark) | 3.8+ |
| NumPy (pystark) | Any recent version |

### Linux
GCC 9+ or Clang 9+ recommended. OpenMP is usually available via `libgomp` (GCC) or `libomp` (Clang).

### macOS
Apple Clang does not ship OpenMP by default. Install via Homebrew:
```bash
brew install libomp
```
then pass the include/lib paths to CMake.

### Windows
MSVC 2019+ with the C++ and CMake workloads installed.
OpenMP support is enabled by the `/openmp` flag (added automatically by CMake's `FindOpenMP`).
