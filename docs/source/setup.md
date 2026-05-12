# Setup

Stark requires **CMake 3.15+**, a **C++17** compiler, and **OpenMP**.
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

Stark inherits SymX CMake options for AVX2, compiler path, and Hessian storage.
See the [SymX setup docs](https://github.com/InteractiveComputerGraphics/SymX) for details on those.

## Building pystark (Python Bindings)

pystark uses [nanobind](https://github.com/wjakob/nanobind) and is built as a separate CMake project inside `pystark/cpp/`.

```bash
# From the repo root
cd pystark/cpp
cmake -B build
cmake --build build --parallel
```

This produces a shared library (`pystark*.so` or `.pyd` on Windows) in `pystark/cpp/build/`.

To make `import pystark` work, either:

- Add `pystark/cpp/build/` to your `PYTHONPATH`:
  ```bash
  export PYTHONPATH=/path/to/stark/pystark/cpp/build:$PYTHONPATH
  ```
- Or copy `pystark*.so` into your Python environment's site-packages.

A convenience wrapper at `pystark/pystark/__init__.py` re-exports everything from the native module.

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
