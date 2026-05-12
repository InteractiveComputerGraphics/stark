# Python API (pystark)

pystark exposes the full Stark C++ API to Python via [nanobind](https://github.com/wjakob/nanobind) bindings.
The API is intentionally kept parallel to C++ — almost every class and method has a 1:1 counterpart.

## Installation

Build pystark as part of the main CMake project (enabled by default with `STARK_BUILD_PYTHON_BINDINGS=ON`):

```bash
# From the repo root — specify your Python interpreter explicitly
cmake -B build -DPython_EXECUTABLE=$(which python)
cmake --build build --parallel --target pystark
```

Then set `PYTHONPATH` to include both the native `.so` directory and the Python package wrapper:

```bash
export PYTHONPATH=/path/to/stark/pystark:$PYTHONPATH
```

Verify the install:

```python
import pystark
settings = pystark.Settings()
print(settings.as_string())
```

See [Setup](setup.md) for full build instructions including conda/virtualenv and platform-specific notes.

The `pystark/pystark/` folder is a thin Python wrapper (`__init__.py`) that re-exports everything from the native module and adds convenience aliases (`pystark.ZERO`, `pystark.UNITX`, etc.) and utilities (`pystark.blend`).

## API Differences from C++

### Accessor Methods vs. Direct Fields

In C++, subsystems are public fields accessed with `->`:

```cpp
simulation.deformables->prescribed_positions->add_inside_aabb(...)
simulation.rigidbodies->add_constraint_fix(rb)
simulation.interactions->contact->set_global_params(params)
simulation.presets->deformables->add_surface_grid(...)
```

In Python, they are accessor methods:

```python
simulation.deformables().prescribed_positions().add_inside_aabb(...)
simulation.rigidbodies().add_constraint_fix(rb)
simulation.interactions().contact().set_global_params(params)
simulation.presets().deformables().add_surface_grid(...)
```

### NumPy Arrays

All `Eigen::Vector3d`, `Eigen::Vector2d`, `Eigen::Matrix3d`, and `Eigen::Quaterniond` arguments accept NumPy arrays:

```python
rb.set_translation(np.array([0.0, 0.0, 1.0]))
rb.set_rotation(45.0, np.array([0.0, 0.0, 1.0]))   # angle_deg, axis
rb.add_force_at_centroid(np.array([0.0, 0.0, -9.81 * mass]))
```

Return values are also NumPy arrays:

```python
pos   = rb.get_translation()   # np.ndarray shape (3,)
quat  = rb.get_quaternion()    # np.ndarray shape (4,) — [x, y, z, w]
R     = rb.get_rotation_matrix()  # np.ndarray shape (3, 3)
```

### `std::array<int, N>` and `std::vector`

Connectivity arrays (triangles, segments, tets) are passed as nested Python lists or NumPy integer arrays:

```python
triangles = [[0, 1, 2], [1, 3, 2], ...]
tets      = np.array([[0,1,2,3], ...], dtype=np.int32)
```

### Lambda Callbacks

C++ lambdas become Python callables (functions or lambdas):

```python
# C++: simulation.add_time_event(0, dur, [&](double t) { ... });
simulation.add_time_event(0.0, duration, lambda t: my_update(t))
```

### `run()` Signatures

```python
simulation.run(duration)                      # run for 'duration' seconds
simulation.run(duration, lambda: callback())  # with per-step callback (no args)
simulation.run(lambda: callback())            # run until end time set in Settings
simulation.run_one_time_step()                # advance exactly one time step
```

## Convenience Utilities

### `pystark.blend`

Smooth interpolation between two values over a time interval:

```python
import pystark

v = pystark.blend(
    v0=0.0, v1=1.0,
    t0=0.0, t1=2.0,
    t=current_time,
    blend_type=pystark.BlendType.EaseInOut
)
```

Available blend types: `Linear`, `EaseIn`, `EaseOut`, `EaseInOut`.

## Example: Boxes on Cloth

A minimal working example (from `pystark/examples/boxes_on_cloth.py`):

```python
import numpy as np
import pystark

settings = pystark.Settings()
settings.output.simulation_name   = "boxes_on_cloth"
settings.output.output_directory  = "output_folder"
settings.output.codegen_directory = "codegen_folder"
simulation = pystark.Simulation(settings)

# Contact parameters
thickness = 0.00025
contact_params = pystark.EnergyFrictionalContact.GlobalParams()
contact_params.default_contact_thickness = thickness
contact_params.min_contact_stiffness = 2e8
simulation.interactions().contact().set_global_params(contact_params)

# Static floor
fV, fC, fH = simulation.presets().rigidbodies().add_box(
    "floor", 1.0, np.array([1.0, 10.0, 0.05]))
fH.rigidbody.add_translation(np.array([0.0, 4.0, -0.05/2.0 - thickness]))
simulation.rigidbodies().add_constraint_fix(fH.rigidbody).set_stiffness(1e8)

# Cloth surface
cloth_material = pystark.Surface.Params.Cotton_Fabric()
cV, cC, cH = simulation.presets().deformables().add_surface_grid(
    "cloth", np.array([0.5, 0.5]), np.array([20, 20]), cloth_material)
simulation.interactions().contact().set_friction(fH.contact, cH.contact, 0.5)

# Fixed cloth edge
cloth_bc = simulation.deformables().prescribed_positions().add_inside_aabb(
    cH.point_set,
    np.array([0.0, 0.25, 0.0]),   # AABB center
    np.array([0.25, 0.001, 0.5]), # AABB half-extents
    pystark.EnergyPrescribedPositions.Params()
)

# A rigid box falling on the cloth
bV, bC, bH = simulation.presets().rigidbodies().add_box(
    "box", 1.0, np.array([0.08, 0.08, 0.08]))
bH.rigidbody.add_translation(np.array([0.0, 0.0, 0.2]))
simulation.interactions().contact().set_friction(bH.contact, cH.contact, 0.5)

simulation.run(3.0)
```

See `pystark/examples/` for complete, runnable examples:

| File | Scene |
|---|---|
| `boxes_on_cloth.py` | Stack of rigid boxes dropped on a cloth |
| `twisting_cloth.py` | Cloth twisted by two spinning rigid disks |
| `inflation.py` | Inflatable surface under internal pressure |
| `viscoelasticity.py` | Viscoelastic volumetric deformable |

## Serialization

`pystark.serialize` provides basic utilities for saving and loading simulation state.
See `pystark/pystark/serialize.py` for details.
