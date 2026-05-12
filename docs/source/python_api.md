# Python API (pystark)

pystark exposes the full Stark C++ API to Python via [nanobind](https://github.com/wjakob/nanobind) bindings.
The API is intentionally kept parallel to C++ — almost every class and method has a 1:1 counterpart.

## Installation

See [Setup](setup.md) for the build instructions.
Once built, make the native module importable:

```bash
export PYTHONPATH=/path/to/stark/pystark/cpp/build:$PYTHONPATH
```

Then in Python:

```python
import pystark

# Check it works
settings = pystark.Settings()
print(settings.output.simulation_name)  # ""
```

The `pystark/pystark/` folder contains a thin Python wrapper (`__init__.py`) that re-exports everything and adds a few convenience utilities (e.g. `pystark.blend`).

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

### `run()` Callback

```python
simulation.run(10.0, lambda: per_step_callback())
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

## Example: Inflation

```python
import numpy as np
import pystark

settings = pystark.Settings()
settings.output.simulation_name   = "inflation"
settings.output.output_directory  = "output"
settings.output.codegen_directory = "codegen"
simulation = pystark.Simulation(settings)

contact_params = pystark.EnergyFrictionalContact.GlobalParams()
contact_params.default_contact_thickness = 0.001
simulation.interactions().contact().set_global_params(contact_params)

# Balloon surface
params = pystark.Surface.Params.Cotton_Fabric()
params.inertia.density = 50.0
vV, vC, vH = simulation.presets().deformables().add_surface_grid(
    "balloon", np.array([0.5, 0.5]), np.array([20, 20]), params
)

# Internal pressure via a time event
def apply_pressure(t):
    pressure = 1000.0 * min(t / 2.0, 1.0)  # ramp up over 2 s
    for i in range(len(vV)):
        # Add outward force proportional to pressure
        pass  # placeholder — see examples/inflation.py

simulation.add_time_event(0.0, 10.0, apply_pressure)
simulation.run(10.0)
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
