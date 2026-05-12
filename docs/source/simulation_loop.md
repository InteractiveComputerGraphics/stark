# Simulation Loop

This page explains how Stark advances time, how to script kinematic motion, and how to inject custom logic into the loop via callbacks.

## Running the Simulation

### `run(duration)`

The most common entry point.
Runs the simulation until `duration` seconds of simulated time have elapsed (or any other termination condition in [Settings](settings.md) is met).

```cpp
// C++
simulation.run(10.0);

// With a per-step callback (called once per Newton step attempt)
simulation.run(10.0, [&]() {
    // Custom logic here
});
```

```python
# Python
simulation.run(10.0)
simulation.run(10.0, lambda: my_callback())
```

### `run_one_time_step()`

Advances the simulation by exactly one time step (which may be retried internally by the adaptive time step scheme).
Useful when you want to control the outer loop yourself.

```cpp
while (my_condition()) {
    simulation.run_one_time_step();
}
```

## Time Events

Time events are the primary way to script kinematic motion.
They fire every time step within the given `[t0, t1]` interval.

```cpp
// C++
simulation.add_time_event(t0, t1, [&](double t) {
    my_handler.set_transformation(position(t), angle_deg(t), axis);
});
```

```python
# Python
simulation.add_time_event(t0, t1, lambda t: my_handler.set_transformation(...))
```

The callback receives the current simulated time `t`.
Multiple time events can overlap; they all fire in registration order.

### `EventInfo`

A more detailed overload gives you `EventInfo`:

```cpp
simulation.add_time_event(t0, t1, [&](double t, stark::EventInfo& info) {
    // info.t0, info.t1 — event interval
    // info.is_first   — true on the first call (t ≈ t0)
    // info.is_last    — true on the last call  (t ≈ t1)
    if (info.is_first) { /* initialization */ }
    my_handler.set_transformation(...);
});
```

## Scripting Kinematic Objects

The typical pattern for scripted motion is:

1. Add a kinematic constraint (e.g. `add_constraint_fix` for rigid bodies, `prescribed_positions` for deformables).
2. Store the returned handler.
3. Call `set_transformation` (or equivalent) inside a time event.

```cpp
// Rigid body spinning in place
auto fix = simulation.rigidbodies->add_constraint_fix(box.rigidbody);

simulation.add_time_event(0.0, duration, [&](double t) {
    fix.set_transformation(
        Eigen::Vector3d(0, 0, 0),   // pivot (world)
        t * 90.0,                    // angle in degrees
        Eigen::Vector3d(0, 0, 1)    // axis
    );
});
```

```python
# Deformable prescribed position
left = simulation.deformables().prescribed_positions().add_inside_aabb(...)

simulation.add_time_event(0.0, duration,
    lambda t: left.set_transformation(np.zeros(3), t * 90.0, np.array([1,0,0]))
)
```

## Gravity

Gravity can be changed at run time:

```cpp
// C++
simulation.set_gravity({0.0, 0.0, -9.81});
```

```python
# Python
simulation.set_gravity(np.array([0.0, 0.0, -9.81]))
```

This is useful for scenes where gravity is ramped up gradually (e.g. to avoid explosive initial conditions).

## Callbacks (Advanced)

For lower-level control, Stark exposes a callback system.
These are mainly used internally by the physics models, but you can register your own.

| Callback | When it fires |
|---|---|
| `add_before_simulation` | Once, just before the first time step (after SymX compilation) |
| `add_before_time_step` | At the start of every time step attempt |
| `add_after_time_step` | After every accepted time step |
| `add_on_time_step_accepted` | After Newton converges, before advancing time |
| `add_write_frame` | When Stark decides to output a frame |
| `add_should_continue_execution` | Every step; return `false` to stop the simulation |

```cpp
// C++ (via stark.callbacks, available on core::Stark)
// In typical usage you access this through Simulation internals or model code
```

For most user scenarios, time events cover everything you need.
Callbacks are mainly relevant when you are [extending Stark](extending.md) with new models.

## Query Methods

At any point during the simulation loop (including inside time events and callbacks) you can query the current simulation state:

```cpp
double t     = simulation.get_time();
double dt    = simulation.get_time_step_size();
int    frame = simulation.get_frame();
```

```python
t     = simulation.get_time()
dt    = simulation.get_time_step_size()
frame = simulation.get_frame()
```
