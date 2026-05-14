# Simulation Loop

This page explains how STARK advances time, how to script kinematic motion, and how to inject custom logic into the loop via callbacks.

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

The callback receives the current simulated time `t`.
Multiple time events can overlap; they all fire in registration order.


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


This is useful for scenes where gravity is ramped up gradually (e.g. to avoid explosive initial conditions).

## Callbacks (Advanced)

For lower-level control, STARK exposes a callback system.
These are mainly used internally by the physics models, but you can register your own.

| Callback | When it fires |
|---|---|
| `add_before_simulation` | Once, just before the first time step (after SymX compilation) |
| `add_before_time_step` | At the start of every time step attempt |
| `add_after_time_step` | After every accepted time step |
| `add_on_time_step_accepted` | After Newton converges, before advancing time |
| `add_write_frame` | When STARK decides to output a frame |
| `add_should_continue_execution` | Every step; return `false` to stop the simulation |

For most user scenarios, time events should cover everything you need.
Callbacks are mainly relevant when you are [extending STARK](extending.md) with new models.

## Query Methods

At any point during the simulation loop (including inside time events and callbacks) you can query the current simulation state:

```cpp
double t     = simulation.get_time();
double dt    = simulation.get_time_step_size();
int    frame = simulation.get_frame();
```

