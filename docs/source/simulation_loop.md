# Simulation Loop

STARK advances a simulation by repeatedly solving one nonlinear optimization problem per time step.
The public API is intentionally simple:

```cpp
stark::Simulation simulation(settings);

// Build objects, materials, contact, boundary conditions, and output...

simulation.run();
```

Internally, each call to `run_one_time_step()` updates time-dependent data, runs Newton's method via SymX, commits or retries the step, writes frame output, and logs solver statistics.


## Optimization Problem

STARK does **not** solve directly for final _positions_.
For deformable point sets, the primary unknowns are the next-step velocities `v1`.
The next positions are reconstructed by first-order time integration:

```cpp
x1 = x0 + dt * v1;
```

For example, a deformable object contributes one 3D velocity unknown per point:

```text
soft.v1: 3267
```

Rigid bodies contribute linear and angular velocity unknowns:

```text
rigid.v1: 3
rigid.w1: 3
```

All energies are written as functions of the predicted next state.
A minimum of the global potential corresponds to a force balance, found by Newton's method.


## Initialization

The first time a simulation step is requested, STARK initializes the solver:

1. check that at least one degree of freedom was registered,
2. create the SymX Newton solver from the global potential,
3. compile/load the generated SymX kernels,
4. print the degrees of freedom,
5. write frame zero,
6. run `before_simulation` callbacks,
7. check that the initial state is valid.


## High-level time loop

A simplified version of `simulation.run()` is:

```cpp
while (!finished) {
    script.run_a_cycle(simulation.get_time());
    user_callback();              // optional callback passed to run(...)

    bool keep_running = stark.run_one_step();
    if (!keep_running) {
        break;
    }
}

stark.print_summary();
```

The loop stops when one of the configured limits is reached:

- `settings.execution.end_simulation_time`
- `settings.execution.end_frame`
- `settings.execution.allowed_execution_time`
- the duration passed to `simulation.run(duration)`
- a callback requests termination
- the solver fails in a non-recoverable way


## Newton's method inside each step

The nonlinear solve is handled by the SymX Newton solver.
Check the [original SymX docs](https://symx.physics-simulation.org/newtons_method.html) for more details.

The simplified inner loop is:

```text
check initial state validity

for each Newton iteration:
    run before_energy_evaluation callbacks
    evaluate energy, gradient, and local Hessians
    compute residual
    check residual convergence

    repeat:
        project local Hessians if needed
        assemble/update the global Hessian
        solve H * du = -grad
        check that du is a descent direction
        if not, increase projection and retry

    check step-size convergence

    line search:
        apply user step_cap
        apply max_allowed_step callbacks
        backtrack until intermediate state is valid
        backtrack until Armijo sufficient decrease holds

    run optional user convergence callbacks

check converged-state validity
```


### Hessian projection

STARK commonly runs with positive-definite Hessian projection enabled. This makes the Newton system more robust for simulation problems whose local Hessians may become indefinite.
Depending on the selected projection mode, SymX can:

- run pure Newton without projection,
- project all local Hessians,
- project only after a failed linear solve or non-descending direction,
- progressively project selected blocks based on the gradient magnitude.

The console field `ph` reports the percentage of projected local Hessians during a step.

### Line search

The line search has several layers:

| Stage | Meaning | Typical use |
|---|---|---|
| `cap` | Clamp the update by `settings.newton.step_cap` | Prevent overly large Newton steps |
| `max` | Clamp by `max_allowed_step` callbacks | Continuous collision detection and other hard step limits |
| `inv` | Backtrack while the intermediate state is invalid | Avoid inversions, penetrations, or other invalid states |
| `bt` | Armijo backtracking | Require sufficient energy decrease |

These are the four counters printed as

```text
ls (cap|max|inv|bt):  0| 0| 1| 4
```

Large `inv` counts usually indicate that the Newton direction is trying to pass through an invalid state. Large `bt` counts usually indicate that the quadratic model is too optimistic and Armijo has to shrink the step.

## Adaptive time step

The time step starts as

```cpp
dt = settings.simulation.max_time_step_size;
```

After a successful accepted step, STARK grows it by

```cpp
dt = min(max_time_step_size, dt * time_step_size_success_multiplier);
```

If Newton fails because the step is too difficult, STARK halves `dt` and retries the same simulation time:

```cpp
dt *= 0.5;
```

If `dt` drops below `settings.simulation.time_step_size_lower_bound`, the simulation exits.

Some failures do **not** immediately reduce `dt`. For example, a converged state may be rejected by a validity callback because a contact or prescribed-position constraint exceeded its tolerance. In that case, the responsible model may harden a stiffness parameter and STARK retries the same time step with the same `dt`.

This distinction is important:

| Failure type | What STARK does |
|---|---|
| invalid converged state | retry same time with modified model parameters |
| too many invalid intermediate states | run invalid-state callbacks, then retry if recoverable |
| too many Newton iterations | halve `dt` if adaptive stepping is enabled |
| Armijo failure | run Armijo-failure callbacks; halve `dt` or exit depending on settings |
| linear solve failure / non-descent | increase projection if possible; otherwise fail the step |

If adaptive stepping is disabled, a non-recoverable failed step exits the simulation instead of shrinking `dt`.

## Callbacks

Callbacks inject behavior into the solver loop.
Most users only need the scripting interface, but lower-level callbacks are useful when extending STARK.

### STARK-level callbacks

| Callback | When it runs | Purpose |
|---|---|---|
| `before_simulation` | once during initialization, after frame zero is written | initialize derived data |
| `before_time_step` | before Newton starts | reset unknowns, update forces, update time-step data |
| `on_time_step_accepted` | only after Newton succeeds and the state is accepted | commit the solved state |
| `after_time_step` | after accepting the step, before advancing output/logging is complete | post-step updates |
| `write_frame` | whenever a frame should be written | write VTK/output data |
| `should_continue_execution` | before each attempted step | allow systems/users to stop the simulation |

The frame-writing callback is how STARK's built-in output system writes registered meshes every frame.
The frame cadence is controlled by `settings.output.fps`.

### SymX/Newton callbacks
Check the [original SymX docs](https://symx.physics-simulation.org/newtons_method.html) for more details.

| Callback | When it runs | Purpose |
|---|---|---|
| `before_energy_evaluation` | before each energy or gradient/Hessian evaluation | update collision sets, derived fields, or time-dependent data |
| `is_initial_state_valid` | before Newton starts | reject invalid starting states |
| `max_allowed_step` | before line-search backtracking | impose step limits, commonly from CCD |
| `is_intermediate_state_valid` | during line search | reject invalid intermediate states |
| `on_intermediate_state_invalid` | after too many invalid-state backtracks | harden constraints/contact or report failure |
| `on_armijo_fail` | after too many Armijo backtracks | diagnostics or model adjustment |
| `is_converged` | after a line-search update | add custom convergence criteria |
| `is_converged_state_valid` | after Newton reports success | verify final constraints/contact tolerances |

Boolean validity callbacks are combined with logical `AND`. All registered checks must pass.
`max_allowed_step` callbacks are combined by taking the smallest returned step fraction.
The default residual used by Newton is the infinity norm of the gradient, unless a custom residual callback is installed.

## Public scripting and control

For ordinary scene control, use the public scripting interface rather than low-level core callbacks.

### Time events

A time event runs while `t0 <= simulation.get_time() < t1`:

```cpp
simulation.add_time_event(0.0, 2.0,
    [&](double t)
    {
        const double z = stark::blend(0.0, 1.0, 0.0, 2.0, t, stark::BlendType::Linear);
        boundary.set_transformation({0.0, 0.0, z}, Eigen::Matrix3d::Identity());
    }
);
```

This is the most convenient way to animate boundary conditions, gravity, material parameters, friction coefficients, target positions, rigid-body motors, or other time-dependent scene data.

### Events with state

For more advanced cases, the event callback can receive an `EventInfo` object:

```cpp
simulation.add_time_event(1.0, 3.0,
    [&](double t, stark::EventInfo& event)
    {
        if (event.is_first_call()) {
            event.msg = "activated";
        }

        // event.get_n_calls(), event.get_begin_time(), event.pack(...), event.unpack(...)
        // can be used to keep small pieces of state between calls.
    }
);
```

For full control, `simulation.get_script().add_event(...)` exposes the event system directly with custom `run_when` and `delete_when` predicates.

### Simple run callback

`simulation.run(callback)` executes `callback` before every attempted time step:

```cpp
simulation.run(
    [&]()
    {
        const Eigen::Vector3d center = magnet.rigidbody.get_translation();

        for (int i = 0; i < object.point_set.size(); ++i) {
            const Eigen::Vector3d x = object.point_set.get_position(i);
            const Eigen::Vector3d u = center - x;
            object.point_set.set_force(i, 0.1 * u.normalized() / u.squaredNorm());
        }
    }
);
```

Use this for simple procedural control that should run throughout the simulation. For time-bounded behavior, prefer `add_time_event`.

### Manual stepping

For interactive applications or external control loops, use:

```cpp
while (viewer_is_open) {
    simulation.run_one_time_step();
    // update viewer, UI, custom data, etc.
}
```

This executes one script cycle and then attempts one STARK time step.

## Quasistatic simulations

STARK supports quasistatic problems, though not as a first-class mode: it still solves for velocity-like unknowns using the same time-step machinery.
The trick is to choose parameters so the velocity unknown behaves as a positional increment.

The `quasistatic_column_extrusion()` example uses the following key changes:

```cpp
const double duration = 1.0;
const double dt = duration * 0.99999;  // Do not overflow the sim duration in a single step
const bool is_quasistatic = true;

settings.execution.end_simulation_time = duration;
settings.simulation.max_time_step_size = dt;
settings.output.fps = 1.0 / dt;

settings.newton.projection_mode = symx::ProjectionToPD::ProjectedNewton; // Advisable
settings.newton.project_to_pd_use_mirroring = true; // Advisable

stark::Volume::Params material;
material.inertia.quasistatic = true;  // Important!
```
