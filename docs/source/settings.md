# Settings

All STARK configuration is collected in a single `stark::Settings` object that is passed to the `stark::Simulation` constructor.
STARK owns the output, simulation, and execution settings, while the nonlinear solver settings are provided by SymX through `symx::NewtonSettings`.
At minimum, a simulation should provide a name and an output directory.

It is **strongly recommended** that you take a look at all the options available.
Depending on the problem you might want to use specific tolerances, linear solvers, time step size, etc.
The following is a typical lean configuration that relies almost entirely on defaults:

```cpp
#include <stark>

int main()
{
    stark::Settings settings;

    // Required output settings
    settings.output.simulation_name = "hanging_cloth";
    settings.output.output_directory = OUTPUT_PATH + "/hanging_cloth";

    // Typical simulation controls
    settings.execution.end_simulation_time = 5.0;
    settings.simulation.max_time_step_size = 1.0 / 30.0;
    settings.simulation.init_frictional_contact = false;
    settings.newton.step_tolerance = 1e-3;

    stark::Simulation simulation(settings);
    simulation.run();
}
```

## Structure

`stark::Settings` is organized into four sections:

| Section | Type | Purpose |
|---|---|---|
| `settings.output` | `stark::Settings::Output` | Paths, frame writing, console output, and log files. |
| `settings.simulation` | `stark::Settings::Simulation` | Global physical and time-stepping parameters. |
| `settings.newton` | `symx::NewtonSettings` | SymX nonlinear solver, line search, Hessian projection, and linear solver settings. |
| `settings.execution` | `stark::Settings::Execution` | Runtime limits and thread count. |

STARK applies a few defaults in `Settings::Settings()` after the raw member initializers are constructed.
In particular, it sets the timestamp, chooses a code-generation directory from SymX, sets the default thread count, and overwrites some SymX Newton defaults for STARK simulations.
The tables below list the effective STARK defaults.

## Output settings

Output settings control where STARK writes generated code, logs, and frame data.
`simulation_name`, `output_directory`, and `codegen_directory` must be non-empty when the internal `Stark` object is constructed.
By default, `codegen_directory` is filled from `symx::get_codegen_dir()`.

| Field | Effective default | Description |
|---|---:|---|
| `simulation_name` | `""` | Name of the simulation. Used in log and frame file names. Must be set by the user. |
| `output_directory` | `""` | Directory where STARK writes logs and frame output. Must be set by the user. |
| `codegen_directory` | `symx::get_codegen_dir()` | Directory used by SymX for generated and compiled code. Cached code can be reused between runs. |
| `time_stamp` | creation time | Timestamp string generated when `Settings` is constructed, formatted as `YYYY-MM-DD__hh-mm-ss`. Used to avoid log-file collisions. |
| `fps` | `30` | Frame output rate in simulation frames per second. Use `fps < 0` to write every accepted time step, and `fps == 0` to disable frame-write callbacks. |
| `console_verbosity` | `symx::Verbosity::Summary` | Console verbosity. Common values are `Minimal`, `Summary`, `Medium`, and `Full`. |
| `file_verbosity` | `symx::Verbosity::Full` | Verbosity of the `.log` file written next to the simulation output. |
| `enable_frame_writes` | `true` | Enables frame writing after accepted time steps. Useful to disable heavy VTK/output callbacks while keeping the simulation and logs active. |
| `enable_output` | `true` | Enables the SymX output sink. Set to `false` to silence normal console/file output. |

## Simulation settings

Simulation settings control global physical parameters and adaptive time stepping.
The time step starts at `max_time_step_size`.
When a step is solved successfully, STARK advances the simulation and may increase `dt` up to this maximum.
When Newton fails because the step is too difficult, STARK halves `dt` and retries, unless adaptive time stepping is disabled.

| Field | Effective default | Description |
|---|---:|---|
| `gravity` | `{0.0, 0.0, -9.81}` | Global gravity acceleration used by STARK systems that query the simulation gravity. Can also be changed later with `simulation.set_gravity(...)`. |
| `init_frictional_contact` | `true` | Initializes the frictional contact infrastructure. Set to `false` for examples without contact to reduce setup/runtime overhead. |
| `max_time_step_size` | `1.0 / 30.0` | Maximum time step size, and also the initial `dt`. |
| `use_adaptive_time_step` | `true` | If `true`, failed difficult steps reduce `dt` and retry. If `false`, such failures stop the simulation. |
| `time_step_size_success_multiplier` | `1.05` | Multiplicative growth factor applied to `dt` after a successful accepted step, clamped by `max_time_step_size`. |
| `time_step_size_lower_bound` | `1e-6` | Minimum allowed adaptive time step. If `dt` drops below this value, STARK stops with an error. |

## Newton settings

`settings.newton` is a `symx::NewtonSettings` object.
These settings are passed to SymX when STARK creates the internal Newton solver during initialization.

### Iteration and convergence

| Field | Effective default | Description |
|---|---:|---|
| `max_iterations` | `INT_MAX` | Maximum Newton iterations per solve. The default is effectively unlimited. |
| `min_iterations` | `0` | Minimum Newton iterations before convergence is accepted. |
| `residual_tolerance_abs` | `1e-6` | Absolute convergence tolerance on the nonlinear residual. |
| `residual_tolerance_rel` | `0.0` | Relative convergence tolerance on the nonlinear residual. Disabled by default. |
| `step_tolerance` | `1e-3` | Convergence tolerance on the Newton step size. STARK overwrites the raw SymX default of `1e-6`. |
| `max_iterations_as_success` | `false` | If `true`, reaching `max_iterations` is treated as a successful solve instead of a failure. |

### Line search

| Field | Effective default | Description |
|---|---:|---|
| `step_cap` | `infinity` | Maximum allowed Newton step norm. Use this to limit aggressive updates in difficult problems. |
| `enable_armijo_backtracking` | `true` | Enables Armijo backtracking for sufficient energy decrease. |
| `line_search_armijo_beta` | `1e-4` | Armijo sufficient-decrease parameter. Smaller values accept weaker agreement with the local model. |
| `max_backtracking_armijo_iterations` | `20` | Maximum number of Armijo backtracking iterations per Newton step. |
| `max_backtracking_invalid_state_iterations` | `8` | Maximum number of backtracking iterations caused by invalid intermediate states, such as contact penetration. |
| `print_line_search_upon_failure` | `false` | Prints additional line-search information when a solve fails. Useful for debugging. |

### Hessian projection

SymX can modify local Hessians to obtain a positive-definite global system for Newton-like descent directions.
This is important for robust implicit time stepping, especially with nonlinear materials and contact.

| Field | Effective default | Description |
|---|---:|---|
| `projection_mode` | `Progressive` | Hessian projection strategy. STARK overwrites the raw SymX default of `ProjectedNewton`. |
| `projection_eps` | `1e-10` | Minimum eigenvalue used by projection. |
| `project_to_pd_use_mirroring` | `false` | If `true`, negative eigenvalues are mirrored instead of clamped. |
| `project_on_demand_countdown` | `4` | Number of iterations to continue projecting after a projection-triggering failure in `ProjectOnDemand` mode. |
| `ppn_tightening_factor` | `0.5` | Tightening factor used by progressive projected Newton after non-descending behavior. |
| `ppn_release_factor` | `2.0` | Release factor used by progressive projected Newton after successful behavior. |

Available projection modes:

| Value | Meaning |
|---|---|
| `symx::ProjectionToPD::Newton` | Pure Newton. Do not project Hessians. Fast, but not guaranteed to produce descent directions for indefinite problems. |
| `symx::ProjectionToPD::ProjectedNewton` | Always project local Hessians to positive definite before assembly. Robust, but potentially more expensive. |
| `symx::ProjectionToPD::ProjectOnDemand` | Start without projection and enable it after solver failures or non-descending directions. |
| `symx::ProjectionToPD::Progressive` | Progressively adjusts projection based on solve behavior. This is STARK's default. |

### Linear solver

| Field | Effective default | Description |
|---|---:|---|
| `linear_solver` | `symx::LinearSolver::BDPCG` | Linear solver used for the Newton system. |
| `cg_max_iterations` | `10000` | Maximum conjugate-gradient iterations. Used by iterative solvers. |
| `cg_abs_tolerance` | `1e-12` | Absolute CG residual tolerance. |
| `cg_rel_tolerance` | `1e-4` | Relative CG residual tolerance. |
| `cg_stop_on_indefiniteness` | `true` | Stops CG when indefiniteness is detected. This can trigger projection or failure handling depending on the projection mode. |
| `bailout_residual` | `1e-10` | Skips taking a step if the nonlinear residual is already below this value, avoiding unnecessary unstable linear solves near convergence. |

Available linear solvers:

| Value | Meaning |
|---|---|
| `symx::LinearSolver::BDPCG` | Block-diagonal preconditioned conjugate gradient. Default. |
| `symx::LinearSolver::DirectLLT` | Direct LLT solve. Useful for smaller problems or debugging. |

## Execution settings

Execution settings control when the simulation stops and how many CPU threads are forwarded to SymX.
They are independent of physical time-stepping parameters.

| Field | Effective default | Description |
|---|---:|---|
| `allowed_execution_time` | `std::numeric_limits<double>::max()` | Wall-clock runtime limit in seconds. STARK stops when this limit is exceeded. |
| `end_simulation_time` | `std::numeric_limits<double>::max()` | Simulated-time limit. Commonly set in examples, e.g. `settings.execution.end_simulation_time = 5.0;`. |
| `end_frame` | `std::numeric_limits<int>::max()` | Frame-count limit. STARK stops when the frame counter exceeds this value. |
| `n_threads` | `omp_get_max_threads() / 2` | Number of threads forwarded to the SymX context. The raw member initializer is `-1`, but STARK replaces it at construction time with half of the OpenMP maximum. Set this explicitly for reproducible CPU usage. |
