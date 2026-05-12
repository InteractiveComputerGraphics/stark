# Settings

All Stark configuration lives in a single `stark::Settings` struct that is passed to the `Simulation` constructor.
Once the simulation is constructed, settings are immutable.

## C++ Usage

```cpp
stark::Settings settings;

// Output
settings.output.simulation_name    = "my_sim";
settings.output.output_directory   = "/tmp/my_sim_output";
settings.output.codegen_directory  = "/tmp/codegen";
settings.output.fps                = 30;
settings.output.console_verbosity  = symx::Verbosity::Summary;

// Simulation physics
settings.simulation.gravity                 = {0.0, 0.0, -9.81};
settings.simulation.max_time_step_size      = 1.0 / 60.0;
settings.simulation.use_adaptive_time_step  = true;

// Execution limits
settings.execution.end_simulation_time = 5.0;
settings.execution.n_threads           = 8;  // -1 = use all available

stark::Simulation simulation(settings);
```

## Python Usage

```python
import pystark

settings = pystark.Settings()
settings.output.simulation_name   = "my_sim"
settings.output.output_directory  = "/tmp/my_sim_output"
settings.output.codegen_directory = "/tmp/codegen"
settings.simulation.gravity       = np.array([0.0, 0.0, -9.81])
settings.execution.n_threads      = 8

simulation = pystark.Simulation(settings)
```

## Output Settings

| Field | Default | Description |
|---|---|---|
| `simulation_name` | `""` | Used as a prefix for all output files |
| `output_directory` | `""` | Directory where frame VTK files and logs are written |
| `codegen_directory` | `""` | Directory for SymX-generated C++ files (cached between runs) |
| `time_stamp` | *(creation time)* | Appended to log file names to avoid collisions |
| `fps` | `30` | Output frame rate; `0` = no frame writes; `-1` = write every time step |
| `console_verbosity` | `Summary` | Verbosity of console output: `Minimal`, `Summary`, `Medium`, `Full` |
| `file_verbosity` | `Full` | Verbosity of the log file |
| `enable_frame_writes` | `true` | Toggle frame VTK output |
| `enable_output` | `true` | Toggle all output (console + file) |

## Simulation Settings

| Field | Default | Description |
|---|---|---|
| `gravity` | `{0, 0, -9.81}` | Global gravity vector (m/s²) |
| `init_frictional_contact` | `true` | Initialize the IPC contact system even if no contact pairs are registered; set to `false` for small speedups when contact is not needed |
| `max_time_step_size` | `1/120 s` | Upper bound on the time step; also the initial dt |
| `use_adaptive_time_step` | `true` | Halve dt when Newton's Method fails to converge; double back on success (up to `max_time_step_size`) |
| `time_step_size_success_multiplier` | `1.05` | Factor by which dt grows after a successful step |
| `time_step_size_lower_bound` | `1e-6 s` | Simulation stops with an error if dt drops below this |

## Newton's Method Settings

These are passed through as `symx::NewtonSettings`.
The most commonly tuned fields are:

| Field | Default | Description |
|---|---|---|
| `max_iterations` | `INT_MAX` | Per-step Newton iteration limit (effectively unlimited) |
| `residual_tolerance_abs` | `1e-6` | Absolute force residual convergence threshold |
| `residual_tolerance_rel` | `0` | Relative convergence threshold (disabled by default) |
| `max_armijo_iterations` | `20` | Max Armijo backtracking steps per Newton iteration |
| `cg_rel_tolerance` | `1e-4` | Relative CG convergence tolerance for the linear system solve |

See the [SymX Newton's Method docs](https://github.com/InteractiveComputerGraphics/SymX) for the full list of options.

## Execution Settings

| Field | Default | Description |
|---|---|---|
| `allowed_execution_time` | `∞` | Wall-clock time limit in seconds; simulation stops when exceeded |
| `end_simulation_time` | `∞` | Simulated time at which to stop |
| `end_frame` | `INT_MAX` | Frame count at which to stop |
| `n_threads` | `-1` | Number of OpenMP threads; `-1` = use all available cores |
