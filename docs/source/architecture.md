# Architecture

STARK's public API is straightforward despite the internal complexity.
Users work through a single entry point, `Simulation`, which exposes all physics subsystems.
Internally, `core::Stark` orchestrates time stepping and Newton solves via SymX.

## Component Map

```{mermaid}
flowchart TD
    Sim["Simulation"]

    Def["deformables"]
    RB["rigidbodies"]
    Int["interactions"]
    Pre["presets"]

    Core["Stark"]
    SymX["SymX"]

    Sim --> Def
    Sim --> RB
    Sim --> Int
    Sim --> Pre
    Sim --> Core
    Def & RB & Int -.->|register| Core
    Core --> SymX
```


## `Simulation`

`Simulation` is the single public entry point.
Constructing it with a `Settings` object is all that is needed to start a simulation:

```cpp
stark::core::Settings settings;
settings.output.directory = "./output";
settings.simulation.max_time_step_size = 0.01;

stark::Simulation simulation(settings);
```

The four subsystems are available as public `shared_ptr` fields:

```cpp
simulation.deformables   // deformable meshes and FEM energies
simulation.rigidbodies   // rigid bodies and joints
simulation.interactions  // cross-system contact and attachments
simulation.presets       // high-level scene factories
```

Once the scene is built, the simulation is advanced by calling `run()`:

```cpp
simulation.run();                  // run until no time events remain
simulation.run(5.0);               // run for exactly 5 seconds
simulation.run_one_time_step();    // advance by a single time step
```

Time-dependent boundary conditions and scripted motions are registered through `simulation.get_script()` or `simulation.add_time_event()`.
See [Simulation Loop](simulation_loop.md) for the full API.

---

## Physics Subsystems

### `deformables`

Manages all deformable objects: rods, shells (or cloth), and volumetric soft bodies.

Internally, all deformable node positions and velocities are stored in a single flat array inside `PointDynamics`.
When you add a mesh, its nodes are appended to that shared array and identified by a `PointSetHandler`.
This flat representation keeps internals simple and improves parallelism and vectorization.

The energy models that act on point sets are:

| Model | Description |
|---|---|
| `EnergyLumpedInertia` | Mass and Rayleigh damping for any topology |
| `EnergyPrescribedPositions` | Penalty-based kinematic boundary conditions |
| `EnergySegmentStrain` | 1D stretching for rods and cables |
| `EnergyTriangleStrain` | 2D membrane strain for cloth and shells |
| `EnergyDiscreteShells` | Bending stiffness (Bergou discrete shells) |
| `EnergyTetStrain` | 3D volumetric FEM for soft bodies |

In practice you almost never compose these by hand.
The [Presets](presets.md) subsystem wraps the most common combinations into single calls.
The [Deformables](deformables.md) page covers the full API for when you need direct control.

### `rigidbodies`

Manages rigid bodies and the constraints between them.

Like deformables, all rigid-body state lives in a single flat array inside `RigidBodyDynamics`.
Each body has six DOFs (3 translational, 3 rotational) and is identified by a `RigidBodyHandler`.

The energy models are:

| Model | Description |
|---|---|
| `EnergyRigidBodyInertia` | Mass, inertia tensor, and Rayleigh damping |
| `EnergyRigidBodyConstraints` | All joints, motors, and springs between bodies |

See [Rigid Bodies](rigidbodies.md) and [Rigid Body Constraints](rb_constraints.md) for the full API.

### `interactions`

Manages energies that couple the two state systems — or connect objects of the same system across separate point sets.

| Model | Description |
|---|---|
| `EnergyFrictionalContact` | IPC-based frictional contact (deformable–deformable, rigid–deformable, rigid–rigid) |
| `EnergyAttachments` | Penalty-based gluing of surface or point pairs |

See [Contact](contact.md) and [Attachments](attachments.md).

### `presets`

High-level factory calls that compose inertia, strain, bending, and output registration into single calls.
Most users should start here.
See [Presets](presets.md) for the full list.

---

## `core::Stark`: The Solver Engine

`core::Stark` is the engine that drives the simulation.

Its key components are:

- **`GlobalPotential`** — the SymX energy registry. Energy models call `global_potential->add_potential(...)` during construction to register their symbolic expression and the state variables it acts on.
- **`Callbacks`** — hook points for injecting logic at fixed moments in the loop: `before_time_step`, `on_time_step_accepted`, `after_time_step`, and `write_frame`.
- **`EventDrivenScript`** — fires lambdas at specified simulation times or on solver events.
- **`Settings`** — output paths, time step size, Newton tolerances, contact parameters, and more.

Under normal use you do not interact with `core::Stark` directly.
When writing a custom energy model that extends STARK, you access it through `simulation.get_stark()`.
See [Extending STARK](extending.md) for a worked example.

