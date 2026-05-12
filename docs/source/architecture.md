# Architecture

Stark is organized as a layered platform.
The outermost layer is what users interact with; the innermost layer is SymX, which handles symbolic math, code generation, and the Newton solver.

## The Big Picture

```{mermaid}
flowchart BT

classDef user   fill:#dcfce7,stroke:#22c55e,stroke-width:1.5px,color:#064e3b
classDef model  fill:#dbeafe,stroke:#3b82f6,stroke-width:1.5px,color:#1e3a8a
classDef core   fill:#fef9c3,stroke:#eab308,stroke-width:1.5px,color:#713f12
classDef symx   fill:#ffe4e6,stroke:#fb7185,stroke-width:1.5px,color:#881337

SIM["Simulation"]:::user

DEF["Deformables"]:::model
RB["RigidBodies"]:::model
INT["Interactions"]:::model
PRE["Presets"]:::model

STARK["stark::core::Stark"]:::core

SYMX["SymX\n(GlobalPotential + NewtonsMethod)"]:::symx

SIM ==>|"owns"| DEF
SIM ==>|"owns"| RB
SIM ==>|"owns"| INT
SIM ==>|"owns"| PRE
SIM ==>|"owns"| STARK

DEF ==>|"registers potentials in"| STARK
RB  ==>|"registers potentials in"| STARK
INT ==>|"registers potentials in"| STARK

STARK ==>|"drives"| SYMX
```

## Layers

### Green · User-Facing API

`stark::Simulation` is the single entry point for users.
It owns four subsystems — `Deformables`, `RigidBodies`, `Interactions`, and `Presets` — as public fields (C++) or accessor methods (Python).
You construct a `Simulation`, add objects, set up scripting, and call `run()`.

### Blue · Physics Models

Each subsystem manages a family of energy models.
When you add a cloth, a rigid body, a contact pair, or an attachment, the subsystem registers the corresponding energy potentials with the core, connects them to the degree-of-freedom (DoF) arrays, and sets up any required callbacks.

Models exposed to users:

| Subsystem | Models |
|---|---|
| `Deformables` | Lumped inertia, prescribed positions, segment strain (rods), triangle strain (cloth), discrete shells (bending), tet strain (volumes) |
| `RigidBodies` | Rigid body inertia, rigid body constraints (joints, motors, springs) |
| `Interactions` | IPC frictional contact, attachments (deformable–deformable, rigid–deformable) |
| `Presets` | High-level constructors for common objects (grid cloth, tet box, sphere, cylinder, …) |

### Yellow · Core Engine

`stark::core::Stark` owns:
- The SymX `GlobalPotential` (all registered energy potentials)
- The SymX `Context` (thread count, logger, output verbosity)
- The simulation loop (`run()`, `run_one_step()`)
- The `EventDrivenScript` and `Callbacks` systems

### Red · SymX

SymX handles everything below the waterline:
- Symbolic differentiation of all energy potentials
- JIT C++ code generation and compilation (once, then cached)
- Newton's Method with adaptive line search and Hessian projection

You do not need to interact with SymX directly to use Stark.
If you want to add new energy models, you will use the SymX API to define potentials — see [Extending Stark](extending.md) and the [SymX documentation](https://github.com/InteractiveComputerGraphics/SymX).

## Data Flow Through a Time Step

Each simulated time step follows this sequence:

```{mermaid}
flowchart LR

classDef cb fill:#dbeafe,stroke:#3b82f6,color:#1e3a8a
classDef nw fill:#ffe4e6,stroke:#fb7185,color:#881337

A["before_time_step\ncallbacks"]:::cb
B["Newton's Method\n(SymX)"]:::nw
C["on_time_step_accepted\ncallbacks"]:::cb
D["after_time_step\ncallbacks"]:::cb
E["write_frame\ncallbacks"]:::cb

A --> B --> C --> D --> E
```

Callbacks run at well-defined points in the loop.
They are the mechanism by which models react to solver events (e.g. contact stiffness hardening on penetration) and by which user scripts update kinematic objects.

## Output

Stark writes per-frame mesh output as VTK (`.vtk`) files by default, one file per registered object per frame.
The frame rate is controlled by `settings.output.fps`.
A YAML log with timing and solver statistics is also written to disk.
