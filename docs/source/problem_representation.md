# Problem Representation

Every STARK simulation is built from two ingredients: **state** (inertial point and rigid body systems) and **energies** (the physics that act on that state).

## State

STARK maintains two global state systems:

- **Point sets**: `PointDynamics` holds all deformable objects.
When you add a cloth, a tet mesh, or a cable, its state is appended to one shared array of 3D positions and velocities.
This flat representation gives a clean internal structure and improves parallelization and vectorization.
Each object is identified by its `PointSetHandler`.
- **Rigid bodies**: `RigidBodyDynamics` holds one entry per rigid body, each representing a translation (3 DOFs) and an orientation quaternion mapped to an angular velocity (3 DOFs).
Like point sets, all rigid bodies live in a single shared array.
Each body is identified by its `RigidBodyHandler`.


## Energies

Mechanical effects and constraints are expressed as energy potentials that are functions of the current state.
SymX differentiates them automatically to produce gradient and Hessian contributions to the Newton solver.
Energies fall into three natural groups:

**Deformable energies** — act exclusively on point sets:

| Energy | Purpose |
|---|---|
| `EnergyLumpedInertia` | Mass and Rayleigh damping for any topology (edges, triangles, tets) |
| `EnergyPrescribedPositions` | Penalty-based kinematic boundary conditions |
| `EnergySegmentStrain` | 1D stretching for rods and cables |
| `EnergyTriangleStrain` | 2D membrane strain for cloth and shells |
| `EnergyDiscreteShells` | Bending stiffness (Bergou discrete shells) |
| `EnergyTetStrain` | 3D volumetric FEM for soft bodies |

**Rigid body energies** — act exclusively on rigid bodies:

| Energy | Purpose |
|---|---|
| `EnergyRigidBodyInertia` | Mass, inertia tensor, and Rayleigh damping |
| `EnergyRigidBodyConstraints` | All joints, motors, and springs between bodies |

**Coupling energies** — bridge the two state systems (or bodies of the same system):

| Energy | Purpose |
|---|---|
| `EnergyFrictionalContact` | IPC-based frictional contact (d–d, rb–d, rb–rb) |
| `EnergyAttachments` | Penalty-based gluing of surface/point pairs |

Modelling an object typically requires composing multiple energies (e.g. inertia + triangle strain + bending for cloth).
[Presets](presets.md) wrap the most common combinations into single calls.
