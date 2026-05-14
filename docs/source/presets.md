# Presets

Presets are the most convenient way to declare a simulation.
They combine state and energy registration, and optionally geometry generation in a single call.
Most of the examples bundled with STARK use presets.

Access them via `simulation.presets->deformables` and `simulation.presets->rigidbodies` (C++), or `simulation.presets().deformables()` / `simulation.presets().rigidbodies()` (Python).

## Handlers

Every preset returns a **VCH** (Vertex-Connectivity-Handler) struct with three fields:

- **`vertices`** — the generated vertex positions
- **`triangles` / `segments` / `tets`** — the generated connectivity (type depends on preset)
- **`handler`** — sub-handlers for each registered energy model, plus a `PointSetHandler` for transforms and a `ContactHandler` for friction


## Deformable Presets

All deformable presets register: **lumped inertia** + the appropriate **deformation energy** + **contact**.

### Volume (Soft Body)

A tet mesh filling an axis-aligned box, or any custom tet mesh.
Registered energies: `EnergyLumpedInertia` + `EnergyTetStrain`.

```cpp
// Uniform tet grid
auto [V, T, H] = simulation.presets->deformables->add_volume_grid(
    "body",
    Eigen::Vector3d(0.1, 0.1, 0.2),  // physical size (m)
    {3, 3, 6},                         // subdivisions per axis
    stark::Volume::Params::Soft_Rubber()
);

// Custom tet mesh (e.g. from TetGen / Gmsh)
auto [V, T, H] = simulation.presets->deformables->add_volume("body", vertices, tets, params);
```

`H` is a `Volume::Handler` with fields: `point_set`, `inertia`, `strain`, `contact`.

**Material parameters:**
```cpp
stark::Volume::Params params = stark::Volume::Params::Soft_Rubber();

params.inertia.density                  = 1000.0;  // kg/m³
params.inertia.damping                  = 0.1;

params.strain.elasticity_only           = false;
params.strain.youngs_modulus            = 1e4;
params.strain.poissons_ratio            = 0.3;
params.strain.strain_limit              = 1.0;
params.strain.strain_limit_stiffness    = 1e2;
params.strain.damping                   = 0.0;

params.contact.thickness                = 0.001;
```

### Surface (Cloth / Shell)

A triangle mesh, flat grid or custom, with in-plane strain and optional bending.
Registered energies: `EnergyLumpedInertia` + `EnergyTriangleStrain` + `EnergyDiscreteShells`.

```cpp
// Flat grid
auto [V, T, H] = simulation.presets->deformables->add_surface_grid(
    "cloth",
    Eigen::Vector2d(0.5, 0.5),         // physical size in X and Y
    {30, 30},                           // vertex count per axis
    stark::Surface::Params::Cotton_Fabric()
);

// Custom triangle mesh
auto [V, T, H] = simulation.presets->deformables->add_surface("cloth", vertices, triangles, params);
```

`H` is a `Surface::Handler` with fields: `point_set`, `inertia`, `strain`, `bending`, `contact`.

**Material parameters:**
```cpp
stark::Surface::Params params = stark::Surface::Params::Cotton_Fabric();

params.inertia.density                  = 0.2;     // kg/m²
params.inertia.damping                  = 0.1;

params.strain.elasticity_only           = false;
params.strain.thickness                 = 0.001;   // m
params.strain.youngs_modulus            = 5e3;
params.strain.poissons_ratio            = 0.3;
params.strain.strain_limit              = 0.1;
params.strain.strain_limit_stiffness    = 1e6;
params.strain.damping                   = 0.1;

params.bending.flat_rest_angle          = true;
params.bending.stiffness                = 1e-6;
params.bending.damping                  = 0.01;

params.contact.thickness                = 0.001;
```

### Line (Rod / Cable)

A 1D chain of segments.
Registered energies: `EnergyLumpedInertia` + `EnergySegmentStrain`.

```cpp
// Straight rod with uniform segments
auto [V, S, H] = simulation.presets->deformables->add_line_as_segments(
    "rod",
    Eigen::Vector3d(0, 0, 0),   // start
    Eigen::Vector3d(1, 0, 0),   // end
    20,                          // number of segments
    stark::Line::Params::Elastic_Rubberband()
);

// Arbitrary geometry
auto [V, S, H] = simulation.presets->deformables->add_line("rod", vertices, segments, params);
```

`H` is a `Line::Handler` with fields: `point_set`, `inertia`, `strain`, `contact`.

**Material parameters:**
```cpp
stark::Line::Params params = stark::Line::Params::Elastic_Rubberband();

params.inertia.density                  = 0.05;    // kg/m
params.inertia.damping                  = 0.1;

params.strain.elasticity_only           = false;
params.strain.section_radius            = 0.002;   // m
params.strain.youngs_modulus            = 1e4;
params.strain.strain_limit              = 0.1;
params.strain.strain_limit_stiffness    = 1e5;
params.strain.damping                   = 1e-4;

params.contact.thickness                = 0.001;
```



### Prescribed Surface

A surface with zero dynamic DOFs — vertices follow exactly the transformation you script.
Useful for kinematic obstacles and animated rigid-looking objects that need contact.
Registered energies: `EnergyPrescribedPositions` + contact.

```cpp
auto H = simulation.presets->deformables->add_prescribed_surface(
    "obstacle", vertices, triangles, stark::PrescribedSurface::Params()
);
```

## Rigid Body Presets

Rigid body presets register inertia and attach a collision/render mesh.
The inertia tensor is computed automatically from the chosen shape.

```cpp
// Box (uniform cube or non-uniform)
auto [V, T, H] = simulation.presets->rigidbodies->add_box("box", mass, 0.1);         // cube side
auto [V, T, H] = simulation.presets->rigidbodies->add_box("box", mass, size_vec3);    // non-uniform

// Sphere
auto [V, T, H] = simulation.presets->rigidbodies->add_sphere("ball", mass, radius);

// Cylinder
auto [V, T, H] = simulation.presets->rigidbodies->add_cylinder("cyl", mass, radius, height);

// Torus
auto [V, T, H] = simulation.presets->rigidbodies->add_torus("torus", mass, outer_r, inner_r);

// Custom mesh
auto H = simulation.presets->rigidbodies->add(
    "body", mass, inertia_tensor_local, vertices, triangles
);
```

`H.rigidbody` is the `RigidBodyHandler` for motion control (see [Rigid Bodies](rigidbodies.md)).
`H.contact` is the `ContactHandler` for friction pairing (see [Contact](contact.md)).

