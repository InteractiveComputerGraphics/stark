# Deformables

Stark's deformable simulation covers three geometric primitives — **lines** (rods), **surfaces** (cloth/shells), and **volumes** (tet meshes) — plus a **prescribed positions** mechanism for kinematic control.
Each primitive bundles inertia, strain, and (optionally) bending energies into a single parameter struct and handler.

The lowest-level access is through `simulation.deformables`, which exposes individual energy systems.
For common cases, use the [Presets](presets.md) layer instead — it wires everything up automatically.

## Point Sets

Every deformable object is backed by a **point set**: an array of 3D positions that are the degrees of freedom solved by Newton's Method.
`PointSetHandler` is the handle returned to represent a specific point set.

```cpp
auto& ps = cloth.handler.point_set;

// Transform the rest state before the simulation starts
ps.add_translation({0.0, 0.0, 1.0});
ps.add_rotation(90.0, {1.0, 0.0, 0.0});   // angle_deg, axis

// Set initial velocity
ps.set_velocity({0.0, 0.0, -1.0});
```

```python
ps = cloth.handler.point_set
ps.add_translation(np.array([0.0, 0.0, 1.0]))
ps.add_rotation(90.0, np.array([1.0, 0.0, 0.0]))
ps.set_velocity(np.array([0.0, 0.0, -1.0]))
```

## Lines (Rods)

1D elastic rods modeled with a linear segment strain energy and lumped inertia.
Useful for cables, ropes, and spring elements.

### Parameters

```cpp
stark::Line::Params params;

// Inertia
params.inertia.density = 1000.0;    // kg/m³ — used to compute lumped mass per vertex
params.inertia.damping = 0.0;       // Rayleigh mass damping coefficient

// Strain
params.strain.youngs_modulus = 1e6; // Pa
params.strain.poissons_ratio = 0.4;
params.strain.damping        = 0.0;
params.strain.cross_section_area = 1e-4; // m²
```

Or use a built-in preset:

```cpp
auto params = stark::Line::Params::Elastic_Rubberband();
```

### Adding a Line

```cpp
// From existing geometry
auto h = simulation.presets->deformables->add_line("rod", vertices, segments, params);

// Convenience: uniform segments along a straight line
auto vch = simulation.presets->deformables->add_line_as_segments("rod",
    Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), 20, params);
// vch.vertices, vch.segments, vch.handler
```

## Surfaces (Cloth / Shells)

2D elastic surfaces with in-plane strain (membrane) and optional bending (discrete shells).
The typical use case is cloth simulation.

<p align="center">
    <img src="twisting_cloth.gif" alt="Twisting cloth simulation" style="width:55%;">
</p>

### Parameters

```cpp
stark::Surface::Params params;

// Inertia
params.inertia.density = 200.0;  // kg/m³ (effective surface density when × thickness)
params.inertia.damping = 0.5;

// In-plane strain
params.strain.youngs_modulus       = 1e5;
params.strain.poissons_ratio       = 0.3;
params.strain.damping              = 0.0;
params.strain.strain_limit         = 0.2;           // max strain before stiffening kicks in
params.strain.strain_limit_stiffness = 1e8;

// Bending (discrete shells)
params.bending.stiffness = 1e-3;
params.bending.damping   = 0.0;

// Contact thickness (overrides global default)
params.contact.thickness = 0.001;
```

Or use a built-in preset:

```cpp
auto params = stark::Surface::Params::Cotton_Fabric();
```

### Adding a Surface

```cpp
// From existing triangle mesh
auto h = simulation.presets->deformables->add_surface("cloth", vertices, triangles, params);

// Convenience: uniform grid (planar, axis-aligned)
auto vch = simulation.presets->deformables->add_surface_grid("cloth",
    Eigen::Vector2d(0.5, 0.5),  // physical size (m)
    {30, 30},                    // subdivisions
    params);
// vch.vertices, vch.triangles, vch.handler
```

### Handler Fields

```cpp
auto& h = vch.handler;
h.point_set   // PointSetHandler — positions / velocity / transforms
h.inertia     // EnergyLumpedInertia::Handler
h.strain      // EnergyTriangleStrain::Handler
h.bending     // EnergyDiscreteShells::Handler
h.contact     // ContactHandler — for setting friction pairs
```

## Volumes (Tet Meshes)

3D volumetric FEM with tetrahedral elements.
Suitable for soft bodies, deformable objects with interior deformation, and viscoelastic materials.

<p align="center">
    <img src="viscoelasticity.gif" alt="Volumetric deformable" style="width:55%;">
</p>

### Parameters

```cpp
stark::Volume::Params params;

params.inertia.density = 1000.0;  // kg/m³
params.inertia.damping = 1.0;

params.strain.youngs_modulus = 1e5;
params.strain.poissons_ratio = 0.4;
params.strain.damping        = 0.0;

params.contact.thickness = 0.001;
```

### Adding a Volume

```cpp
// From existing tet mesh (vertices + tetrahedra)
auto h = simulation.presets->deformables->add_volume("body", vertices, tets, params);

// Convenience: uniform tet grid (axis-aligned box)
auto vch = simulation.presets->deformables->add_volume_grid("body",
    Eigen::Vector3d(0.1, 0.1, 0.2),  // physical size (m)
    {3, 3, 6},                         // subdivisions
    params);
```

## Prescribed Positions

Prescribed positions are the mechanism for kinematically driving deformable vertices.
They attach an elastic penalty that pulls selected vertices to a target configuration.

### Selecting Vertices

By AABB:

```cpp
auto bc = simulation.deformables->prescribed_positions->add_inside_aabb(
    point_set_handler,
    Eigen::Vector3d(0, 0, 0),    // AABB center
    Eigen::Vector3d(0.001, 1, 1), // AABB half-extents (very thin along X → selects left edge)
    stark::EnergyPrescribedPositions::Params()
);
```

### Scripting

```cpp
// Set target transformation every time step
simulation.add_time_event(0.0, duration, [&](double t) {
    bc.set_transformation(
        Eigen::Vector3d(0, 0, 0),    // translation offset
        t * 90.0,                     // rotation angle (deg)
        Eigen::Vector3d(1, 0, 0)     // rotation axis
    );
});
```

The stiffness of the penalty is set in `EnergyPrescribedPositions::Params`.
High stiffness = near-kinematic constraint; lower stiffness = soft attraction.
