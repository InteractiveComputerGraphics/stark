# Deformables

This page covers the individual energy models available for deformable objects.
All deformable models operate on **point sets**.
A point set is an array of 3D positions and velocities managed by `PointDynamics`.
You get a `PointSetHandler` back when you register a point set, and that handle is what you pass to every energy model you want to attach to those points.

```cpp
auto ps = simulation.deformables->point_sets->add(vertices);
```

## Transforms

You can reposition and reorient the point set, or give it an initial velocity:

```cpp
ps.add_translation({0.0, 0.0, 1.0});
ps.add_rotation(90.0, {1.0, 0.0, 0.0});  // angle_deg, axis
ps.set_velocity({0.0, 0.0, -1.0});        // applied to all points
```

## Potential energies

### EnergyLumpedInertia

Mass and Rayleigh damping for a point set.
Works with any element topology: edges, triangles, or tetrahedra;
The mass is distributed (lumped) to the vertices proportional to element volume.

```cpp
auto inertia_h = simulation.deformables->lumped_inertia->add(
    ps, triangles,
    stark::EnergyLumpedInertia::Params()
        .set_density(0.2)   // [kg/m²] for surfaces, [kg/m³] for volumes, etc.
        .set_damping(0.5)
);
```

Every dynamic point set needs at least one inertia registration.
The `quasistatic` flag disables the kinetic energy term; see [Simulation Loop](simulation_loop.md) for quasistatic usage.

---

### EnergyPrescribedPositions

Penalty-based kinematic boundary conditions.
Selected vertices are attracted to a target configuration with a stiffness penalty.

```cpp
auto bc = simulation.deformables->prescribed_positions->add_inside_aabb(
    ps,
    Eigen::Vector3d(0, 0, 0),       // AABB centre
    Eigen::Vector3d(0.001, 1, 1),   // AABB half-extents
    stark::EnergyPrescribedPositions::Params()
        .set_stiffness(1e6)
);
```

You can also select by explicit vertex list (`add`) or invert the AABB selection (`add_outside_aabb`).

To animate the boundary condition, update its target transformation each time step:

```cpp
simulation.add_time_event(0.0, duration, [&](double t) {
    bc.set_transformation(
        Eigen::Vector3d(0, 0, 0),   // translation
        t * 90.0,                    // rotation angle (deg)
        Eigen::Vector3d(1, 0, 0)    // rotation axis
    );
});
```

---

### EnergySegmentStrain

1D axial strain energy for rods and cables.
Acts on pairs of connected vertices (segments).

```cpp
auto strain_h = simulation.deformables->segment_strain->add(
    ps, segments,
    stark::EnergySegmentStrain::Params()
        .set_section_radius(5e-3)       // rod cross-section radius
        .set_youngs_modulus(1e6)
        .set_damping(0.0)
);
```

Optional strain limiting caps elongation beyond a given threshold, preventing explosive stretching.
`elasticity_only = true` disables strain limiting and deformation damping for simpler more performant solve if needed.

---

### EnergyTriangleStrain

2D membrane strain energy for cloth and thin shells.
Acts on triangles and models in-plane stretching and compression using the Neo-Hookean constitutive model.

```cpp
auto strain_h = simulation.deformables->triangle_strain->add(
    ps, triangles,
    stark::EnergyTriangleStrain::Params()
        .set_thickness(0.001)           // shell thickness (m)
        .set_youngs_modulus(1e5)
        .set_poissons_ratio(0.3)
        .set_strain_limit(0.2)          // optional; 0 = disabled
);
```

An `inflation` parameter adds a pressure-like outward force, useful for inflatable objects.
`elasticity_only = true` disables strain limiting and deformation damping for simpler more performant solve if needed.

---

### EnergyDiscreteShells

Bending energy for triangle meshes, based on the discrete shells formulation (Grinspun et al.) or the flat rest shape quadratic bending model (Bergou et al.).
Acts on hinge edges (pairs of adjacent triangles).

```cpp
auto bending_h = simulation.deformables->discrete_shells->add(
    ps, triangles,
    stark::EnergyDiscreteShells::Params()
        .set_stiffness(1e-3)
        .set_flat_rest_angle(true)   // true → Bergou quadratic; false → use mesh's rest angle
);
```

This is always paired with `EnergyTriangleStrain` on the same triangle mesh.
The `Surface` preset does this automatically.

---

### EnergyTetStrain

3D volumetric FEM strain energy for soft bodies.
Acts on linear tetrahedra using the Neo-Hookean constitutive model.

```cpp
auto strain_h = simulation.deformables->tet_strain->add(
    ps, tets,
    stark::EnergyTetStrain::Params()
        .set_youngs_modulus(1e5)
        .set_poissons_ratio(0.4)
);
```

`elasticity_only = true` disables strain limiting and deformation damping for simpler more performant solve if needed.


---

## Manual Composition

Any combination of the above energies can be registered on the same `PointSetHandler`.
This is how the `hanging_box_with_composite_material` [TODO: link] example builds a single mesh with volumetric interior, shell surface, and rod edges — each energy registers independently and they all contribute to the same Newton solve.
See `examples/main.cpp` for the full code.

## Output

Mesh output is registered separately:

```cpp
simulation.deformables->output->add_triangle_mesh("cloth", ps, triangles);
simulation.deformables->output->add_tet_mesh("body", ps, tets);
simulation.deformables->output->add_segment_mesh("rod", ps, segments);
simulation.deformables->output->add_point_set("pts", ps);
```

Presets handle this automatically.


