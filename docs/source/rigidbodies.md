# Rigid Bodies

STARK's rigid body system handles articulated mechanisms, kinematic objects, and rigid-deformable interaction.
The main entry point is `simulation.rigidbodies`.

Rigid bodies are most conveniently created through [Presets](presets.md), which compute inertia tensors and attach a collision mesh automatically:

```cpp
auto [V, T, H] = simulation.presets->rigidbodies->add_box("box", mass, size);
RigidBodyHandler& rb = H.rigidbody;
```

For a custom mesh or manually computed inertia tensor you can call the low-level interface:

```cpp
Eigen::Matrix3d I = stark::inertia_tensor_box(mass, size);
RigidBodyHandler rb = simulation.rigidbodies->add(mass, I);
```

Helper functions cover common shapes: `inertia_tensor_box`, `inertia_tensor_sphere`, `inertia_tensor_cylinder`, `inertia_tensor_torus`.

---

## RigidBodyHandler

`RigidBodyHandler` is the primary handle for controlling a single rigid body before and during the simulation.

### Transforms (pre-simulation setup)

```cpp
rb.set_translation({0.0, 0.0, 1.0});
rb.add_translation({0.0, 0.0, 0.5});
rb.set_rotation(45.0, {0.0, 0.0, 1.0});   // angle_deg, axis
rb.add_rotation(10.0, {0.0, 1.0, 0.0});
```

### Velocities

```cpp
rb.set_velocity({1.0, 0.0, 0.0});
rb.set_angular_velocity({0.0, 0.0, 3.14});
```

### External forces and torques

Forces and torques accumulate across calls and are reset each time step.

```cpp
rb.add_force_at_centroid({0.0, 0.0, -9.81 * mass});
rb.add_force_at(force_world, application_point_world);  // also induces torque
rb.add_torque({0.0, 0.0, 1.0});
```

### Coordinate transforms

```cpp
Eigen::Vector3d x_world = rb.transform_local_to_global_point({0.0, 0.0, 0.5});
Eigen::Vector3d d_world = rb.transform_local_to_global_direction({1.0, 0.0, 0.0});
Eigen::Matrix3d R       = rb.get_rotation_matrix();
```

---

## Scripting Rigid Bodies

The most common pattern is to fix a body and drive it with a prescribed trajectory.
See [Rigid Body Constraints](rb_constraints.md) for `add_constraint_fix` and other joints.
A scripted box spinning over time looks like:

```cpp
auto fix = simulation.rigidbodies->add_constraint_fix(rb);
simulation.add_time_event(0.0, duration, [&](double t) {
    fix.set_transformation(
        Eigen::Vector3d(0, 0, 0),
        t * 90.0,
        Eigen::Vector3d(0, 0, 1)
    );
});
```

## Output

Rigid body mesh output is handled by the presets automatically.
For custom rigid bodies, register the mesh explicitly:

```cpp
simulation.presets->rigidbodies->add(
    "my_body", mass, I,
    collision_vertices, collision_triangles,
    render_vertices, render_triangles
);
```

## Gravity

Rigid bodies automatically respond to `simulation.gravity` (set in `GlobalParams`).

