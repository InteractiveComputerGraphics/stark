# Rigid Bodies

STARK's rigid body system handles articulated mechanisms, kinematic objects, and rigid-deformable interaction.
The main entry point is `simulation.rigidbodies`.

## Adding a Rigid Body

The low-level interface takes mass and an inertia tensor in the local frame:

```cpp
// C++
Eigen::Matrix3d I = stark::inertia_tensor_box(mass, {0.1, 0.1, 0.2});
stark::RigidBodyHandler rb = simulation.rigidbodies->add(mass, I);
```

For common shapes use the [Presets](presets.md) — they compute the inertia tensor automatically:

```cpp
auto vch = simulation.presets->rigidbodies->add_box("box", mass, Eigen::Vector3d(0.1, 0.1, 0.2));
stark::RigidBodyHandler& rb = vch.handler.rigidbody;
```

```python
vV, vC, vH = simulation.presets().rigidbodies().add_box("box", mass, np.array([0.1, 0.1, 0.2]))
rb = vH.rigidbody
```

## RigidBodyHandler

`RigidBodyHandler` is the primary handle for controlling a single rigid body.

### Transform

```cpp
// Set/query position
Eigen::Vector3d pos = rb.get_translation();
rb.set_translation({0.0, 0.0, 1.0});
rb.add_translation({0.0, 0.0, 0.5});

// Set/query orientation (quaternion or angle-axis)
Eigen::Quaterniond q = rb.get_quaternion();
rb.set_rotation(45.0, {0.0, 0.0, 1.0});     // angle_deg, axis
rb.add_rotation(10.0, {0.0, 1.0, 0.0}, pivot); // angle_deg, axis, pivot_point
```

### Velocity

```cpp
Eigen::Vector3d v = rb.get_velocity();
rb.set_velocity({1.0, 0.0, 0.0});
rb.add_velocity({0.0, 0.5, 0.0});

Eigen::Vector3d omega = rb.get_angular_velocity();
rb.set_angular_velocity({0.0, 0.0, 3.14});
rb.add_angular_velocity({0.0, 0.0, 0.5});

// Velocity at a specific world-space point
Eigen::Vector3d v_at = rb.get_velocity_at(world_point);
```

### Forces and Torques

Forces and torques accumulate across calls and are reset each time step.

```cpp
// Force applied at the centroid
rb.set_force_at_centroid({0.0, 0.0, -9.81 * mass});  // sets (overwrites)
rb.add_force_at_centroid({0.0, 0.0, -1.0});           // accumulates

// Force applied at a world-space point (induces torque)
rb.set_force_at(force_world, application_point_world);  // sets force + induced torque
rb.add_force_at(force_world, application_point_world);  // accumulates

// Torque directly
Eigen::Vector3d tau = rb.get_torque();
rb.set_torque({0.0, 0.0, 1.0});
rb.add_torque({0.0, 0.0, 0.5});
```

### Accelerations

Accelerations are an alternative to forces when you want to control the body in acceleration space (independent of mass):

```cpp
rb.set_acceleration({0.0, 0.0, -9.81});
rb.set_angular_acceleration({0.0, 0.0, 1.0});
```

### Coordinate Transforms

```cpp
Eigen::Vector3d x_world = rb.transform_local_to_global_point({0.0, 0.0, 0.5});
Eigen::Vector3d d_world = rb.transform_local_to_global_direction({1.0, 0.0, 0.0});
Eigen::Matrix3d R       = rb.get_rotation_matrix();
```

## Inertia Tensors

STARK provides helper functions to compute inertia tensors for common shapes:

```cpp
Eigen::Matrix3d I_box      = stark::inertia_tensor_box(mass, size_vec3);
Eigen::Matrix3d I_sphere   = stark::inertia_tensor_sphere(mass, radius);
Eigen::Matrix3d I_cylinder = stark::inertia_tensor_cylinder(mass, radius, height);
```

These are automatically used by the preset constructors.

## Mesh Output

Rigid bodies in STARK are output as triangle meshes.
You register collision/render meshes through the presets or directly:

```cpp
// Via presets (recommended)
auto vch = simulation.presets->rigidbodies->add_box("box", mass, size);
// vch.vertices and vch.triangles hold the mesh

// Manual registration for custom shapes
simulation.presets->rigidbodies->add("my_body", mass, I,
    collision_vertices, collision_triangles,
    render_vertices, render_triangles);
```

The output mesh file name is determined by `output_label` and the current frame.

## Gravity on Rigid Bodies

Rigid bodies automatically respond to `simulation.gravity`.
You can override per-body by applying a custom acceleration:

```cpp
rb.set_acceleration(simulation.get_gravity() * custom_factor);
```

Or disable gravity response by setting zero acceleration (gravity is added via inertia energy; disabling that is a model-level concern — see [Extending STARK](extending.md)).
