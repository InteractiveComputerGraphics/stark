# Rigid Body Constraints

Rigid body constraints are the mechanism used to strict rigid body motion, attach bodies to each other, build joints, add springs, and drive motion with motors.

All constraints are added through `simulation.rigidbodies->add_constraint_*()` and each call returns a handle. The handle can be used to change parameters, enable/disable the constraint, assign labels, or query the current violation.

```cpp
// Global defaults used by subsequently created hard constraints.
simulation.rigidbodies->set_default_constraint_stiffness(1e6);
simulation.rigidbodies->set_default_constraint_distance_tolerance(1e-3); // m
simulation.rigidbodies->set_default_constraint_angle_tolerance(1.0);     // deg

// Example: a hinge joint between two rigid bodies.
auto hinge = simulation.rigidbodies->add_constraint_hinge(
    body_a,
    body_b,
    pivot_point,   // world-space point at creation time
    hinge_axis     // world-space direction at creation time
);

hinge.set_stiffness(1e8);
hinge.set_tolerance_in_m(1e-4);
hinge.set_tolerance_in_deg(0.1);
hinge.set_label("main_hinge");
```

## How constraints work

STARK rigid body constraints are implemented as **energy terms** in the global optimization problem.
Most rigid body constraints are defined from points or directions given in world coordinates at creation time.

Hard constraints use finite stiffness and a tolerance. At the end of Newton convergence, STARK checks whether the constraint violation is within tolerance. If not, the constraint stiffness is hardened and the solve continues. This makes constraints robust without requiring the user to manually guess extremely large stiffness values from the beginning.

There are two important categories:

- **Primitive constraints** are the actual energy terms registered in SymX.
- **Composed constraints** are convenience joints built by combining primitive constraints.


## Common handle operations

All constraint handles support basic operations such as:

```cpp
constraint.set_label("name"); // Used for traceability
constraint.enable(true);      // activate
constraint.enable(false);     // deactivate
```

Hard positional constraints usually provide:

```cpp
constraint.set_stiffness(1e8);
constraint.set_tolerance_in_m(1e-4);
```

Hard directional constraints usually provide:

```cpp
constraint.set_stiffness(1e8);
constraint.set_tolerance_in_deg(1.0);
```

Velocity controllers and springs expose their physical parameters directly, such as target velocity, maximum force, maximum torque, damping, or rest length.

## Primitive Constraints

Primitive constraints are the low-level constraint energies implemented by STARK. The higher-level joints later in this page are built from these pieces.

### Global Point

Fixes one body-local point to a target point in world space.

```cpp
auto c = simulation.rigidbodies->add_constraint_global_point(
    body,
    world_point
);
```

### Global Direction

Aligns one body-local direction with a target direction in world space.

```cpp
auto c = simulation.rigidbodies->add_constraint_global_direction(
    body,
    world_direction
);
```

### Point

Constrains one point on body A to coincide with one point on body B. This is the primitive ball-joint constraint.

```cpp
auto c = simulation.rigidbodies->add_constraint_point(
    body_a,
    body_b,
    world_point
);
```

### Point on Axis

Constrains a point on body B to lie on an axis attached to body A.

```cpp
auto c = simulation.rigidbodies->add_constraint_point_on_axis(
    body_a,
    body_b,
    world_point,
    world_axis
);
```


### Distance

Constrains the distance between two body-local points to remain equal to the distance at creation time.

```cpp
auto c = simulation.rigidbodies->add_constraint_distance(
    body_a,
    body_b,
    point_on_a_world,
    point_on_b_world
);
```

### Distance Limits

Constrains the distance between two body-local points to remain inside an interval.

```cpp
auto c = simulation.rigidbodies->add_constraint_distance_limits(
    body_a,
    body_b,
    point_on_a_world,
    point_on_b_world,
    min_distance,
    max_distance
);
```

### Direction

Aligns one body-local direction on body A with one body-local direction on body B.

```cpp
auto c = simulation.rigidbodies->add_constraint_direction(
    body_a,
    body_b,
    world_direction
);
```

### Angle Limit

Restricts the angle between two body-local directions.

```cpp
auto c = simulation.rigidbodies->add_constraint_angle_limit(
    body_a,
    body_b,
    world_direction,
    admissible_angle_deg
);
```

### Damped Spring

Adds a damped linear spring between two body-local points.

```cpp
auto spring = simulation.rigidbodies->add_constraint_spring(
    body_a,
    body_b,
    point_on_a_world,
    point_on_b_world,
    stiffness,
    damping        // optional, default 0.0
);
```

### Linear Velocity

Drives the relative linear velocity of body B with respect to body A along a direction attached to body A.

```cpp
auto c = simulation.rigidbodies->add_constraint_linear_velocity(
    body_a,
    body_b,
    world_direction,
    target_v,       // m/s
    max_abs_force,  // N
    delay           // optional, default 0.01 s
);
```

### Angular Velocity

Drives the relative angular velocity of body B with respect to body A around a direction attached to body A.

```cpp
auto c = simulation.rigidbodies->add_constraint_angular_velocity(
    body_a,
    body_b,
    world_axis,
    target_w,        // rad/s
    max_abs_torque,  // Nm
    delay            // optional, default 0.01 s
);
```


## Composed Constraints

Composed constraints are convenience APIs that combine primitive constraints. They return composed handles that forward stiffness, tolerance, label, and activation changes to the underlying primitives.

### Fix

Fixes a rigid body in world space by combining one global point constraint with two global direction constraints.

```cpp
auto fix = simulation.rigidbodies->add_constraint_fix(body);
```

The body is fixed at its current translation and orientation at creation time. This is also the usual way to create kinematically scripted rigid bodies: create a fix constraint, then update its target transformation during the simulation.

```cpp
auto fix = simulation.rigidbodies->add_constraint_fix(body);

simulation.add_time_event(0.0, duration, [&](double t) {
    fix.set_transformation(
        Eigen::Vector3d(0.0, 0.0, 0.2 * t),  // target translation
        30.0 * t,                             // angle in degrees
        Eigen::Vector3d::UnitZ()              // rotation axis
    );
});
```

### Attachment

Rigidly attaches two bodies together, producing a zero-DOF joint.

```cpp
auto attachment = simulation.rigidbodies->add_constraint_attachment(
    body_a,
    body_b
);
```

The attachment is created at the midpoint between the current body translations and locks the relative orientation with two direction constraints.


### Point with Angle Limit

Combines a point constraint with an angular range limit.

```cpp
auto joint = simulation.rigidbodies->add_constraint_point_with_angle_limit(
    body_a,
    body_b,
    pivot_point,
    limit_axis,
    admissible_angle_deg
);
```

Conceptually, this behaves like a ball joint with an angular cone limit.

### Hinge

Creates a one-DOF hinge joint: the bodies share a pivot point and may rotate relative to each other around the hinge axis.

```cpp
auto hinge = simulation.rigidbodies->add_constraint_hinge(
    body_a,
    body_b,
    pivot_point,
    hinge_axis
);
```


### Hinge with Angle Limit

Creates a hinge joint with a symmetric angular limit around the hinge axis.

```cpp
auto hinge = simulation.rigidbodies->add_constraint_hinge_with_angle_limit(
    body_a,
    body_b,
    pivot_point,
    hinge_axis,
    admissible_angle_deg
);
```


### Spring with Limits

Adds a damped spring and additionally constrains its length to remain inside a prescribed interval.

```cpp
auto spring = simulation.rigidbodies->add_constraint_spring_with_limits(
    body_a,
    body_b,
    point_on_a_world,
    point_on_b_world,
    spring_stiffness,
    min_length,
    max_length,
    damping             // optional, default 0.0
);
```

The spring stiffness and the hard distance-limit stiffness are independent. The composed handle exposes `set_spring_stiffness()` for the spring and `set_stiffness()` for the distance limits.

### Slider

Creates a slider-like joint where a point on one body can move along an axis attached to the other body.

```cpp
auto slider = simulation.rigidbodies->add_constraint_slider(
    body_a,
    body_b,
    pivot_point,
    slide_axis
);
```

This permits sliding along the axis while keeping the corresponding body directions aligned.

### Prismatic Slider

Creates a stricter prismatic joint by adding an additional orthogonal direction lock to the slider.

```cpp
auto slider = simulation.rigidbodies->add_constraint_prismatic_slider(
    body_a,
    body_b,
    pivot_point,
    slide_axis
);
```

This keeps the bodies aligned as a prismatic pair while allowing translation along the slide axis.

### Prismatic Press

Creates a prismatic slider driven by a force-limited linear velocity controller.

```cpp
auto press = simulation.rigidbodies->add_constraint_prismatic_press(
    body_a,
    body_b,
    pivot_point,
    slide_axis,
    target_v,
    max_force,
    delay       // optional, default 0.01 s
);
```

This is useful for grippers, presses, pistons, and other constrained linear actuators.

### Motor

Creates a hinge driven by a torque-limited angular velocity controller.

```cpp
auto motor = simulation.rigidbodies->add_constraint_motor(
    body_a,
    body_b,
    pivot_point,
    rotation_axis,
    target_w,
    max_torque,
    delay       // optional, default 0.01 s
);
```

This is the standard rigid body rotational motor.

