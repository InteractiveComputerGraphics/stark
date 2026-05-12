# Rigid Body Constraints

Constraints are the mechanism for joining rigid bodies together (joints), fixing them in space, driving them with prescribed motion (motors), and applying spring forces between them.

All constraints are added through `simulation.rigidbodies->add_constraint_*()`.
Each method returns a typed handler that you can use to query or update the constraint at run time.

## Fixing a Body in Space

The most common constraint: hold a body at its current position and orientation.

```cpp
auto fix = simulation.rigidbodies->add_constraint_fix(rb);

// Script: kinematic motion
simulation.add_time_event(0.0, duration, [&](double t) {
    fix.set_transformation(
        Eigen::Vector3d(0, 0, 0),   // translation (world)
        t * 90.0,                    // rotation angle (deg)
        Eigen::Vector3d(0, 0, 1)    // rotation axis
    );
});
```

```python
fix = simulation.rigidbodies().add_constraint_fix(rb)
simulation.add_time_event(0, duration,
    lambda t: fix.set_transformation(np.zeros(3), t*90.0, np.array([0,0,1])))
```

## Attaching Two Bodies

Rigidly attaches two bodies at their current relative pose (zero DOF joint):

```cpp
auto att = simulation.rigidbodies->add_constraint_attachment(body_a, body_b);
```

## Point Constraint

Constrains a specific world-space point to remain coincident between two bodies:

```cpp
auto h = simulation.rigidbodies->add_constraint_point(body_a, body_b, world_point);
```

## Hinge Joint

One rotational DOF around a shared axis.
Optionally with an angular range limit:

```cpp
auto hinge = simulation.rigidbodies->add_constraint_hinge(
    body_a, body_b,
    pivot_point,   // world-space pivot
    hinge_axis     // world-space axis
);

auto hinge_lim = simulation.rigidbodies->add_constraint_hinge_with_angle_limit(
    body_a, body_b, pivot_point, hinge_axis,
    admissible_angle_deg   // symmetric ±angle limit
);
```

## Slider (Prismatic Joint)

One translational DOF along a shared axis:

```cpp
auto slider = simulation.rigidbodies->add_constraint_slider(
    body_a, body_b, pivot_point, slide_axis
);

auto pslider = simulation.rigidbodies->add_constraint_prismatic_slider(
    body_a, body_b, pivot_point, slide_axis
);
```

## Spring

A linear spring between two points on two bodies:

```cpp
auto spring = simulation.rigidbodies->add_constraint_spring(
    body_a, body_b,
    point_on_a_world,   // attachment point on body A (world coords at creation time)
    point_on_b_world,   // attachment point on body B
    stiffness,
    damping             // optional, default 0
);

// Spring with distance limits (acts only outside [min, max])
auto spring_lim = simulation.rigidbodies->add_constraint_spring_with_limits(
    body_a, body_b, p_a, p_b, stiffness, min_length, max_length, damping
);
```

## Motors and Velocity Constraints

### Angular Velocity Motor

Drives the relative angular velocity between two bodies around an axis:

```cpp
auto motor = simulation.rigidbodies->add_constraint_motor(
    body_a, body_b,
    pivot_point,
    rotation_axis,
    target_omega,      // target relative angular velocity (rad/s)
    max_torque,        // torque limit
    delay              // time constant for velocity ramp-up (default 0.01 s)
);
```

### Linear Velocity Actuator

Drives the relative linear velocity along an axis:

```cpp
auto linear_vel = simulation.rigidbodies->add_constraint_linear_velocity(
    body_a, body_b, axis, target_v, max_force, delay
);

auto press = simulation.rigidbodies->add_constraint_prismatic_press(
    body_a, body_b, pivot_point, axis, target_v, max_force, delay
);
```

## Direction Constraints

### Fixed Global Direction

Constrains a body-local direction to remain aligned with a global direction:

```cpp
auto global_dir = simulation.rigidbodies->add_constraint_global_direction(
    body, world_direction
);
```

### Direction Pair

Constrains a direction on body A to remain aligned with a direction on body B:

```cpp
auto dir = simulation.rigidbodies->add_constraint_direction(
    body_a, body_b, shared_direction_world
);
```

### Angular Limit

Restricts the angle between two body-local directions:

```cpp
auto ang_lim = simulation.rigidbodies->add_constraint_angle_limit(
    body_a, body_b, direction_world, admissible_angle_deg
);
```

## Global Point

Fixes a specific body-local point to remain at a fixed world-space location:

```cpp
auto gp = simulation.rigidbodies->add_constraint_global_point(body, world_point);
```

## Distance Constraints

Constrains the distance between two points on two bodies:

```cpp
auto dist = simulation.rigidbodies->add_constraint_distance(
    body_a, body_b, point_a_world, point_b_world
);

// Or with a range [min, max]
auto dist_lim = simulation.rigidbodies->add_constraint_distance_limits(
    body_a, body_b, point_a_world, point_b_world, min_distance, max_distance
);
```

## Point on Axis

Constrains a point on body A to lie on a line (axis) defined on body B:

```cpp
auto poa = simulation.rigidbodies->add_constraint_point_on_axis(
    body_a, body_b, point_world, axis_world
);
```

## Default Constraint Parameters

All constraints share stiffness and tolerance defaults that can be adjusted globally:

```cpp
simulation.rigidbodies->set_default_constraint_stiffness(1e6);
simulation.rigidbodies->set_default_constraint_distance_tolerance(0.001); // meters
simulation.rigidbodies->set_default_constraint_angle_tolerance(1.0);      // degrees
```

Per-constraint overrides are available through the returned handler (e.g. `h.set_stiffness(1e8)`).
