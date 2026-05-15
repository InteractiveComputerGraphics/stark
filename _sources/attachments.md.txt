# Attachments

Attachments are penalty-based constraints that connect points on two deformable objects, or points on a rigid body to a deformable object.
They are the standard way to glue parts of the simulation together.

Attachments are managed by `simulation.interactions->attachments` (C++) or `simulation.interactions().attachments()` (Python).

## Deformable–Deformable

### Point to Point

Connects a list of vertices on one point set to the corresponding vertices on another:

```cpp
auto h = simulation.interactions->attachments->add(
    set_a_handler, set_b_handler,
    points_a,       // std::vector<int> — vertex indices in set A
    points_b,       // std::vector<int> — vertex indices in set B
    stark::EnergyAttachments::Params()
);
```

### Point to Edge (Barycentric)

Attaches vertices on set A to positions on edges of set B via barycentric coordinates:

```cpp
auto h = simulation.interactions->attachments->add(
    set_a, set_b,
    points_a,
    edges_b,          // std::vector<std::array<int,2>>
    bary_coords,      // std::vector<std::array<double,2>>
    params
);
```

### Point to Triangle (Barycentric)

Attaches vertices on set A to triangles of set B:

```cpp
auto h = simulation.interactions->attachments->add(
    set_a, set_b,
    points_a,
    triangles_b,       // std::vector<std::array<int,3>>
    bary_coords,       // std::vector<std::array<double,3>>
    params
);
```

### By Distance

Automatically finds and attaches all vertices within a given distance of a target mesh:

```cpp
auto multi_h = simulation.interactions->attachments->add_by_distance(
    set_a, set_b,
    points_a,        // candidate source vertices
    triangles_b,     // target triangle mesh
    distance,        // match within this distance
    params
);
```

Returns a `MultiHandler` that groups the individual attachment handlers.

## Rigid Body–Deformable

Attaches vertices on a deformable point set to local-frame points on a rigid body:

```cpp
auto h = simulation.interactions->attachments->add(
    rb_handler,
    set_handler,
    rb_points_local,   // std::vector<Eigen::Vector3d> — rigid body frame
    set_points,        // std::vector<int> — vertex indices in the point set
    params
);
```


## Parameters

```cpp
stark::EnergyAttachments::Params params;
params.stiffness  = 1e6;   // penalty stiffness (N/m)
params.tolerance  = 1e-3;  // distance below which the constraint is considered satisfied
```

High stiffness gives a near-rigid bond; lower stiffness gives a soft connection.
Stiffness hardens automatically when the tolerance is not met at the end of a time step.

