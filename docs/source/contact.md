# Frictional Contact

STARK uses an **IPC (Incremental Potential Contact)** based formulation for frictional contact.
It is guaranteed to be intersection-free and handles all contact pairs (deformable–deformable, rigid–deformable, rigid–rigid) in a unified framework.

Contact is handled by `simulation.interactions->contact` (C++) or `simulation.interactions().contact()` (Python).

## Global Parameters

Set once before the simulation starts:

```cpp
// C++
stark::EnergyFrictionalContact::GlobalParams params;

// Contact thickness (d_hat): the distance at which contact forces activate
// MUST be set; no sensible default exists — it depends on your scene scale
params.default_contact_thickness = 0.001;  // 1 mm

// Stiffness range (automatically hardened when penetration is detected)
params.min_contact_stiffness = 1e4;
params.max_contact_stiffness = 1e20;

// Friction: stick-to-slide threshold (relative tangential displacement)
params.friction_stick_slide_threshold = 0.01;

// Enable/disable specific contact modes
params.collisions_enabled        = true;
params.friction_enabled          = true;
params.triangle_point_enabled    = true;
params.edge_edge_enabled         = true;
params.intersection_test_enabled = true;

simulation.interactions->contact->set_global_params(params);
```

```python
# Python
params = pystark.EnergyFrictionalContact.GlobalParams()
params.default_contact_thickness       = 0.001
params.min_contact_stiffness           = 1e4
params.friction_stick_slide_threshold  = 0.01
simulation.interactions().contact().set_global_params(params)
```

## Registering Contact Objects

Every object that should participate in contact needs a `ContactHandler`.
Presets return it automatically in `vch.handler.contact`.
For manual meshes, you register the mesh explicitly:

```cpp
// Via presets (recommended)
auto cloth = simulation.presets->deformables->add_surface_grid("cloth", ...);
ContactHandler& ch_cloth = cloth.handler.contact;

auto box = simulation.presets->rigidbodies->add_box("box", ...);
ContactHandler& ch_box = box.handler.contact;
```

## Setting Friction Between Pairs

Friction must be explicitly enabled between each pair of contact objects.
The Coulomb friction coefficient $\mu$ is set per pair:

```cpp
// C++
ch_cloth.set_friction(ch_box, 0.5);   // μ = 0.5 between cloth and box
ch_cloth.set_friction(ch_cloth, 0.1); // self-contact with μ = 0.1
```

```python
# Python
cH.contact.set_friction(bH.contact, 0.5)
cH.contact.set_friction(cH.contact, 0.1)
```

Pairs without an explicit `set_friction` call have no contact forces between them.

## Per-Object Contact Thickness

You can override the global default thickness per object:

```cpp
ch_cloth.set_contact_thickness(0.0005);  // 0.5 mm
```

## Disabling Collisions Between Pairs

To exclude specific pairs from collision detection entirely (e.g. objects that start overlapping):

```cpp
ch_a.disable_collision(ch_b);
```

## Stiffness Hardening

When the Newton solver converges to a state where penetration is detected by the intersection test, the contact stiffness is automatically doubled and the time step is re-attempted with the new stiffness.
This process repeats until either penetration is resolved or `max_contact_stiffness` is reached.
You will see messages like:

```
Penetration couldn't be avoided. Contact stiffness hardened from 1.0e+04 to 2.0e+04.
```

This is normal behavior for heavily contacted scenes.

