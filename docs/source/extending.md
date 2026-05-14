# Extending STARK

STARK's physics models are defined using SymX — the symbolic differentiation and JIT compilation engine bundled with STARK.
Adding a new material or interaction energy requires writing the potential in symbolic form; STARK handles differentiation, code generation, compilation, and assembly automatically.

> For the full SymX API reference, see the [SymX documentation](https://github.com/InteractiveComputerGraphics/SymX).

## The Core Idea

Every physics model in STARK is an **energy potential** — a scalar function of the degrees of freedom (DoFs).
STARK assembles all potentials, differentiates them symbolically to obtain the gradient and Hessian, and feeds them to Newton's method.

The DoFs for deformable objects are already registered by `PointDynamics` (velocities `v1`, positions `x0`/`x1`).
To add a custom energy you only need to:

1. Define a potential function in symbolic form using SymX types (`Scalar`, `Vector`).
2. Register it with `GlobalPotential` via a connectivity table.

## Worked Example: `EnergyMagneticAttraction`

The complete, runnable example is in [examples/main.cpp](../../examples/main.cpp) — see the `EnergyMagneticAttraction` struct and `magnetic_deformables_implicit()` scene function.

### What it does

Each deformable vertex `i` is attracted to a magnet position by the regularised 1/r potential:

$$E_i = -\frac{k}{\|\mathbf{x}_1^i - \mathbf{p}_{\mathrm{magnet}}\|_\varepsilon}$$

where $\|\cdot\|_\varepsilon = \sqrt{\|\cdot\|^2 + \varepsilon^2}$ avoids a singularity at zero distance, $k$ is a per-group strength constant, and $\mathbf{x}_1^i = \mathbf{x}_0^i + \Delta t\,\mathbf{v}_1^i$ is the time-integrated position (the velocity $\mathbf{v}_1$ is the DOF the Newton solver updates).

Because the energy is registered implicitly, SymX differentiates it with respect to $\mathbf{v}_1$ automatically, giving the attractive 1/r² force and its Hessian contribution within each Newton iteration.

### Struct layout

```cpp
struct EnergyMagneticAttraction
{
    stark::PointDynamics* dyn = nullptr;
    symx::LabelledConnectivity<2> conn{{ "glob", "group" }};
    Eigen::Vector3d magnet_center = Eigen::Vector3d::Zero();
    std::vector<double> strength;   // one entry per add() call
    double eps_sq = 0.01 * 0.01;   // regularisation constant (m²)

    void initialize(stark::core::Stark& stark_core, stark::PointDynamics* dynamics);
    void add(const stark::PointSetHandler& set, double strength_val);
    void update_magnet_center(const stark::RigidBodyHandler& rb);
};
```

`conn` is a `LabelledConnectivity<2>` with columns `{"glob", "group"}`.
Each row maps one vertex (global index) to one strength group.
The SymX lambda receives one row at a time via `node["glob"]` and `node["group"]`.

### Registering the potential

The `initialize()` method calls `add_potential` once, at setup time.
The lambda defines the symbolic energy expression; it is **not** called per time step — it is compiled once and then evaluated automatically by STARK:

```cpp
void initialize(stark::core::Stark& stark_core, stark::PointDynamics* dynamics)
{
    using namespace symx;
    this->dyn = dynamics;

    stark_core.global_potential->add_potential("EnergyMagneticAttraction", this->conn,
        [&](MappedWorkspace<double>& mws, Element& node)
        {
            // DOF: velocity at time n+1
            Vector v1 = mws.make_vector(this->dyn->v1.data, node["glob"]);
            // Known state at time n
            Vector x0 = mws.make_vector(this->dyn->x0.data, node["glob"]);
            // Time step (scalar reference, read-only each evaluation)
            Scalar dt = mws.make_scalar(stark_core.dt);
            // Per-group strength constant
            Scalar k  = mws.make_scalar(this->strength, node["group"]);
            // Magnet center: reference to member, updated each time step via callback
            Vector magnet = mws.make_vector(this->magnet_center);

            // Time-integrated position: x1 = x0 + dt * v1
            Vector x1 = x0 + dt * v1;

            // Regularised attractive potential
            Vector diff = x1 - magnet;
            Scalar dist = diff.stable_norm(this->eps_sq);
            return -k / dist;
        }
    );
}
```

### Adding vertices and wiring into a scene

```cpp
// In the scene function:
EnergyMagneticAttraction mag_energy;  // must outlive simulation.run()

// simulation.get_stark() exposes the core::Stark instance (callbacks, global_potential, dt).
mag_energy.initialize(simulation.get_stark(), simulation.deformables->point_sets.get());

for (auto& obj : objs) {
    mag_energy.add(obj.point_set, /*strength=*/0.1);
}

// Keep magnet_center up to date before each Newton solve:
simulation.get_stark().callbacks->add_before_time_step([&]() {
    mag_energy.update_magnet_center(magnet.rigidbody);
});
```

`Simulation::get_stark()` is a public getter that exposes the internal `core::Stark` instance, giving access to `global_potential`, `callbacks`, `dt`, and `gravity`.

### Data lifetime

The compiled potential holds **references** (not copies) to:
- `this->dyn->v1.data`, `this->dyn->x0.data` — kept alive by `PointDynamics`
- `this->strength` — member of the struct
- `this->magnet_center` — member of the struct
- `stark_core.dt` — member of `core::Stark`

The struct must therefore not be moved or destroyed while the simulation is running.
In the scene function the struct lives on the stack and `simulation.run()` returns before the stack frame unwinds, which is safe.

## Key SymX API

| Call | What it produces |
|---|---|
| `mws.make_vector(std::vector<Eigen::Vector3d>& arr, node["label"])` | Symbolic `Vector` indexed into `arr` |
| `mws.make_vector(const Eigen::Vector3d& v)` | Constant symbolic `Vector` (reference to `v`) |
| `mws.make_scalar(std::vector<double>& arr, node["label"])` | Symbolic `Scalar` indexed into `arr` |
| `mws.make_scalar(const double& s)` | Constant symbolic `Scalar` (reference to `s`) |
| `Vector::stable_norm(double eps_sq)` | $\sqrt{\|\cdot\|^2 + \varepsilon^2}$ — regularised norm |
| `global_potential->add_potential(name, conn, lambda)` | Register energy; lambda called once at compile time |

## Looking at Existing Models

- **`EnergyMagneticAttraction`** in [examples/main.cpp](../../examples/main.cpp) — the worked example above; self-contained, no library changes needed.
- **`EnergyPrescribedPositions`** — a simple penalty attachment. See `stark/src/models/deformables/point/`.
- **`EnergyLumpedInertia`** — per-vertex inertia. See `stark/src/models/deformables/`.
- **`EnergyAttachments`** — deformable–rigid attachments. See `stark/src/models/interactions/`.

## Callbacks in Custom Models

```cpp
// Called before each Newton solve — use to update time-varying parameters.
stark.callbacks->add_before_time_step([&]() { ... });

// Called after Newton converges — return false to reject the step and harden stiffness.
stark.callbacks->newton->add_is_converged_state_valid([&]() -> bool {
    return check_validity();
});
```

The `is_converged_state_valid` callback is the standard mechanism for adaptive stiffness hardening, as used internally by the contact model.
