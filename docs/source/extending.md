# Extending Stark

Stark's physics models are defined using SymX — the symbolic differentiation and JIT compilation engine bundled with Stark.
Adding a new material, joint, or interaction model requires writing the energy potential in symbolic form.
Stark takes care of differentiation, code generation, compilation, and assembly automatically.

> This page gives an overview of the process.
> For the full SymX API reference, see the [SymX documentation](https://github.com/InteractiveComputerGraphics/SymX).

## The Core Idea

Every physics model in Stark is an **energy potential** — a scalar function of the degrees of freedom (DoFs).
Stark assembles all potentials, differentiates them symbolically to get the gradient and Hessian, and feeds them to Newton's Method.

To add a model you need to:

1. Register your DoF arrays with the `GlobalPotential`.
2. Define a potential function using SymX symbolic types (`Scalar`, `Vector`, `Matrix`).
3. Register the potential with the `GlobalPotential`.

The framework handles the rest.

## Minimal Example: Custom Point Spring

```cpp
#include <stark>
#include <symx>

struct MySpring
{
    // Data
    std::vector<Eigen::Vector3d> x;  // current positions (DoFs)
    std::vector<Eigen::Vector3d> x0; // rest positions
    std::vector<std::array<int, 2>> edges; // spring connectivity
    double stiffness = 1e4;

    void add_to_simulation(stark::Simulation& sim)
    {
        auto& G = *sim.stark.global_potential;   // GlobalPotential
        using namespace symx;

        // 1. Register DoFs
        G.add_dof_array(this->x, "spring_x");

        // 2. Define the potential
        G.add_potential("my_spring", this->edges,
            [&](MappedWorkspace<double>& mws, Element& elem)
            {
                // Create symbols from data
                Vector xa  = mws.make_vector(this->x,  elem[0]);
                Vector xb  = mws.make_vector(this->x,  elem[1]);
                Vector Xa  = mws.make_vector(this->x0, elem[0]);
                Vector Xb  = mws.make_vector(this->x0, elem[1]);
                Scalar k   = mws.make_scalar(this->stiffness);

                // Energy: 0.5 * k * (|xa - xb| - |Xa - Xb|)^2
                Scalar rest_len    = (Xa - Xb).norm();
                Scalar current_len = (xa - xb).norm();
                Scalar stretch     = current_len - rest_len;
                return 0.5 * k * stretch * stretch;
            }
        );
    }
};
```

## Key SymX Concepts Used in Stark

| Concept | What it does |
|---|---|
| `MappedWorkspace<double>` | Connects symbolic variables to your data arrays; manages the map from symbol to memory |
| `mws.make_vector(array, idx)` | Creates a symbolic `Vector` that, at evaluation time, reads from `array[idx]` |
| `mws.make_scalar(value)` | Creates a symbolic `Scalar` from a scalar data reference |
| `Element& elem` | Holds the connectivity indices for the current element (e.g. `elem[0]`, `elem[1]`) |
| `G.add_potential(name, connectivity, lambda)` | Registers the energy for all elements in the connectivity list |
| `G.add_dof_array(array, name)` | Declares an array as a set of degrees of freedom |

## Looking at Existing Models

The cleanest way to learn how to extend Stark is to read an existing, simple model:

- **`EnergyPrescribedPositions`** — a point-to-point penalty attachment; one of the simplest models.
  See `stark/src/models/deformables/point/`.
- **`EnergyLumpedInertia`** — per-vertex inertia energy (diagonal mass matrix via penalty).
  See `stark/src/models/deformables/`.
- **`EnergyAttachments`** — multi-type attachment constraints between point sets and rigid bodies.
  See `stark/src/models/interactions/`.

## SymX Documentation

The full SymX API is documented at the [SymX GitHub repo](https://github.com/InteractiveComputerGraphics/SymX).

Topics most relevant to extending Stark:

- **Symbol–Data Maps (Layer 3):** `MappedWorkspace`, `CompiledInLoop` — the mechanism Stark uses internally for all its models.
- **Second-Order Optimization (Layer 4):** `GlobalPotential`, `NewtonsMethod` — the solver infrastructure Stark wraps.
- **FEM Integration:** built-in FEM element types and integration helpers (`fem_integrator`, `fem_jacobian`).

## Callbacks in Custom Models

Custom models often need to hook into the simulation loop.
Use `stark.core.Stark` callbacks (accessible via `sim.stark` in `Simulation`):

```cpp
stark.callbacks->add_before_time_step([&]() {
    // Update any data that changes each time step
});

stark.callbacks->newton->add_is_converged_state_valid([&]() -> bool {
    // Return false if the converged state is invalid (e.g. interpenetration)
    // Stark will harden your stiffness parameter and retry the time step
    return check_validity();
});
```

The `is_converged_state_valid` callback is the standard mechanism for adaptive stiffness hardening (as used by the contact model).
