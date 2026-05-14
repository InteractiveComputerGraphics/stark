# Extending STARK

STARK's physics models are defined using [SymX](https://github.com/InteractiveComputerGraphics/SymX), the symbolic differentiation and JIT compilation engine bundled with STARK.
The easiest way to extend STARK is by adding functionality to the existing infrastructure.
For instance, it would be rather easy to add a new bending potential or effect directly by extending `EnergyDiscreteShells` with new models and parameters.
In this page however, we cover the process to injecting functionality from the outside to illustrate that option.

## The Core Idea

Every physics model in STARK is an **energy potential**, that is, a scalar function of the degrees of freedom (DoFs).
STARK assembles all potentials, differentiates them symbolically to obtain the gradient and Hessian, and feeds them to Newton's method.

The DoFs for deformable objects are already registered by `PointDynamics` (velocities `v1`, positions `x0`/`x1`), and analogously for rigid bodies in `RigidBodyDynamics`.
To add a custom energy you only need to register it with `GlobalPotential` via a connectivity table.

Check [SymX docs](https://symx.physics-simulation.org/second_order_optimization.html#) on global potential definitions.


## Example: implicit magnetic attraction

The following example adds a simple magnetic attraction potential to deformable boxes. The magnet is represented by a scripted rigid sphere, while the actual attraction is a custom SymX energy acting on the deformable vertices.

For every affected deformable vertex, the energy is

$$
E_i(x_i) = -\frac{k}{\sqrt{\|x_i - x_c\|^2}},
$$

where $x_i$ is the current candidate position of the attracted vertex, `x_c` is the current magnet position and `k` controls the strength.

The force produced by this energy is attractive, and because the term is registered as a SymX potential, STARK also receives the corresponding Hessian contribution automatically.
This is incorporated to the rest of the physical effects and piped into the optimization pipeline.

The following is the energy definition

```cpp
stark::core::Stark& stark_core = simulation.get_stark();
stark::PointDynamics* dyn = simulation.deformables->point_sets.get();

stark_core.global_potential->add_potential("EnergyMagneticAttraction", magnetic_vertices,
    [&](MappedWorkspace<double>& mws, Element& elem)
    {
        Vector v1 = mws.make_vector(dyn->v1.data, elem["point"]);
        Vector x0 = mws.make_vector(dyn->x0.data, elem["point"]);
        Scalar dt = mws.make_scalar(stark_core.dt);
        Scalar k = mws.make_scalar(magnet_force);
        Vector m = mws.make_vector(magnet_center);

        Vector x1 = stark::time_integration(x0, v1, dt);
        Vector r = x1 - m;
        Scalar d = r.norm();

        return -k / d;
    }
);
```

As you can see, incorporating implicit non-trivial effects that otherwise would require significant knowledge and expertise of the codebase takes just a few lines of code.
This is what makes STARK a powerful research tool and repository as it allows for very quick iterations and novel compositions of complex problems.

The entire example can be found in examples as `magnetic_deformables_implicit()`.

