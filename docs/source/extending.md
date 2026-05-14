# Extending STARK

STARK physics models are SymX energy potentials, scalar functions of the degrees of freedom.
SymX differentiates them symbolically and feeds the gradient and Hessian to Newton's method.

The simplest form of extension is adding a model inside the existing infrastructure (e.g. new bending modes in `EnergyDiscreteShells`).
This page shows how to inject a custom energy from outside, without modifying STARK internals.

## The Core Idea

The DoFs for deformable objects are registered by `PointDynamics` (velocities `v1`, positions `x0`/`x1`), and analogously for rigid bodies in `RigidBodyDynamics`.
To add a custom energy, register it with `GlobalPotential` via a connectivity table.

See the [SymX docs](https://symx.physics-simulation.org/second_order_optimization.html#) for the `GlobalPotential` API.


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

As you can see, non-trivial implicit effects that would otherwise require deep solver expertise reduce to a handful of lines of symbolic code.
This is what makes STARK effective as a research and prototyping platform.

The full example is in `examples/main.cpp` as `magnetic_deformables_implicit()`.

