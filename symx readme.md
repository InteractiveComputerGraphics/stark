# SymX

<p align="center">
    <img src="docs/source/_static/symx1920.png" alt="SymX Logo" style="width:75%;">
</p>

<p align="center">
    <strong>Symbolic differentiation. C++ code generation. JIT compilation. Global assembly. Non-linear optimization.</strong><br>
    <a href="https://symx.physics-simulation.org/">Docs</a> &nbsp;·&nbsp; <a href="https://animation.rwth-aachen.de/media/papers/96/2025-TOG-SymX.pdf">PDF</a> &nbsp;·&nbsp; <a href="https://doi.org/10.1145/3764928">ACM Page</a>
</p>

SymX is a C++ library for **symbolic differentiation** with automatic **code generation**, **compilation** and **evaluation**.
Write complex mathematical expressions concisely, differentiate them arbitrarily, and let SymX evaluate them on your data structures — including global gradient and Hessian assembly.

SymX targets **non-linear optimization pipelines** typical of **FEM solvers**, but it can be used for any application that needs JIT compiled math.
It uses a **stencil-based** perspective: expressions are defined per _element_ and evaluated over a discretization.
SymX is the core engine of [STARK](https://github.com/InteractiveComputerGraphics/stark), a simulation framework for FEM elasticity, shells, rigid bodies, and frictional contact.

Here is an overview of the SymX pipeline for FEM elasticity simulation:
<p align="center">
    <img src="docs/source/_static/overview.jpg" alt="SymX overview" style="width:85%;">
</p>

The goal is to reduce **time-to-solution**.
In research, the bottleneck is often the time between an idea and a trustworthy result.
SymX lets you quickly iterate while avoiding sinking time in **manual differentiation**, **testing derivatives**, **manual optimization**, **stack-indexed evaluation loops**, **parallelism/SIMD** and more.
You can develop complex solvers with great performance in a **fraction of the time and code footprint**, which quickly compounds productivity.

## Applications where SymX was used
Here a list of simulations solved using SymX, corresponding to public research listed in [Research Using SymX](README.md#research-using-symx):
* Linear and high-order FEM for elasticity
* Linear and high-order shell mechanics
* Constrained rigid body systems
* IPC frictional contact in and across mechanical systems
* Dynamic and quasistatic simulation
* Volumetric and membrane micropolar materials
* Volumetric and membrane strain limiting and viscoelasticity
* Magnetism
* Differentiable shell mechanics
* Adjoint method (diff sim) for quasistatics

The following gallery shows some of such results:
<p align="center">
  <img src="https://github.com/InteractiveComputerGraphics/SymX/blob/gh-pages/gallery.webp" alt="SymX gallery" width="680">
</p>

## Features
- **Powerful symbolics:** Arithmetic, trig, sqrt/pow, min/max/abs, branching; arbitrary nested differentiation; automatic CSE.
- **Fast processing:** Expression building, differentiation and C++ code generation take milliseconds, scaling well to high-order FEM and shell models.
- **Fast evaluation:** Generated C++ with optional AVX2/SIMD for batched multi-element evaluation.
- **Parallel execution:** OpenMP-parallelized loops with optional graph coloring for concurrent evaluation.
- **Incremental compilation:** Generated code is cached on disk; unchanged expressions skip differentiation and compilation.
- **Zero-overhead abstraction:** Definition code (lambdas, dynamic loops, containers) runs only during expression building; emitted binaries are fully specialized. This allows for very expressive compositions that are concise and powerful (e.g. high-order FEM).
- **Layered API:** Three entry points — single expression, expression + discretization loop, full second-order global solve.
- **Bring your own data:** Views into user arrays; no data format imposed on your discretization.
- **FEM integrator:** Symbolic FEM integrator with Tet4, Hex8, Hex27 and other element types for 3D mechanics.
- **Global assembly:** Concurrent and fast global assembly of gradients and Hessians from composable potential definitions; multiple coupled DoF sets supported.
- **Newton's method:** Robust second-order solver with line search; customizable convergence and callbacks.
- **Projection to PD:** Newton, [Projected Newton](https://dl.acm.org/doi/10.1145/1073368.1073394), [Project-on-Demand](https://arxiv.org/abs/2311.14526) and [Progressively Projected Newton](https://arxiv.org/abs/2505.21013), with clamping or [mirroring](https://dl.acm.org/doi/10.1145/3641519.3657433).


## Hello World
Here are "hello world" examples for the lowest and highest entry points of SymX.

### Low-level: compile a single expression

```cpp
Workspace sws;
Scalar x = sws.make_scalar();

Scalar dsinx_dx = diff(sin(x), x);

Compiled<double> compiled({ dsinx_dx }, "hello_world", "../codegen");
compiled.set(x, 0.0);

View<double> result = compiled.run();
std::cout << "dsinx_dx(0.0) = " << result[0] << std::endl;
```

It defines an expression containing derivatives.
SymX writes the generated source, compiles it with your system compiler and loads the shared object.
Then a numerical value is set for the symbol `x`, the function is executed and the result printed.

### High-level: Non-linear Newton solve from potential definitions

```cpp
spGlobalPotential G = GlobalPotential::create();

G->add_potential("neohookean_tet4", tets,
    [&](MappedWorkspace<double>& mws, Element& elem)
    {
        std::vector<Vector> x = mws.make_vectors(data.x, elem);
        std::vector<Vector> X = mws.make_vectors(data.X, elem);
        Scalar mu = mws.make_scalar(data.mu);
        Scalar lambda = mws.make_scalar(data.lambda);

        return neohookean_strain_energy_tet4(X, x, mu, lambda);
    }
);
G->add_potential("inertia", vertices,
    [&](MappedWorkspace<double>& mws, Element& elem)
    {
        Vector x = mws.make_vector(data.x, elem[0]);
        Vector x0 = mws.make_vector(data.x0, elem[0]);
        Vector v0 = mws.make_vector(data.v0, elem[0]);
        Vector a = mws.make_vector(data.a, elem[0]);
        Scalar m = mws.make_scalar(data.m, elem[0]);
        Scalar dt = mws.make_scalar(data.dt);

        return inertia_energy(x, x0, v0, a, dt, m);
    }
);

G->add_dof(data.x);

spContext context = Context::create();
NewtonsMethod newton(G, context);
SolverReturn ret = newton.solve();
```

Here we define two energy potentials using the functions shown in the diagram above.
SymX solves the problem using Newton's Method with all default parameters.
Data initialization is omitted for simplicity.
SymX takes over the heavy lifting: differentiation of element gradient and Hessian, code generation, compilation, evaluation, projection to PD, assembly, linear solves, line search, etc.

## Examples

This repository comes with a few examples to get you started.
Select the desired experiment (or all) in `examples/examples_main.cpp`.
You can find descriptions and links to the code in [Examples in Docs](https://symx.physics-simulation.org/examples.html).

Here is a summary:

- `examples/hello_world.cpp`: the minimal compile-and-run workflow
- `examples/triangle_mesh_area.cpp`: a cleanest expression + user discretization example
- `examples/NeoHookean.cpp`: FEM elasticity examples and evaluation benchmarks
- `examples/rubber_extrusion.cpp`: a compact, realistic Newton-based FEM solve
- `examples/xpbd_cloth.cpp`: proof that SymX can be applied beyond FEM
- `examples/dynamic_elasticity_with_contact.cpp`: Dynamic simulation with elasticity and IPC

## Documentation

Full documentation: <https://symx.physics-simulation.org/>

- [Hello World](https://symx.physics-simulation.org/hello_world.html)
- [Setup](https://symx.physics-simulation.org/setup.html)
- [Architecture Overview](https://symx.physics-simulation.org/diagram.html)
- [Core Symbolics](https://symx.physics-simulation.org/symbols.html)
- [Compilation](https://symx.physics-simulation.org/compilation.html)
- [Symbol-Data Maps](https://symx.physics-simulation.org/mapped_workspace.html)
- [Loops](https://symx.physics-simulation.org/compiled_in_loop.html)
- [Second-order Optimization](https://symx.physics-simulation.org/second_order_optimization.html)
- [FEM Integration](https://symx.physics-simulation.org/fem_integration.html)
- [Newton's Method](https://symx.physics-simulation.org/newtons_method.html)
- [Examples](https://symx.physics-simulation.org/examples.html)


## Build SymX

SymX bundles its core dependencies, requiring only `CMake 3.15+` and a `C++17` compiler.

```bash
cmake -B build
cmake --build build --parallel
build/tests/tests                 # Run tests
build/examples/examples           # Run examples
```

See [Setup in Docs](https://symx.physics-simulation.org/setup.html) for a detailed explanation of how to set SymX up and integrate it in a parent CMake project.

**Note for Windows users:** JIT compilation may be significantly slower on Windows than on Linux or macOS due to the way compiler toolchains are loaded dynamically.


## Research Using SymX

* [Progressively Projected Newton's Method](https://arxiv.org/abs/2505.21013)
* [Interactive facial animation: Enhancing facial rigs with real-time shell and contact simulation](https://doi.org/10.1145/3747860)
* [Strongly coupled simulation of magnetic rigid bodies](https://doi.org/10.1111/cgf.15185)
* [Micropolar Elasticity in Physically-Based Animation](https://doi.org/10.1145/3606922)
* [Curved Three‐Director Cosserat Shells with Strong Coupling](https://doi.org/10.1111/cgf.15183)
* [STARK: A unified framework for strongly coupled simulation of rigid and deformable bodies with frictional contact](https://doi.org/10.1109/ICRA57147.2024.10610574)

## Cite SymX

If SymX contributes to your research, please cite the paper.

```bibtex
@article{10.1145/3764928,
    author = {Fern\'{a}ndez-Fern\'{a}ndez, Jos\'{e} Antonio and L\"{o}schner, Fabian and Westhofen, Lukas and Longva, Andreas and Bender, Jan},
    title = {SymX: Energy-based Simulation from Symbolic Expressions},
    year = {2025},
    issue_date = {February 2026},
    publisher = {Association for Computing Machinery},
    address = {New York, NY, USA},
    volume = {45},
    number = {1},
    issn = {0730-0301},
    url = {https://doi.org/10.1145/3764928},
    doi = {10.1145/3764928},
    journal = {ACM Trans. Graph.},
    month = oct,
    articleno = {5},
    numpages = {19},
    keywords = {Physically-based simulation, symbolic differentiation, optimization time integration}
}
```

## Contributors
- [José Antonio Fernández-Fernández](https://github.com/JoseAntFer) (Corresponding author)
- [Fabian Löschner](https://github.com/w1th0utnam3)
- [Lukas Westhofen](https://github.com/BjoernBinaer)
- [Andreas Longva](https://github.com/Andlon)
- [Jan Bender](https://animation.rwth-aachen.de/person/1/)


## Want to Collaborate?

SymX is exactly the kind of project that benefits from real use in real environments.

If you are:

- deploying symbolic differentiation in a research codebase
- building non-linear FEM or optimization tooling
- exploring contact, constraints, differentiable simulation, or multi-physics problems
- interested in extending the symbolic, compilation, or solver layers

then feel free to reach out!

