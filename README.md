# STARK

<p align="center">
    <img src="docs/source/_static/stark1920.png" alt="STARK Logo" style="width:75%;">
</p>

<p align="center">
    <strong>FEM · Shells · Rigid Bodies · Frictional Contact · Strongly Coupled</strong><br>
    <a href="https://stark.physics-simulation.org/">Docs</a> &nbsp;·&nbsp; <a href="https://animation.rwth-aachen.de/media/papers/88/2024_-_ICRA_-_STARK.pdf">PDF</a> &nbsp;·&nbsp; <a href="https://doi.org/10.1109/icra57147.2024.10610574">IEEE Page</a>
</p>

**STARK** is a C++ and Python simulation platform for **strongly coupled simulation of rigid and deformable bodies with frictional contact**.
It provides a rich set of physics models — volumetric FEM, discrete shells, rods, rigid body joints, and IPC frictional contact — driven by [SymX](https://github.com/InteractiveComputerGraphics/SymX), a symbolic differentiation and JIT compilation engine that eliminates manual derivative computation, evaluation loops, and most performance tuning.

STARK is **very easy to use** and it is **great for research**.
It has been validated through real-world, challenging cases of interactions between robots and deformable objects, see the [STARK ICRA'24 paper](https://www.animation.rwth-aachen.de/publication/0588/).

<p align="center">
  <img src="https://github.com/InteractiveComputerGraphics/SymX/blob/gh-pages/gallery.webp" alt="STARK/SymX gallery" width="680">
</p>


## Features

- **Deformable objects**
  - 1D rods and cables — axial strain with optional strain limiting
  - 2D cloth and shells — Neo-Hookean membrane strain ([Stable Neo-Hookean](https://dl.acm.org/doi/10.1145/3180491)) + [discrete shell bending](https://dl.acm.org/doi/10.5555/846276.846284), inflation pressure
  - 3D soft bodies — linear tetrahedral FEM with Neo-Hookean constitutive model
  - [Strain limiting](https://dl.acm.org/doi/10.1145/3450626.3459767), inertial and material damping, quasistatic mode
  - Composite materials: freely mix volume, surface, and rod energies on a single mesh
- **Rigid bodies**
  - Comprehensive joint library: fix, ball, hinge, slider, universal, planar, point-on-axis, and more
  - Velocity controllers with smooth force/torque caps ([paper](https://www.animation.rwth-aachen.de/publication/0588/))
  - Damped springs and distance limits
  - Automatic stiffness hardening to enforce tight tolerances without hand-tuning
- **Frictional contact**
  - [IPC](https://dl.acm.org/doi/abs/10.1145/3386569.3392425)-based, guaranteed intersection-free
  - All coupling modes: deformable–deformable, rigid–deformable, rigid–rigid
  - Per-pair Coulomb friction, per-object contact thickness
- **Attachments** — penalty-based gluing by point list, barycentric coords, or proximity search; deformable–deformable and rigid–deformable
- **Powered by [SymX](https://github.com/InteractiveComputerGraphics/SymX)** — symbolic differentiation, automatic code generation, JIT compilation, OpenMP parallelism, and a robust Newton solver with line search and projection to PD
- **Python API** (`pystark`) — full access to the C++ API from Python with NumPy interop
- **Event-based scripting** — time events, callbacks, and animated boundary conditions
- **Extensible** — add custom SymX energy potentials without modifying STARK internals; the symbolic gradient and Hessian are derived automatically


## Hello World

```python
import numpy as np
import pystark

# 1. Configure output and solver settings
settings = pystark.Settings()
settings.output.simulation_name = "spinning_box_cloth"
settings.output.output_directory = "output_folder"
settings.output.codegen_directory = "codegen_folder"

# 2. Create the simulation
simulation = pystark.Simulation(settings)

# 3. Set global contact parameters
contact_params = pystark.EnergyFrictionalContact.GlobalParams()
contact_params.default_contact_thickness = 0.0025
simulation.interactions().contact().set_global_params(contact_params)

# 4. Add a deformable cloth surface
cV, cT, cH = simulation.presets().deformables().add_surface_grid(
    "cloth",
    size=np.array([0.4, 0.4]),
    subdivisions=np.array([32, 32]),
    params=pystark.Surface.Params.Cotton_Fabric()
)

# 5. Add a rigid body box
bV, bT, bH = simulation.presets().rigidbodies().add_box("box", mass=1.0, size=0.08)
bH.rigidbody.add_translation(np.array([0.0, 0.0, -0.08]))
fix_handler = simulation.rigidbodies().add_constraint_fix(bH.rigidbody)

# 6. Script: spin the box over time
duration = 10.0
def script(t):
    fix_handler.set_transformation(
        np.array([0.0, 0.0, -0.08]),
        90.0*t,
        np.array([0.0, 0.0, 1.0])
    )

# 7. Run
simulation.run(duration, script)
```

The native C++ scene definition is 1-to-1 the same calls.
Output is written as VTK files; you can open them in [ParaView](https://www.paraview.org/) or in Blender with the [Sequence Loader Addon](https://github.com/InteractiveComputerGraphics/blender-sequence-loader).


## Extending STARK

STARK physics models are SymX symbolic definitions of energy potentials.
You can inject custom physics, without modifying worrying about implementation details or STARK internals.

The following example adds an implicit magnetic attraction to deformable vertices.
What would take some effort, even for such a simple model, it's just a handful of lines:

```cpp
stark::core::Stark& stark_core = simulation.get_stark();
stark::PointDynamics* dyn = simulation.deformables->point_sets.get();

stark_core.global_potential->add_potential("EnergyMagneticAttraction", magnetic_vertices,
    [&](MappedWorkspace<double>& mws, Element& elem)
    {
        Vector v1  = mws.make_vector(dyn->v1.data, elem["point"]);
        Vector x0  = mws.make_vector(dyn->x0.data, elem["point"]);
        Scalar dt  = mws.make_scalar(stark_core.dt);
        Scalar k   = mws.make_scalar(magnet_force);
        Vector m   = mws.make_vector(magnet_center);

        Vector x1 = stark::time_integration(x0, v1, dt);
        Vector r  = x1 - m;
        return -k / r.norm();
    }
);
```


## Examples

The repository comes with C++ and Python examples to get you started.

**C++ examples** (`examples/main.cpp`):
- `hanging_deformable_box` — a soft body fixed by two corners, baseline for volumetric FEM
- `hanging_box_with_composite_material` — volume + surface + rod energies on a single mesh
- `quasistatic_column_extrusion` — quasistatic Neo-Hookean compression, large deformations
- `spinning_box_cloth` — cloth on a spinning rigid box, IPC contact and friction
- `deformable_and_rigid_collisions` — stacked soft boxes on a rigid floor
- `attachments` — two cloth panels and a rigid body glued by proximity-based attachments
- `magnetic_deformables_implicit` — custom SymX magnetic energy added from outside STARK

**Python examples** (`pystark/examples/`):
- `spinning_box_cloth.py` — the hello world scene
- `boxes_on_cloth.py` — stack of rigid boxes landing on a pinned cloth
- `twisting_cloth.py` — cloth twisted by prescribed boundary conditions
- `inflation.py` — inflatable membrane with animated internal pressure
- `viscoelasticity.py` — viscoelastic soft body compressed by a scripted rigid torus


## Get STARK

### C++

STARK requires only CMake 3.18+, a C++20 compiler, and OpenMP.

```bash
cmake -S . -B build
cmake --build build --parallel

build/examples/examples   # run C++ examples
```
See [Setup in Docs](https://stark.physics-simulation.org/setup.html) for the full integration guide.

### Python (`pystark`)

Build from source with CMake:

```bash
cmake -S . -B build \
  -DSTARK_BUILD_PYTHON_BINDINGS=ON \
  -DSTARK_PYTHON_EXECUTABLE=$(which python)
cmake --build build --parallel --target pystark

export PYTHONPATH=/path/to/stark/pystark:$PYTHONPATH
```

See [Setup in Docs](https://stark.physics-simulation.org/setup.html) for Conda/virtualenv instructions and Windows notes.


## Documentation

Full documentation: <https://stark.physics-simulation.org/>

- [Hello World](https://stark.physics-simulation.org/hello_world.html)
- [Setup](https://stark.physics-simulation.org/setup.html)
- [Architecture Overview](https://stark.physics-simulation.org/architecture.html)
- [Settings](https://stark.physics-simulation.org/settings.html)
- [Deformables](https://stark.physics-simulation.org/deformables.html)
- [Rigid Bodies](https://stark.physics-simulation.org/rigidbodies.html)
- [Rigid Body Constraints](https://stark.physics-simulation.org/rb_constraints.html)
- [Frictional Contact](https://stark.physics-simulation.org/contact.html)
- [Attachments](https://stark.physics-simulation.org/attachments.html)
- [Extending STARK](https://stark.physics-simulation.org/extending.html)


## Research Using STARK

* [Progressively Projected Newton's Method](https://arxiv.org/abs/2505.21013)
* [Strongly coupled simulation of magnetic rigid bodies](https://doi.org/10.1111/cgf.15185)
* [Micropolar Elasticity in Physically-Based Animation](https://doi.org/10.1145/3606922)
* [Curved Three‐Director Cosserat Shells with Strong Coupling](https://doi.org/10.1111/cgf.15183)


## Cite STARK

If STARK contributes to your research, please cite the paper.

```bibtex
@InProceedings{FLL+24,
  author={Fern\'{a}ndez-Fern\'{a}ndez, Jos\'{e} Antonio and Lange, Ralph and Laible, Stefan and Arras, Kai O. and Bender, Jan},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  title={STARK: A Unified Framework for Strongly Coupled Simulation of Rigid and Deformable Bodies with Frictional Contact},
  year={2024},
  pages={16888-16894},
  doi={10.1109/ICRA57147.2024.10610574}
}
```

## Want to Collaborate?

STARK is exactly the kind of project that benefits from real use in real environments.

If you are:

- simulating soft robots, surgical tools, garments, or other compliant mechanisms
- building FEM pipelines for animation or engineering
- exploring differentiable simulation, contact, or multi-physics coupling
- interested in extending the physics models, solver, or Python API

then feel free to reach out!


## Acknowledgments

<table border="0" style="align_center;border-radius: 20px;padding: 20px;margin:auto;border:0px">
  <tr>
    <td>
      <img src="https://raw.githubusercontent.com/boschresearch/bosch-corporate-information/main/static/Bosch_symbol_logo_black_red.svg" width="300">
    </td>
    <td>
      Robert Bosch GmbH is acknowledged for generous financial support of the development of the initial version of STARK from 2019 to 2021.
    </td>
  </tr>
</table>

Contributors to the codebase:
- [José Antonio Fernández-Fernández](https://github.com/JoseAntFer) (Corresponding author)
- [Fabian Löschner](https://github.com/w1th0utnam3)


