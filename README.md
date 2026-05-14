# STARK

<p align="center">
    <img src="docs/source/_static/stark1920.png" alt="STARK Logo" style="width:75%;">
</p>

<p align="center">
    <strong>Robust Coupled Simulation · Symbolic Differentiation </strong><br>
    <strong>FEM · Volumes · Shells · Rigid Bodies · Frictional Contact</strong><br>
    <a href="https://stark.physics-simulation.org/">Docs</a> &nbsp;·&nbsp; <a href="https://animation.rwth-aachen.de/media/papers/88/2024_-_ICRA_-_STARK.pdf">PDF</a> &nbsp;·&nbsp; <a href="https://doi.org/10.1109/icra57147.2024.10610574">IEEE Page</a>
</p>


**STARK** is a C++ and Python simulation platform providing robust, state-of-the-art methods for simulating rigid and deformable objects in strongly coupled scenarios. It provides a rich set of physics models — FEM deformables, discrete shells, rigid bodies with joints, and IPC frictional contact — with a powerful symbolic differentiation engine ([SymX](https://github.com/InteractiveComputerGraphics/SymX)) that eliminates the need for virtually any manual task beyond mathematical and discretization definitions.

STARK is **very easy to use** and it is **great for research**.
It has been validated through real-world, challenging cases of interactions between robots and deformable objects, see the [STARK ICRA'24 paper](https://www.animation.rwth-aachen.de/publication/0588/).

The following simulations have been run in the STARK/SymX:
<p align="center">
  <img src="https://github.com/InteractiveComputerGraphics/SymX/blob/gh-pages/gallery.webp" alt="SymX gallery" width="680">
</p>


## Hello World
Hee is a script using STARK's Python API to run a simulation of a piece of cloth falling on a rigid box with a prescribed spinning motion.

<p align=center>
 <img src="docs/images/spinning_box_cloth.gif">
</p>

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

## Features
* Deformable objects
  - 2D and 3D linear FEM (triangles and tets) with the Neo-Hookean and Stable Neo-Hookean constitutive models, respectively ([paper](https://dl.acm.org/doi/10.1145/3180491))
  - Discrete Shells ([paper](https://dl.acm.org/doi/10.5555/846276.846284))
  - Strain limiting ([paper](https://dl.acm.org/doi/10.1145/3450626.3459767))
  - Inertial and material damping
* Rigid bodies
  - Soft joints (linear)
  - Smooth force/torque capped motors [(paper)](https://www.animation.rwth-aachen.de/publication/0588/)
  - Comprehensive collection of constraints (ball joints, hinge joints, sliders, ...)
* Frictional contact (based on triangle meshes)
  - IPC [(paper)](https://dl.acm.org/doi/abs/10.1145/3386569.3392425)
* Attachments (based on triangle mesh distances)
* SymX for symbolic differentiation and optimization
* Triangle-based collision detection
* Event-based scripting


## Get STARK
TODO: Brief summary of installing with CMake
See the docs! (link)

### Python API
TODO: install with `pip install stark-sim`
TODO: Build from source (see the docs! link)

## Examples
TODO

## Documentation
TODO



## Research using STARK
* [Progressively Projected Newton's Method](https://arxiv.org/abs/2505.21013)
* [Strongly coupled simulation of magnetic rigid bodies](https://doi.org/10.1111/cgf.15185)
* [Micropolar Elasticity in Physically-Based Animation](https://doi.org/10.1145/3606922)
* [Curved Three‐Director Cosserat Shells with Strong Coupling](https://doi.org/10.1111/cgf.15183)


## Cite STARK
```bibtex
@InProceedings{FLL+24,
  author={Fernández-Fernández, José Antonio and Lange, Ralph and Laible, Stefan and Arras, Kai O. and Bender, Jan},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={STARK: A Unified Framework for Strongly Coupled Simulation of Rigid and Deformable Bodies with Frictional Contact}, 
  year={2024},
  pages={16888-16894},
  doi={10.1109/ICRA57147.2024.10610574}}
}
```

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

List of collaborators to the codebase:
  - [José Antonio Fernández-Fernández](https://github.com/JoseAntFer)
  - [Fabian Löschner](https://github.com/w1th0utnam3)
