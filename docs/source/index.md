# Stark Documentation

<p align="center">
    <img src="spinning_box_cloth.gif" alt="Cloth falling on a spinning rigid box" style="width:60%;">
</p>

**Stark** is a C++ and Python simulation platform providing robust, state-of-the-art methods for simulating rigid and deformable objects in strongly coupled scenarios.

Stark combines a rich set of physics models — FEM deformables, discrete shells, rigid bodies with joints, and IPC frictional contact — with a powerful symbolic differentiation engine ([SymX](https://github.com/InteractiveComputerGraphics/SymX)) that eliminates the manual derivation of gradients and Hessians.
The result is a simulator that is both **easy to use** and **research-ready**.

Check out the [Stark GitHub repo](https://github.com/InteractiveComputerGraphics/stark) and the [Stark ICRA'24 paper](https://www.animation.rwth-aachen.de/publication/0588/).

## Table of Contents

```{toctree}
:caption: Getting Started
:maxdepth: 1

hello_world
setup
architecture
```

```{toctree}
:caption: Core Concepts
:maxdepth: 1

settings
simulation_loop
```

```{toctree}
:caption: Physics Models
:maxdepth: 1

deformables
rigidbodies
rb_constraints
contact
attachments
```

```{toctree}
:caption: High-Level API
:maxdepth: 1

presets
python_api
```

```{toctree}
:caption: Advanced
:maxdepth: 1

extending
```
