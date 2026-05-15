# STARK Documentation

<p align="center">
    <img src="_static/stark1920.png" alt="SymX Logo" style="width:75%;">
</p>

Welcome to the STARK documentation.
STARK is a C++ and Python simulation platform for simulating rigid and deformable objects in strongly coupled scenarios.
Check out the [STARK GitHub repo](https://github.com/InteractiveComputerGraphics/stark) and the [STARK ICRA'24 paper](https://www.animation.rwth-aachen.de/publication/0588/).

These pages give a high-level overview of the main concepts and show how to use the library.
They are not a doxygen-style exhaustive documentation, but rather a guide on how the library works and how to use it effectively.

STARK is built on top of [SymX](https://github.com/InteractiveComputerGraphics/SymX), a mathematical engine that handles differentiation, evaluation, and optimization via a Newton solver.
STARK can be seen as a repository of common simulation models with a convenient interface, scripting, and a Python API.

If you need something simpler or more customizable, using SymX directly may be a better fit.
If you need performance beyond what STARK offers (e.g. GPU), STARK can still serve as a rapid prototyping environment before committing to a custom solver.

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
io
simulation_loop
```

```{toctree}
:caption: Physics Models
:maxdepth: 1

problem_representation
presets
deformables
rigidbodies
rb_constraints
contact
attachments
```

```{toctree}
:caption: Advanced
:maxdepth: 1

extending
```
