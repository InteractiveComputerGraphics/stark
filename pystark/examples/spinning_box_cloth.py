# -*- coding: utf-8 -*-
import os
import sys
import numpy as np

# PYTHONPATH to the built stark python module (adjust as needed)
build_dir = os.path.join(os.path.dirname(__file__), "../../build")
include_path = os.path.join(os.path.dirname(__file__), "..")
sys.path.append(include_path)
import pystark


# 1. Configure output and solver settings
settings = pystark.Settings()
settings.output.simulation_name = "spinning_box_cloth"
settings.output.output_directory = os.path.join(build_dir, "output/spinning_box_cloth")
settings.output.codegen_directory = os.path.join(build_dir, "codegen")

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
