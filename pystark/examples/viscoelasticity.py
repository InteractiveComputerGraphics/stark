# -*- coding: utf-8 -*-
import numpy as np
import pystark

settings = pystark.Settings()
settings.output.simulation_name = "viscoelasticity"
settings.output.output_directory = "output_folder"
settings.output.codegen_directory = "codegen_folder"
simulation = pystark.Simulation(settings)

# Contact
contact_params = pystark.EnergyFrictionalContact.GlobalParams()
contact_params.default_contact_thickness = 0.00025
contact_params.friction_enabled = False
contact_params.min_contact_stiffness = 1e7
simulation.interactions().contact().set_global_params(contact_params)

# Box
s = 0.2
f = 0.25
d = s*f
n = 25

material = pystark.Volume.Params.Soft_Rubber()
material.strain.damping = 2e3
bV, bT, bH = simulation.presets().deformables().add_volume_grid("box",
    size=np.array([s, s, d]),
    subdivisions=np.array([n, n, f*n], dtype=np.int32),
    params=material
)

# Torus
tV, tC, tH = simulation.presets().rigidbodies().add_torus("torus", 1.0, 0.05, 0.01)
tH.rigidbody.add_translation(0.05*pystark.UNITZ)
fix = simulation.rigidbodies().add_constraint_fix(tH.rigidbody)


# BC
simulation.deformables().prescribed_positions().add_inside_aabb(bH.point_set, np.array([0.0, 0.0, -0.5*d]), np.array([s, s, 0.001]), pystark.EnergyPrescribedPositions.Params().set_stiffness(1e7).set_tolerance(0.005))

# Script
simulation.add_time_event(0, 2.0, lambda t: fix.set_transformation((0.05 + pystark.blend(0.0, -0.04, 0.0, 2.0, t, pystark.BlendType.Linear))*pystark.UNITZ , 0.0, pystark.UNITZ))
simulation.add_time_event(4.0, 6.0, lambda t: fix.set_transformation((0.01 + pystark.blend(0.0, 0.1, 4.0, 6.0, t, pystark.BlendType.Linear))*pystark.UNITZ , 0.0, pystark.UNITZ))

# Run
duration = 8.0
simulation.run(duration)
