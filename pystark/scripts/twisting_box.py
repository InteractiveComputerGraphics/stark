# -*- coding: utf-8 -*-
import sys
build_dir = "D:/builds/stark"
bin_folder = build_dir + "/pystark/RelWithDebInfo"
sys.path.append(bin_folder)

import numpy as np
import pystark

settings = pystark.Settings()
settings.output.simulation_name = "twisting_box"
settings.output.output_directory = build_dir + "/output/" + settings.output.simulation_name
settings.output.codegen_directory = build_dir + "/codegen"
settings.execution.end_simulation_time = 5.0
settings.contact.collisions_enabled = False
settings.simulation.gravity = np.array([0.0, 0.0, 0.0])
settings.newton.project_to_PD = False
simulation = pystark.Simulation(settings)

# Geometrical description
s = 0.2  # Size [m]
n = 7    # Subdivisions per dimension

# Add box
dim = np.array([s, s, s])
subdivisions = np.array([n, 3*n, n])
params = pystark.Volume.Params.Soft_Rubber()
params.strain.set_elasticity_only(True)  # Don't need damping nor strain limiting
params.strain.set_poissons_ratio(0.4)  # A bit more volume conserving
v, c, h = simulation.deformables().add_volume_grid(dim, subdivisions, params, "box")

# Declare Prescribed Positions
def declare_pp(y):
    handler = simulation.deformables().prescribed_positions().add_inside_aabb(
        h.point_set, 
        np.array([0.0, y, 0.0]), # AABB center
        np.array([s, 0.001, s]), # AABB dimensions
        pystark.EnergyPrescribedPositions.Params()
            .set_stiffness(1e6)
            .set_tolerance(0.005)
        )
    return handler

left_pp = declare_pp(-s/2.0)
right_pp = declare_pp(s/2.0)

# Script function that will be executed at the beginning of each time step
def script(t):
    right_pp.set_transformation(
        np.array([0.0, 0.1*t, 0.0]),  # Translation [m]
        50.0*t,  # Angle [deg]
        np.array([0.0, 1, 0.0])  # Rotation axis
    )

# Run
simulation.run(lambda : script(simulation.get_time()))