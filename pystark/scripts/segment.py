# -*- coding: utf-8 -*-
import sys
build_dir = "D:/builds/stark"
bin_folder = build_dir + "/pystark/RelWithDebInfo"
sys.path.append(bin_folder)

import numpy as np
import pystark

settings = pystark.Settings()
settings.output.simulation_name = "segment"
settings.output.output_directory = build_dir + "/output/" + settings.output.simulation_name
settings.output.codegen_directory = build_dir + "/codegen"
settings.execution.end_simulation_time = 5.0
settings.contact.collisions_enabled = False
settings.newton.project_to_PD = False
simulation = pystark.Simulation(settings)

# Add segment
v, c, h = simulation.deformables().add_line_as_segments(
    np.array([0.0, 0.0, 0.0]),  # begin
    np.array([1.0, 1.0, 1.0]),  # end
    100, # n_segments
    pystark.Line.Params.Elastic_Rubberband(),
    "segment"
)

# Declare Prescribed Positions
pp = simulation.deformables().prescribed_positions().add(
    h.point_set, 
    np.array([0]), 
    pystark.EnergyPrescribedPositions.Params()
)

# Run
simulation.run()