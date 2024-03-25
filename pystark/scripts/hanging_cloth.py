# -*- coding: utf-8 -*-
import pystark_paths
import sys
sys.path.append(pystark_paths.bin_dir)

import numpy as np
import pystark

settings = pystark.Settings()
settings.output.simulation_name = "hanging_cloth"
settings.output.output_directory = pystark_paths.build_dir + "/output/" + settings.output.simulation_name
settings.output.codegen_directory = pystark_paths.build_dir + "/codegen"
settings.execution.end_simulation_time = 5.0
settings.simulation.init_frictional_contact = False
simulation = pystark.Simulation(settings)

# Geometrical description
s = 0.5  # Size [m]
n = 20   # Subdivisions per dimension

# Add surface
v, c, h = simulation.presets().deformables().add_surface_grid(
    "cloth",
    np.array([s, s]),  # size
    np.array([n, n]),  # end
    pystark.Surface.Params.Cotton_Fabric()
)

# Declare Prescribed Positions
pp = simulation.deformables().prescribed_positions().add(
    h.point_set, 
    np.array([0, n]), 
    pystark.EnergyPrescribedPositions.Params()
)

# Run
simulation.run()