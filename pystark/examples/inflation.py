# -*- coding: utf-8 -*-
import os
import sys
import numpy as np

# PYTHONPATH to the built stark python module (adjust as needed)
build_dir = os.path.join(os.path.dirname(__file__), "../../build")
include_path = os.path.join(os.path.dirname(__file__), "..")
sys.path.append(include_path)
import pystark


# Create simulation
settings = pystark.Settings()
settings.output.simulation_name = "inflation"
settings.output.output_directory = os.path.join(build_dir, "output/inflation")
settings.output.codegen_directory = os.path.join(build_dir, "codegen")

settings.simulation.gravity = pystark.ZERO
settings.simulation.init_frictional_contact = False
settings.simulation.max_time_step_size = 1.0/30.0
settings.newton.residual_tolerance_abs = 1e-6
settings.newton.step_tolerance = 1e-6
settings.newton.min_iterations = 1

simulation = pystark.Simulation(settings)

# Dimensions
s = 0.4
n = 64

# Add deformable surface
material = pystark.Surface.Params.Cotton_Fabric()
material.inertia.damping = 1.0
material.strain.inflation = 1e4
material.strain.strain_limit_stiffness = 5e3

cV, cT, cH = simulation.presets().deformables().add_surface_grid("cloth",
    size=np.array([s, s]),
    subdivisions=np.array([n, n], dtype=np.int32),
    params=material
)

# Prescribed positions
simulation.deformables().prescribed_positions().add_inside_aabb(cH.point_set, np.array([0.5*s, 0.5*s, 0.0]), 0.001*pystark.ONES, pystark.EnergyPrescribedPositions.Params())
simulation.deformables().prescribed_positions().add_inside_aabb(cH.point_set, np.array([0.5*s, -0.5*s, 0.0]), 0.001*pystark.ONES, pystark.EnergyPrescribedPositions.Params())
simulation.deformables().prescribed_positions().add_inside_aabb(cH.point_set, np.array([-0.5*s, 0.5*s, 0.0]), 0.001*pystark.ONES, pystark.EnergyPrescribedPositions.Params())
simulation.deformables().prescribed_positions().add_inside_aabb(cH.point_set, np.array([-0.5*s, -0.5*s, 0.0]), 0.001*pystark.ONES, pystark.EnergyPrescribedPositions.Params())

# Script
duration = 6.0
def script(t):
    material.strain.inflation = pystark.blend(0.0, 1e4, 0.0, duration, t, pystark.BlendType.EaseInOut)
    cH.strain.set_params(material.strain)
simulation.add_time_event(0, duration, script)
simulation.run(duration)
