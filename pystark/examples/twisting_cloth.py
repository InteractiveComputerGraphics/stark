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
settings.output.simulation_name = "twisting_cloth"
settings.output.output_directory = os.path.join(build_dir, "output/twisting_cloth")
settings.output.codegen_directory = os.path.join(build_dir, "codegen")

settings.simulation.gravity = pystark.ZERO
settings.simulation.max_time_step_size = 1.0/30.0
settings.newton.residual_tolerance_abs = 1e-6
settings.newton.step_tolerance = 1e-3

simulation = pystark.Simulation(settings)

# Contact
contact_params = pystark.EnergyFrictionalContact.GlobalParams()
contact_params.default_contact_thickness = 0.0005
contact_params.min_contact_stiffness = 1e5
contact_params.friction_enabled = False
simulation.interactions().contact().set_global_params(contact_params)

# Geometrical description
s = 0.5  # Size [m]
n = 64   # Subdivisions per dimension

# Add surface
material = pystark.Surface.Params.Cotton_Fabric()
material.inertia.damping = 0.1
material.strain.elasticity_only = True
v, c, h = simulation.presets().deformables().add_surface_grid(
    "cloth",
    np.array([s, s]),  # size
    np.array([n, n]),  # end
    material
)
h.point_set.add_rotation(90.0, np.array([1.0, 0.0, 0.0]))
simulation.interactions().contact().set_friction(h.contact, h.contact, 0.1)

# Declare Prescribed Positions
left = simulation.deformables().prescribed_positions().add_inside_aabb(
    h.point_set, 
    np.array([-s/2.0, 0.0, 0.0]), # AABB center
    np.array([0.001, s, s]), # AABB dim
    pystark.EnergyPrescribedPositions.Params()
)
right = simulation.deformables().prescribed_positions().add_inside_aabb(
    h.point_set, 
    np.array([s/2.0, 0.0, 0.0]), # AABB center
    np.array([0.001, s, s]), # AABB dim
    pystark.EnergyPrescribedPositions.Params()
)

# Script
duration = 10.0
angular_velocity = 90.0  # [deg/s]
Vec3d_zero = np.array([0.0, 0.0, 0.0])
Vec3d_unitX = np.array([1.0, 0.0, 0.0])
simulation.add_time_event(0, duration, lambda t: left.set_transformation(Vec3d_zero, angular_velocity*t, Vec3d_unitX))
simulation.add_time_event(0, duration, lambda t: right.set_transformation(Vec3d_zero, -angular_velocity*t, Vec3d_unitX))

# Run
simulation.run(duration)
