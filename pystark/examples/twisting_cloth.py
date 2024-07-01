# -*- coding: utf-8 -*-
import numpy as np
import pystark

settings = pystark.Settings()
settings.output.simulation_name = "twisting_cloth"
settings.output.output_directory = "output_folder"
settings.output.codegen_directory = "codegen_folder"
settings.simulation.gravity = np.array([0.0, 0.0, 0.0])
simulation = pystark.Simulation(settings)

# Contact
contact_params = pystark.EnergyFrictionalContact.GlobalParams()
contact_params.default_contact_thickness = 0.00025
contact_params.min_contact_stiffness = 1e7
simulation.interactions().contact().set_global_params(contact_params)

# Geometrical description
s = 0.5  # Size [m]
n = 60   # Subdivisions per dimension

# Add surface
material = pystark.Surface.Params.Cotton_Fabric()
material.inertia.damping = 0.1
material.strain.strain_limit = np.inf  # No strain limit
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
