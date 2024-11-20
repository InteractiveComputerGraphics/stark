# -*- coding: utf-8 -*-
import numpy as np
import pystark

settings = pystark.Settings()
settings.output.simulation_name = "inflation"
settings.output.output_directory = "output_folder"
settings.output.codegen_directory = "codegen_folder"
settings.simulation.gravity = pystark.ZERO
settings.simulation.init_frictional_contact = False
simulation = pystark.Simulation(settings)

# Dimensions
s = 0.4
n = 60

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
