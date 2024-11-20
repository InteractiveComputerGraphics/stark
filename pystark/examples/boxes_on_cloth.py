# -*- coding: utf-8 -*-
import numpy as np
import pystark

settings = pystark.Settings()
settings.output.simulation_name = "boxes_on_cloth"
settings.output.output_directory = "output_folder"
settings.output.codegen_directory = "codegen_folder"
simulation = pystark.Simulation(settings)

# Contact
thickness = 0.00025
contact_params = pystark.EnergyFrictionalContact.GlobalParams()
contact_params.default_contact_thickness = thickness
contact_params.friction_stick_slide_threshold = 0.01
contact_params.min_contact_stiffness = 2e8
simulation.interactions().contact().set_global_params(contact_params)

# Floor
fV, fC, fH = simulation.presets().rigidbodies().add_box("floor", 1.0, np.array([1.0, 10.0, 0.05]))
fH.rigidbody.add_translation(np.array([0.0, 4.0, -0.05/2.0 - thickness]))
simulation.rigidbodies().add_constraint_fix(fH.rigidbody).set_stiffness(1e8)

# Geometrical description
s = 0.5  # Size [m]
n = 100   # Subdivisions per dimension (This is very hires!)

# Add cloth
cloth_material = pystark.Surface.Params.Cotton_Fabric()
cloth_material.strain.strain_limit_stiffness = 1e7
cloth_material.inertia.damping = 1.0
cloth_material.strain.damping = 1e1
cV, cC, cH = simulation.presets().deformables().add_surface_grid(
    "cloth",
    np.array([s, s]),  # size
    np.array([n, n]),  # end
    cloth_material
)
cH.point_set.add_translation(np.array([0.0, 0.0, 2.0*thickness]))
simulation.interactions().contact().set_friction(fH.contact, cH.contact, 0.5)

# Declare Prescribed Positions
cloth_bc = simulation.deformables().prescribed_positions().add_inside_aabb(
    cH.point_set,
    np.array([0.0, s/2.0, 0.0]), # AABB center
    np.array([s/2.0, 0.001, s]), # AABB dim
    pystark.EnergyPrescribedPositions.Params()
)

# Add boxes
mu = 0.5
s = 0.08
mass = 1.0
stack = 8
boxes = []
box_height = s + 2.0*thickness
box_base = s/2.0 + 3.0*thickness
for i in range(stack):
    for j in range(3):
        y = -1.0*s if j==1 else -1.5*s

        bV, bC, bH = simulation.presets().rigidbodies().add_box("boxes", mass, np.array([s, s, s]))
        bH.rigidbody.add_translation(np.array([-box_height/2.0 + j*box_height, y, box_base + i*box_height]))
        simulation.interactions().contact().set_friction(bH.contact, cH.contact, mu)
        simulation.interactions().contact().set_friction(bH.contact, fH.contact, mu)
        boxes.append(bH)

for i in range(len(boxes)):
    for j in range(i + 1, len(boxes)):
        simulation.interactions().contact().set_friction(boxes[i].contact, boxes[j].contact, mu)


# Script
lift_start = 1.0
pull_start = 2.0
end = 10.0
Vec3d_unitY = np.array([0.0, 1.0, 0.0])
Vec3d_unitZ = np.array([0.0, 0.0, 1.0])
simulation.add_time_event(0.0, lift_start, lambda t: simulation.set_gravity(Vec3d_unitZ*pystark.blend(0.0, -9.81, 0.0, lift_start, t, pystark.BlendType.Linear)))
simulation.add_time_event(lift_start, pull_start, lambda t: cloth_bc.set_transformation(Vec3d_unitZ*pystark.blend(0.0, 0.1, lift_start, pull_start, t, pystark.BlendType.EaseInOut)))
simulation.add_time_event(pull_start, end, lambda t: cloth_bc.set_transformation(Vec3d_unitZ*0.1 + Vec3d_unitY*pystark.blend(0.0, 1.0, pull_start, end, t, pystark.BlendType.Linear)))

# Run
simulation.run(end)
