# -*- coding: utf-8 -*-
import pystark_paths
import sys
import os
stubgen_path = "pybind11-stubgen"

# Build stubs
if not os.path.isdir(pystark_paths.bin_dir):
    print("Error: bin_folder does not exist")
    sys.exit(1)
else:
    #command = "cd %s & $env:PYTHONPATH = \"$env:PYTHONPATH;%s\" & %s pystark -o %s & cd %s" % (bin_folder, stubgen_path, cwd, cwd)
    print("Don't forget to -> $env:PYTHONPATH = \"$env:PYTHONPATH;%s\"" % pystark_paths.bin_dir)
    command = "%s pystark -o \".\"" % stubgen_path
    print(command)
    os.system(command)
    sys.exit(0)

# Add bin_folder to path
sys.path.append(pystark_paths.bin_dir)
# =================================================================================================================

import numpy as np
import pystark

settings = pystark.Settings()
settings.output.simulation_name = "pystark_0"
settings.output.output_directory = build_dir + "/output/" + settings.output.simulation_name
settings.output.codegen_directory = build_dir + "/codegen"
settings.execution.end_simulation_time = 0.5
settings.contact.collisions_enabled = False
settings.output.console_verbosity = pystark.ConsoleVerbosity.Frames
# settings.simulation.gravity = np.array([0.0, 0.0, 0.0])
simulation = pystark.Simulation(settings)


# x = np.random.rand(3, 3)
# pH = simulation.deformables().point_sets().add(x)  # Here the handler works
# points = pH.all()
# mass = np.ones(x.shape[0], np.float64)
# params = pystark.EnergyLumpedInertia.Params().set_density(10.0)
# E_inertia = simulation.deformables().lumped_inertia().add(pH, points, mass, params)
# print(E_inertia.get_idx())
# print(E_inertia.get_mass())


print("0")
dim = np.array([1.0, 1.0, 1.0])
subdivisions = np.array([1, 1, 1])
params = pystark.Volume.Params.Soft_Rubber()
label = "box"

print("1")

v, c, h = simulation.deformables().add_volume_grid(dim, subdivisions, params, label)
vch = simulation.deformables().add_volume_grid(dim, subdivisions, params, label)

simulation.deformables().prescribed_positions().add_inside_aabb(
    h.point_set, 
    np.array([0.0, -0.5, 0.0]), 
    np.array([1.0, 0.001, 1.0]), 
    pystark.EnergyPrescribedPositions.Params().set_tolerance(0.001)
    )

# print(vch.vertices)
# print(vch.tets)
# print(vch.handler)
# print(v)
# print(c)
# print(h)
print("2")

# simulation.run()

# sys.exit(0)

def script(t):
    vch.handler.strain.set_params(vch.handler.strain.get_params().set_scale(1.0 - t))
    h.strain.set_params(h.strain.get_params().set_scale(1.0 + t))

simulation.run(lambda : script(simulation.get_time()))
#simulation.run(lambda : print("t: ", simulation.get_time()))
# simulation.run_one_time_step()