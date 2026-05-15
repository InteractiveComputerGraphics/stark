import pystark
import numpy as np

def run_test_sim():
    # Create simulation
    settings = pystark.Settings()
    settings.output.simulation_name = "test_hanging_cloth"
    settings.output.output_directory = "output"
    settings.output.codegen_directory = "codegen"
    settings.simulation.init_frictional_contact = False
    simulation = pystark.Simulation(settings)

    # Dimensions
    s = 0.5
    n = 32

    # Add deformable surface
    material = pystark.Surface.Params.Cotton_Fabric()
    cV, cT, cH = simulation.presets().deformables().add_surface_grid("cloth",
        size=np.array([s, s]),
        subdivisions=np.array([n, n], dtype=np.int32),
        params=material
    )

    # Prescribed positions
    simulation.deformables().prescribed_positions().add_inside_aabb(cH.point_set, np.array([0.5*s, 0.5*s, 0.0]), 0.001*pystark.ONES, pystark.EnergyPrescribedPositions.Params())
    simulation.deformables().prescribed_positions().add_inside_aabb(cH.point_set, np.array([0.5*s, -0.5*s, 0.0]), 0.001*pystark.ONES, pystark.EnergyPrescribedPositions.Params())

    # Script
    duration = 1.0
    simulation.run(duration)
