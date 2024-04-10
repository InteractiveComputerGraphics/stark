# General imports
import os
import sys
import numpy as np

##############################################################################
# DATA IN THE UI
##############################################################################
class GlobalParameters:
    def __init__(self):
        self.installation_directory = "C:/StarkFX"  # Contains the `pystark.cp310-win_amd64.pyd` and all the precompiled SymX DLLs
        self.simulation_directory = "C:/Users/josea/Desktop/user_ouput_directory"  # Output meshes and loggers will be written and read here
        self.fps = 30  # TODO: Take this from the Blender UI. Still display it here so the user is aware? Or maybe if changed here is also changed for the animation panel.
        self.number_of_threads = os.cpu_count()
        
        self.time_step_size_in_ms = 10.0  # [milliseconds] Maximum time step size. The actual time step size could be shorter due to adaptivity.
        self.acceleration_tolerance = 1.0  # [m/s^2] Maximum acceleration residual allowed to be left unbalanced. To preserve inertia, this should be almost always lower than gravity (9.81).
        self.default_contact_thickness_in_mm = 1.0  # [millimieters] Every new object declared will have this by default, but it can be changed on a per-object basis
        self.friction_sticking_tolerance = 0.1  # [m/s] Threshold between sticking and sliding. Lower is more accurate, but slower to compute.
        self.enable_contacts = True  # If False, frictional contact facilities are not loaded for the simulation, which should save time

        # The simulation will successfully stop when the first of the following criteria is reached
        self.run_until_frame = 250  # TODO: This should get the max frame set in the Blender UI
        self.run_until_simulation_time = np.inf
        self.run_until_execution_time = np.inf

        self.gravity_x = 0.0  # [m/s^2]
        self.gravity_y = 0.0  # [m/s^2]
        self.gravity_z = -9.81  # [m/s^2]


class DeformableSurfaceParametrization:
    # Note: There will be material presets (Cotton, Silk, rubber, aluminum foil, ...) but in any case, these are all the parameters
    # The parametrization here corresponds to Cotton
    
    def __init__(self):
        self.thickness_in_mm = GlobalParameters().default_contact_thickness_in_mm  # Thickness used to begin contact exerting contact forces
        self.inertia_density = 0.2  # [kg/m^2]
        self.inertia_damping = 0.1  # Slows down any movements
        self.scale = 1.0  # Can shrink or expand the object

        self.elasticity_only = False  # If True, strain damping, strain limiting and bending damping will not be loaded. This will make cloth unrealistically rubbery but much faster to solve.

        #strain_thickness = 0.001  # [m] Jose: Let's not show this one. It is only used to compute the volume integral for the strain. It is confusing.
        self.strain_youngs_modulus = 1e3  # [Pa] Stiffness to resist stretching and shearing
        self.strain_poissons_ratio = 0.3  # Stiffness to resist changes in area
        self.strain_strain_limit_in_percentage = 10.0  # [%] Elongation beyond which an extra resistence will be applied
        self.strain_strain_limit_stiffness = 1e6  # Stiffness of the extra strain resistance for the strain limit
        self.strain_damping = 1e-4*self.strain_youngs_modulus  # Slows down in-plane deformations. High values achieve viscoelastic effects

        self.bending_flat_rest_angle = True  # If True, the surface will tend to deform to a flat configuration. If False, it will tend to deform back to the rest configuration (first frame).
        self.bending_stiffness = 1e-6  # Stiffness to resist bending
        self.bending_damping = 0.1*self.bending_stiffness  # Slows down deformations resulting from bending. High values achieve viscoelastic effects

        self.inflation = 0.0  # [N/m2] Perpendicular force to the surface


##############################################################################
# USER DECLARATION
##############################################################################
# This mimics that the user has changed some things in the panel
global_parameters = GlobalParameters()
global_parameters.run_until_frame = 100  # Just run 100 frames

defomable_surfaces_ui_params = [DeformableSurfaceParametrization(), DeformableSurfaceParametrization()]
defomable_surfaces_ui_params[0].elasticity_only = True  # First surface just elastic
defomable_surfaces_ui_params[1].bending_stiffness = 1e-3  # Second surface stiffer to bend. Like a shell.

deformable_surface_labels = ["surf_0", "surf_1"]

# At this point, `global_parameters` and `defomable_surfaces` contain all the data that was in the panel when the user hits "Run"


##############################################################################
# pystark code trigger upon "Run"
##############################################################################

# import pystark
if not global_parameters.installation_directory in sys.path:
    sys.path.append(global_parameters.installation_directory)
import pystark

# Set up global parameters. We need a correctly populated `Settings` class to start an instance of Stark
settings = pystark.Settings()

## Settings exposed in StarkFX
settings.output.output_directory = global_parameters.simulation_directory
settings.output.codegen_directory = global_parameters.installation_directory
settings.output.fps = global_parameters.fps
settings.execution.n_threads = global_parameters.number_of_threads
settings.execution.end_frame = global_parameters.run_until_frame
settings.execution.end_simulation_time = global_parameters.run_until_simulation_time
settings.execution.allowed_execution_time = global_parameters.run_until_execution_time
settings.simulation.max_time_step_size = global_parameters.time_step_size_in_ms/1000.0
settings.simulation.init_frictional_contact = global_parameters.enable_contacts
settings.simulation.gravity = np.array([global_parameters.gravity_x, global_parameters.gravity_z, global_parameters.gravity_z])
settings.newton.residual.tolerance = global_parameters.acceleration_tolerance

## Default parameters for StarkFX
settings.output.simulation_name = "starkfx"  # Every output file will be prepended by this string. This is mandatory for Stark but the user doesn't need to care. This can't be empty.
settings.output.console_output_to = pystark.ConsoleOutputTo.FileAndConsole  # TODO: Do not show Stark console output in the blender console. For now let's show it as well.
settings.output.enable_output = True  # TODO: This will be `False` as we should only read and write vertex positions as binary data. For now let's save the .vtk files.
settings.debug.symx_force_load = True  # Do not attempt to check if the DLLs are correct. The user can't recompile them anyway.

### Notes on global parameters:
###  - `default_contact_thickness_in_mm` is not really used as a global parameter. We read it directly per object. It is just to make life easier for the user.
###  - There are many more settings in Stark. They are not crucial for this stage

# Initialize an instance of Stark
simulation = pystark.Simulation(settings)

# Frictional contact global parameters
contact_params = pystark.EnergyFrictionalContact.GlobalParams()
contact_params.friction_stick_slide_threshold = global_parameters.friction_sticking_tolerance
simulation.interactions().contact().set_global_params(contact_params)

# Add deformable surfaces
## This function will be useful at creation but also to update the cloth properties in case the user has scripted value changes
def get_deformable_surface_params(ui_params: DeformableSurfaceParametrization):
    """
    Generates a pystark deformable surface parametrization from the data in the addon
    """
    params = pystark.Surface.Params()

    params.inertia.density = ui_params.inertia_density
    params.inertia.damping = ui_params.inertia_damping
    
    params.strain.scale = ui_params.scale
    params.strain.elasticity_only = ui_params.elasticity_only
    params.strain.thickness = 0.001  # Hardcoded. Only used for the volume integral. Nothing to do with contacts.
    params.strain.youngs_modulus = ui_params.strain_youngs_modulus
    params.strain.poissons_ratio = ui_params.strain_poissons_ratio
    params.strain.damping = ui_params.strain_damping
    params.strain.strain_limit = ui_params.strain_strain_limit_in_percentage/100.0  # In stark this is a fraction not a percentage (10% -> 0.1)
    params.strain.strain_limit_stiffness = ui_params.strain_strain_limit_stiffness
    params.strain.inflation = ui_params.inflation

    params.bending.scale = ui_params.scale
    params.bending.elasticity_only = ui_params.elasticity_only
    params.bending.flat_rest_angle = ui_params.bending_flat_rest_angle
    params.bending.stiffness = ui_params.bending_stiffness
    params.bending.damping = ui_params.bending_damping

    params.contact.contact_thickness = ui_params.thickness_in_mm/1000.0

    return params

## Add the deformable surfaces in Stark
deformable_surfaces_stark_handlers = []
for surf_i in range(len(defomable_surfaces_ui_params)):
    ui_params = defomable_surfaces_ui_params[surf_i]
    label = deformable_surface_labels[surf_i]

    # Here is where the mesh would be triangulated from a user mesh, but as placeholder I will just generate one
    mesh = pystark.generate_triangle_grid(np.array([2.0*surf_i, 0.0]), np.array([1.0, 1.0]), np.array([10, 10]))

    pystark_params = get_deformable_surface_params(ui_params)
    handler = simulation.presets().deformables().add_surface(label, mesh.vertices, mesh.triangles, pystark_params)
    deformable_surfaces_stark_handlers.append(handler)


# Create callback to update all the parameters from the UI at the beginning of each time step
def callback():
    
    # Update parameters from UI
    for surf_i in range(len(defomable_surfaces_ui_params)):
        ui_params = defomable_surfaces_ui_params[surf_i]
        handler = deformable_surfaces_stark_handlers[surf_i]
        pystark_params = get_deformable_surface_params(ui_params)
        
        # Note this will be refactor into one single call
        handler.inertia.set_params(pystark_params.inertia)
        handler.strain.set_params(pystark_params.strain)
        handler.bending.set_params(pystark_params.bending)
        handler.contact.set_contact_thickness(pystark_params.contact.contact_thickness)


# Run simulation
simulation.run(callback)
