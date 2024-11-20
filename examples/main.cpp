#include <iostream>

#include <stark>
#include "paths.h"
#include "rb_constraint_test_scenes.h"  // Contains a bunch of simple rigid body scenes with predictable outcomes



void hanging_net()
{
	/*
		Simulation of net of segments fixed by its perimeter and hanging due to gravity.
	*/

	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_net";
	settings.output.output_directory = OUTPUT_PATH + "/hanging_net";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.simulation.init_frictional_contact = false;
	stark::Simulation simulation(settings);

	// Geometry
	const int n = 20;
	const double d = 1.0;
	const double hd = d/2.0;
	auto [V, T] = stark::generate_triangle_grid({ 0.0, 0.0 }, { d, d }, { n, n });
	auto E = stark::find_edges_from_simplices(T, V.size());

	// Segment Net
	auto H = simulation.presets->deformables->add_line("segments", V, E, stark::Line::Params::Elastic_Rubberband());
	
	// BC
	auto bc1 = simulation.deformables->prescribed_positions->add_outside_aabb(H.point_set, { 0.0, 0.0, 0.0 }, { d - 0.001, d - 0.001, d - 0.001 }, stark::EnergyPrescribedPositions::Params());

	// Run
	simulation.run();
}
void hanging_cloth()
{
	/*
		Simulation of a piece of cloth fixed by two corners and hanging due to gravity.
	*/

	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_cloth";
	settings.output.output_directory = OUTPUT_PATH + "/hanging_cloth";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.simulation.init_frictional_contact = false;
	stark::Simulation simulation(settings);

	// Cloth
	const int n = 20;
	const double d = 1.0;
	const double hd = d/2.0;
	auto [V, T, H] = simulation.presets->deformables->add_surface_grid("cloth", { d, d }, { n, n }, stark::Surface::Params::Cotton_Fabric());

	// BC
	auto bc1 = simulation.deformables->prescribed_positions->add_inside_aabb(H.point_set, { hd, hd, 0.0 }, { 0.001, 0.001, 0.001 }, stark::EnergyPrescribedPositions::Params());
	auto bc2 = simulation.deformables->prescribed_positions->add_inside_aabb(H.point_set, { -hd, hd, 0.0 }, { 0.001, 0.001, 0.001 }, stark::EnergyPrescribedPositions::Params());

	// Run
	simulation.run();
}
void hanging_deformable_box()
{
	/*
		Simulation of a deformable box fixed by two corners and hanging due to gravity.
	*/

	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_deformable_box";
	settings.output.output_directory = OUTPUT_PATH + "/hanging_deformable_box";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.simulation.init_frictional_contact = false;
	stark::Simulation simulation(settings);

	// Box
	const int n = 10;
	const double d = 0.5;
	const double hd = d/2.0;
	auto [V, T, H] = simulation.presets->deformables->add_volume_grid("box", { d, d, d }, {n, n, n}, stark::Volume::Params::Soft_Rubber());
	
	// BC
	auto bc1 = simulation.deformables->prescribed_positions->add_inside_aabb(H.point_set, { hd, hd, hd }, { 0.001, 0.001, 0.001 }, stark::EnergyPrescribedPositions::Params().set_stiffness(1e7));
	auto bc2 = simulation.deformables->prescribed_positions->add_inside_aabb(H.point_set, { -hd, hd, hd }, { 0.001, 0.001, 0.001 }, stark::EnergyPrescribedPositions::Params().set_stiffness(1e7));

	// Run
	simulation.run();
}
void attachments()
{
	/*
		Simulation of two pieces of cloth and a rigid body attached by distance constraints and hanging due to gravity.
	*/

	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "attachments";
	settings.output.output_directory = OUTPUT_PATH + "/attachments";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.simulation.init_frictional_contact = false;
	stark::Simulation simulation(settings);

	// Cloth
	const int n = 20;
	const double d = 1.0;
	const double hd = d/2.0;
	const double gap = 0.001;
	auto params = stark::Surface::Params::Cotton_Fabric();
	auto [V1, T1, H1] = simulation.presets->deformables->add_surface_grid("A", { d, d }, { n, n }, params);
	auto [V2, T2, H2] = simulation.presets->deformables->add_surface_grid("B", { d, d }, { n, n }, params);
	H2.point_set.add_rotation(45.0, Eigen::Vector3d::UnitZ());
	H2.point_set.add_displacement({ d, 0.0, gap });

	// RB
	const double bs = 0.25;
	const stark::Mesh<3> box_mesh = stark::make_box({ bs, bs, bs });
	auto [V, C, box] = simulation.presets->rigidbodies->add_box("box", 0.1, bs);
	box.rigidbody.add_translation({ 1.7, 0.0, 0.5 * bs + 2.0 * gap });

	// Attachments
	auto att_cloth = simulation.interactions->attachments->add_by_distance(H2.point_set, H1.point_set, H2.point_set.all(), T1, 2.0*gap, 
		stark::EnergyAttachments::Params().set_tolerance(0.01)
	);
	auto att_rb = simulation.interactions->attachments->add_by_distance(box.rigidbody, H2.point_set, box_mesh.vertices, box_mesh.conn, H2.point_set.all(), 4.0*gap,
		stark::EnergyAttachments::Params().set_tolerance(0.01)
	);

	// BC
	auto bc1 = simulation.deformables->prescribed_positions->add_inside_aabb(H1.point_set, { -hd, -hd, 0.0 }, { 0.001, 0.001, 0.001 }, stark::EnergyPrescribedPositions::Params());
	auto bc2 = simulation.deformables->prescribed_positions->add_inside_aabb(H1.point_set, { -hd, hd, 0.0 }, { 0.001, 0.001, 0.001 }, stark::EnergyPrescribedPositions::Params());

	// Run
	simulation.run();
}
void hanging_box_with_composite_material()
{
	/*
		Simulation of a hanging box with composite material: Volumetric, shell and rod models for interior, surface and ridges.
		The box is fixed by two corners and hanging due to gravity.

		This scene exemplifies how to use individual energies to model different materials in a single deformable object,
		as opose to using a preset with a pre-selection.
	*/

	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_box_with_composite_material";
	settings.output.output_directory = OUTPUT_PATH + "/hanging_box_with_composite_material";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 8.0;
	settings.simulation.init_frictional_contact = false;
	stark::Simulation simulation(settings);

	// Geometry
	const int n = 10;
	const double d = 0.2;
	const double hd = d/2.0;
	auto [vertices, tets] = stark::generate_tet_grid({ 0.0, 0.0, 0.0 }, { d, d, d }, { n, n, n });

	// Meshes
	auto [triangles, tri_tet_map] = stark::find_surface(vertices, tets);
	auto tri_vertices = stark::gather(vertices, tri_tet_map);
	auto triangles_in_tet_connectivity = stark::apply_map(triangles, tri_tet_map);
	auto [sharp_edges, edge_tri_map] = stark::find_sharp_edges(tri_vertices, triangles, 30.0);
	std::vector<int> edge_tet_map = stark::gather(tri_tet_map, edge_tri_map);
	auto edges_in_tet_connectivity = stark::apply_map(sharp_edges, edge_tet_map);

	// Node set
	const stark::PointSetHandler nodeset = simulation.deformables->point_sets->add(vertices)
		.add_rotation(-90.0, Eigen::Vector3d::UnitX());

	// Energies
	auto inertia = simulation.deformables->lumped_inertia->add(nodeset, tets, 
		stark::EnergyLumpedInertia::Params()
		.set_density(1000.0)
		.set_damping(0.5)
	);
	auto tet_strain = simulation.deformables->tet_strain->add(nodeset, tets, 
		stark::EnergyTetStrain::Params()
		.set_youngs_modulus(1e3)
	);
	auto segment_strain = simulation.deformables->segment_strain->add(nodeset, edges_in_tet_connectivity,
		stark::EnergySegmentStrain::Params()
		.set_section_radius(5e-3)
		.set_youngs_modulus(5e5)
		);
	auto triangle_strain = simulation.deformables->triangle_strain->add(nodeset, triangles_in_tet_connectivity,
		stark::EnergyTriangleStrain::Params()
		.set_youngs_modulus(1e4)
		.set_strain_limit(0.2)
		.set_strain_limit_stiffness(100.0)
		);
	auto discrete_shells = simulation.deformables->discrete_shells->add(nodeset, triangles_in_tet_connectivity,
		stark::EnergyDiscreteShells::Params()
		.set_stiffness(2e-3)
		.set_flat_rest_angle(true)
		);
	auto bc = simulation.deformables->prescribed_positions->add_inside_aabb(nodeset, { hd, hd, hd }, { 0.001, 0.001, 0.001 },
		stark::EnergyPrescribedPositions::Params()
		.set_stiffness(1e7)
		.set_tolerance(1e-3)
	);
	simulation.deformables->prescribed_positions->add_inside_aabb(nodeset, { -hd, hd, hd }, { 0.001, 0.001, 0.001 },
		stark::EnergyPrescribedPositions::Params()
		.set_stiffness(1e7)
		.set_tolerance(1e-3)
	);

	// Output
	simulation.deformables->output->add_tet_mesh("tets", nodeset, tets);
	simulation.deformables->output->add_triangle_mesh("triangles", nodeset, triangles, tri_tet_map);
	simulation.deformables->output->add_segment_mesh("segments", nodeset, sharp_edges, edge_tet_map);
	simulation.deformables->output->add_point_set("points", nodeset);

	// Run
	simulation.run();
}
void deformable_and_rigid_collisions()
{
	/*
		Simulation of two stacked deformable boxes of different density and stiffness laying on a rigid, fixed, floor.
	*/

	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "deformable_and_rigid_collisions";
	settings.output.output_directory = OUTPUT_PATH + "/deformable_and_rigid_collisions";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	stark::Simulation simulation(settings);

	// Contact
	simulation.interactions->contact->set_global_params(
		stark::EnergyFrictionalContact::GlobalParams()
			.set_friction_stick_slide_threshold(0.01)
			.set_min_contact_stiffness(1e8)
	);

	// Geometry
	int n1 = 5; // subdivisions box 1
	double d1 = 0.25; // size box 1
	int n2 = 2; // subdivisions box 2
	double d2 = 0.1; // size box 2
	double gap = 0.01; // gap between boxes

	// Deformables
	stark::Volume::Params rubber = stark::Volume::Params::Soft_Rubber();
	
	rubber.contact.contact_thickness = 0.001*d1;
	rubber.inertia.density = 1e3;
	auto [V1, T1, H1] = simulation.presets->deformables->add_volume_grid("boxes", { d1, d1, d1 }, { n1, n1, n1 }, rubber);
	H1.point_set.add_displacement({ 0.0, 0.0, 0.5*d1 + gap });

	rubber.contact.contact_thickness = 0.001*d2;
	rubber.inertia.density = 1e4;
	rubber.strain.youngs_modulus = 1e5;
	auto [V2, T2, H2] = simulation.presets->deformables->add_volume_grid("boxes", { d2, d2, d2 }, { n2, n2, n2 }, rubber);
	H2.point_set.add_displacement({ 0.13*d2 , 0.07*d2, d1 + 0.5*d2 + 2*gap});

	// Rigid
	double d3 = 2.0;
	double contact_distance_floor = gap;
	auto [V, C, floor] = simulation.presets->rigidbodies->add_box("floor", 1.0, { d3, d3, 0.05*d3 }, stark::ContactParams().set_contact_thickness(0.001*d3));
	floor.rigidbody.set_translation({ 0.0, 0.0, -0.025*d3 });
	simulation.rigidbodies->add_constraint_fix(floor.rigidbody);

	// Contact
	double mu = 1.0;
	simulation.interactions->contact->set_friction(floor.contact, H1.contact, mu);
	simulation.interactions->contact->set_friction(floor.contact, H2.contact, mu);
	simulation.interactions->contact->set_friction(H1.contact, H2.contact, mu);

	// Run
	simulation.run();
}
void simple_grasp()
{
	/*
		Simulation of a simple two-finger parallel gripper grasping a deformable object.
		Sequence:
			1. In absence of gravity, the fingers close and squeeze the object.
			2. Gravity is progressively turned on. The object stays between the fingers due to Coulomb friction coefficient slightly larger than the sticking threshold.
			3. The friction coefficient is reduced to slightly below the sticking threshold. The object slides between the fingers.

		This scene also shows how to anonymous lambdas to scope the script code, which is an option to avoid global variables.
	*/

	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "simple_grasp";
	settings.output.output_directory = OUTPUT_PATH + "/simple_grasp";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 7.0;
	settings.simulation.gravity = { 0.0, 0.0, 0.0 };
	stark::Simulation simulation(settings);

	// Parameters
	int n = 5;
	double d = 0.2;
	double hd = d/2.0;
	double gap = 0.02;
	double contact_thickness = 0.001;
	double mass = 1.0;
	double gravity = -10.0;
	double pressure = 10.0; // [N]
	double mu_sticking = 1.05;
	double mu_sliding = 0.95;

	// Contact
	simulation.interactions->contact->set_global_params(
		stark::EnergyFrictionalContact::GlobalParams()
		.set_default_contact_thickness(contact_thickness)
		.set_friction_stick_slide_threshold(0.001) // Tighther threshold for more accurate frictional forces
		.set_min_contact_stiffness(1e7)
	);

	// Object
	auto [obj, obj_c] = [&]()
		{
		auto params = stark::Volume::Params::Soft_Rubber();
		params.inertia.density = mass/std::pow(d, 3);
		params.strain.elasticity_only = true;  // We don't need material damping nor strain limiting for this scene
		params.strain.youngs_modulus = 2e3;
		auto [V, T, H] = simulation.presets->deformables->add_volume_grid("deformable", { d, d, d }, { n, n, n }, params);
		return std::make_tuple( H, H.contact);
	}();

	// Hand
	auto [hand, hand_c] = [&]()
	{
		auto [V, C, H] = simulation.presets->rigidbodies->add_box("hand", mass, { 3*d, 3*d, 3*d });
		H.rigidbody.set_translation({ 0.0, -(3 * hd + hd + gap), 0.0 });
		return std::make_tuple(H.rigidbody, H.contact);
	}();

	// Fingers
	const Eigen::Vector3d fingers_size = { 0.5*d, 2*d, 2*d };
	auto [left_finger, left_finger_c] = [&]()
	{
		auto [V, C, H] = simulation.presets->rigidbodies->add_box("hand", mass, fingers_size);
		H.rigidbody.set_translation({ -(hd + 0.5 * hd + gap), -gap, 0.0 });
		return std::make_tuple(H.rigidbody, H.contact);
	}();
	auto [right_finger, right_finger_c] = [&]()
	{
		auto [V, C, H] = simulation.presets->rigidbodies->add_box("hand", mass, fingers_size);
		H.rigidbody.set_translation({ (hd + 0.5 * hd + gap), -gap, 0.0 });
		return std::make_tuple(H.rigidbody, H.contact);
	}();

	// Disable collisions
	simulation.interactions->contact->disable_collision(hand_c, left_finger_c);
	simulation.interactions->contact->disable_collision(hand_c, right_finger_c);

	// BC
	const Eigen::Vector3d p = Eigen::Vector3d::Zero();
	const Eigen::Vector3d dir = Eigen::Vector3d::UnitX();

	simulation.rigidbodies->add_constraint_fix(hand);
	simulation.rigidbodies->add_constraint_prismatic_press(hand, left_finger, p, dir, 1.0, 0.5*pressure);
	simulation.rigidbodies->add_constraint_prismatic_press(hand, right_finger, p, dir, -1.0, 0.5*pressure);

	// Friction
	simulation.interactions->contact->set_friction(left_finger_c, obj_c, mu_sticking);
	simulation.interactions->contact->set_friction(right_finger_c, obj_c, mu_sticking);

	// Script
	simulation.add_time_event(2.0, 3.0, 
		[&](double t) 
		{ 
			const double gz = stark::blend(0.0, gravity, 2.0, 3.0, t, stark::BlendType::Linear);
			simulation.set_gravity({ 0.0, 0.0, gz }); 
		}
	);
	simulation.add_time_event(5.0, 7.0, 
		[&](double t) 
		{ 
			simulation.interactions->contact->set_friction(left_finger_c, obj_c, mu_sliding);
			simulation.interactions->contact->set_friction(right_finger_c, obj_c, mu_sliding);
		}
	);

	// Run
	simulation.run();
}
void twisting_cloth()
{
	/*
		Simulation of a cloth twisted by rotating two sides in opposite directions in absence of gravity.
		This scene shows how to use boundary condition transformations together with time events to animate the simulation.
		
		Also interesting to note in this simulation is that we can use a high acceleration residual value in Newton's Method to avoid excessive Newton iterations.
		In simulations when objects might fall due to gravity, the acceleration residual must be set to a lower value than the gravity itself to avoid excessive 
		numerical damping or even the objects not falling at all. Since in this simulation there is no gravity involved, we can use a higher residual to obtain 
		a faster simulation while preserving fidelity.
	*/

	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "twisting_cloth";
	settings.output.output_directory = OUTPUT_PATH + "/twisting_cloth";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.simulation.gravity = { 0.0, 0.0, 0.0 };
	settings.newton.residual = { stark::ResidualType::Acceleration, 100.0 };
	stark::Simulation simulation(settings);

	// Contact
	simulation.interactions->contact->set_global_params(
		stark::EnergyFrictionalContact::GlobalParams()
		.set_default_contact_thickness(0.0005)
		.set_min_contact_stiffness(1e6)
	);
	
	// Cloth
	double s = 0.5;
	int n = 50;
	stark::Surface::Params material = stark::Surface::Params::Cotton_Fabric();
	material.strain.elasticity_only = true;  // Strain limiting would make the cloth too stiff and would fight with the prescribed BC, leading to unrealistic stresses
	auto [V, T, H] = simulation.presets->deformables->add_surface_grid("cloth", { s, s }, { n, n }, material);
	H.point_set.add_rotation(90.0, Eigen::Vector3d::UnitX());
	H.contact.set_friction(H.contact, 1.0);

	// BC
	auto bc_params = stark::EnergyPrescribedPositions::Params();
	auto left = simulation.deformables->prescribed_positions->add_inside_aabb(H.point_set, { -s/2.0, 0.0, 0.0 }, { 0.001, s, s }, bc_params);
	auto right = simulation.deformables->prescribed_positions->add_inside_aabb(H.point_set, { s/2.0, 0.0, 0.0 }, { 0.001, s, s }, bc_params);

	// Script
	double duration = 5.0;
	double angular_velocity = 90.0;  // [deg / s]
	simulation.add_time_event(0, duration, [&](double t) { left.set_transformation(Eigen::Vector3d::Zero(), angular_velocity * t, Eigen::Vector3d::UnitX()); });
	simulation.add_time_event(0, duration, [&](double t) { right.set_transformation(Eigen::Vector3d::Zero(), -angular_velocity * t, Eigen::Vector3d::UnitX()); });

	// Run
	simulation.run();
}
void magnetic_deformables()
{
	/*
		Simulation of a stack of deformable objects attracted by a magnet scripted to move up and down.
	*/

	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "magnetic_deformables";
	settings.output.output_directory = OUTPUT_PATH + "/magnetic_deformables";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 7.0;
	stark::Simulation simulation(settings);

	// Contact
	simulation.interactions->contact->set_global_params(
		stark::EnergyFrictionalContact::GlobalParams()
		.set_default_contact_thickness(0.002)
		.set_min_contact_stiffness(1e7)
	);

	// Add floor
	auto [floor_vertices, floor_triangles, floor] = simulation.presets->rigidbodies->add_box("floor", 1.0, { 3.0, 3.0, 0.01 });
	floor.rigidbody.add_translation({ 0.0, 0.0, -0.01 });
	simulation.rigidbodies->add_constraint_fix(floor.rigidbody);

	// Add objects
	std::vector<stark::Volume::Handler> objs;
	{
		double size = 0.1;
		int n = 2;
		std::array<int, 3> grid = { 4, 4, 4 };
		double spacing = size * 1.8;
		double height = 0.5;
		Eigen::Vector3d center = { 0.5*(grid[0]-1)*spacing, 0.5*(grid[1]-1)*spacing, 0.5*(grid[2]-1)*spacing };
		auto material = stark::Volume::Params::Soft_Rubber();
		material.strain.youngs_modulus = 2e4;
		for (int i = 0; i < grid[0]; i++) {
			for (int j = 0; j < grid[1]; j++) {
				for (int k = 0; k < grid[2]; k++) {
					auto [V, C, obj] = simulation.presets->deformables->add_volume_grid("objects", { size, size, size }, { n, n, n }, material);
					obj.point_set.add_rotation(Eigen::Vector3d::Random().x() * 90.0, Eigen::Vector3d::Random());
					obj.point_set.add_displacement({ i * spacing - center.x(), j * spacing - center.y(), k * spacing - center.z() + height});
					objs.push_back(obj);
				}
			}
		}
	}

	// Magnet
	const double magnet_height = 1.5;
	auto [magnet_vertices, magnet_triangles, magnet] = simulation.presets->rigidbodies->add_sphere("magnet", 1.0, 0.2, 3);
	magnet.rigidbody.add_translation({ 0.0, 0.0, magnet_height });
	auto magnet_fix = simulation.rigidbodies->add_constraint_fix(magnet.rigidbody);

	// Friction
	double friction = 0.5;
	for (int i = 0; i < (int)objs.size(); i++) {
		simulation.interactions->contact->set_friction(objs[i].contact, magnet.contact, friction);
		simulation.interactions->contact->set_friction(objs[i].contact, floor.contact, friction);
		for (int j = i + 1; j < (int)objs.size(); j++) {
			simulation.interactions->contact->set_friction(objs[i].contact, objs[j].contact, friction);
		}
	}

	// Magnet script
	simulation.add_time_event(1.5, 3.0,
		[&](double t) {
			double height = stark::blend(magnet_height, 0.5, 1.5, 3.0, t, stark::BlendType::Linear);
			magnet_fix.set_transformation({ 0.0, 0.0, height }, Eigen::Matrix3d::Identity());
		}
	);
	simulation.add_time_event(4.5, 6.0,
		[&](double t) {
			double height = stark::blend(0.5, magnet_height, 4.5, 6.0, t, stark::BlendType::Linear);
			magnet_fix.set_transformation({ 0.0, 0.0, height }, Eigen::Matrix3d::Identity());
		}
	);

	// Run with a callback function that is executed every time step (same than a time event but without time bounds)
	simulation.run(
		[&]()
		{
			double magnet_force = 0.1;
			const Eigen::Vector3d magnet_center = magnet.rigidbody.get_translation();
			for (auto& obj : objs) {
				for (int vertex_idx = 0; vertex_idx < (int)obj.point_set.size(); vertex_idx++) {
					const Eigen::Vector3d vertex = obj.point_set.get_position(vertex_idx);
					const Eigen::Vector3d u = magnet_center - vertex;
					const double d = u.norm();
					const double force = magnet_force / (d * d);
					obj.point_set.set_force(vertex_idx, force * u.normalized());
				}
			}
		}
	);
}

int main()
{
	/*
		Here you can find a list of simple scenes to test the library.
		Each function contains a different scene with a brief description of the simulation.
		To run a scene, simply comment everything else and call the desired function.

		Note that STARK can handle much more complex simulations than the ones presented here, 
		these are just simple examples that don't require external assets to get you started.
	*/

	// Simple rigid body scenes
	rb_constraints_all();

	// Simple simulations: No collisions, only presets
	hanging_net();
	hanging_cloth();
	hanging_deformable_box();
	attachments();

	// Composite materials
	hanging_box_with_composite_material();

	// Simulations with collisions
	deformable_and_rigid_collisions();
	simple_grasp(); 
	twisting_cloth();
	magnetic_deformables();
}
