#include <iostream>

#include <stark>

#include "paths.h"

void hanging_composite_box()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_composite_box";
	settings.output.output_directory = OUTPUT_PATH + "/hanging_composite_box";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.debug.symx_check_for_NaNs = true;
	stark::Simulation simulation(settings);

	// Geometry
	const int n = 7;
	const double d = 0.2;
	const double hd = d/2.0;
	auto [vertices, tets] = stark::utils::generate_tet_grid({ 0.0, 0.0, 0.0 }, { d, d, d }, { n, n, n });

	// Meshes
	auto [triangles, tri_tet_map] = stark::utils::extract_surface(vertices, tets);
	auto tri_vertices = stark::utils::gather(vertices, tri_tet_map);
	auto triangles_in_tet_connectivity = stark::utils::apply_map(triangles, tri_tet_map);
	auto [sharp_edges, edge_tri_map] = stark::utils::find_sharp_edges(tri_vertices, triangles, 30.0);
	std::vector<int> edge_tet_map = stark::utils::gather(tri_tet_map, edge_tri_map);
	auto edges_in_tet_connectivity = stark::utils::apply_map(sharp_edges, edge_tet_map);

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
		.set_youngs_modulus(1e5)
		);
	auto triangle_strain = simulation.deformables->triangle_strain->add(nodeset, triangles_in_tet_connectivity,
		stark::EnergyTriangleStrain::Params()
		.set_youngs_modulus(1e4)
		.set_strain_limit(0.5)
		.set_strain_limit_stiffness(100.0)
		);
	auto discrete_shells = simulation.deformables->discrete_shells->add(nodeset, triangles_in_tet_connectivity,
		stark::EnergyDiscreteShells::Params()
		.set_stiffness(1e-3)
		.set_flat_rest_angle(true)
		);
	auto bc = simulation.deformables->prescribed_positions->add_inside_aabb(nodeset, { 0, hd, hd }, { hd, 0.001, 0.001 },
		stark::EnergyPrescribedPositions::Params()
		.set_tolerance(1e-2)
	);

	// Output
	simulation.deformables->output->add_tet_mesh("tets", nodeset, tets);
	simulation.deformables->output->add_triangle_mesh("triangles", nodeset, triangles, tri_tet_map);
	simulation.deformables->output->add_segment_mesh("segments", nodeset, sharp_edges, edge_tet_map);
	simulation.deformables->output->add_point_set("points", nodeset);

	// Run
	simulation.run(
		[&]()
		{
			//const double t = simulation.get_time() / 5.0;
			//bc.set_transformation({0.0, 0.0, t});
		}
	);
}
void hanging_net()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_net";
	settings.output.output_directory = OUTPUT_PATH + "/hanging_net";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.debug.symx_check_for_NaNs = true;
	settings.simulation.init_frictional_contact = false;
	stark::Simulation simulation(settings);

	// Geometry
	const int n = 20;
	const double d = 1.0;
	const double hd = d/2.0;
	auto [V, T] = stark::utils::generate_triangle_grid({ 0.0, 0.0 }, { d, d }, { n, n });
	auto E = stark::utils::find_edges_from_simplices(T, V.size());

	// Segment Net
	auto H = simulation.presets->deformables->add_line("segments", V, E, stark::Line::Params::Elastic_Rubberband());
	
	// BC
	auto bc1 = simulation.deformables->prescribed_positions->add_outside_aabb(H.point_set, { 0.0, 0.0, 0.0 }, { d - 0.001, d - 0.001, d - 0.001 }, stark::EnergyPrescribedPositions::Params());

	// Run
	simulation.run();
}
void hanging_cloth()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_cloth";
	settings.output.output_directory = OUTPUT_PATH + "/hanging_cloth";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.debug.symx_check_for_NaNs = true;
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
void hanging_box()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "hanging_box";
	settings.output.output_directory = OUTPUT_PATH + "/hanging_box";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.debug.symx_check_for_NaNs = true;
	settings.simulation.init_frictional_contact = false;
	stark::Simulation simulation(settings);

	// Box
	const int n = 10;
	const double d = 0.5;
	const double hd = d/2.0;
	auto [V, T, H] = simulation.presets->deformables->add_volume_grid("surf", { d, d, d }, {n, n, n}, stark::Volume::Params::Soft_Rubber());
	
	// BC
	auto bc1 = simulation.deformables->prescribed_positions->add_inside_aabb(H.point_set, { hd, hd, hd }, { 0.001, 0.001, 0.001 }, stark::EnergyPrescribedPositions::Params());
	auto bc2 = simulation.deformables->prescribed_positions->add_inside_aabb(H.point_set, { -hd, hd, hd }, { 0.001, 0.001, 0.001 }, stark::EnergyPrescribedPositions::Params());

	// Run
	simulation.run();
}
void ball_joints()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "ball_joints";
	settings.output.output_directory = OUTPUT_PATH + "/ball_joints";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.debug.symx_check_for_NaNs = true;
	settings.simulation.init_frictional_contact = false;
	stark::Simulation simulation(settings);

	// Geometry
	const double size = 0.1;
	auto mesh = stark::utils::make_box({ size, size, size });

	// Add Objects
	const int n = 10;
	const double mass = 1.0;

	std::vector<stark::RigidBodyHandler> rbs;
	for (int i = 0; i < n; i++) {
		// Add RB
		rbs.push_back( 
			simulation.rigidbodies->add(mass, stark::inertia_tensor_box(mass, { size, size, size })) 
				.add_translation({ 0.0, i * size, 0.0 })
		);

		// Declare output
		simulation.rigidbodies->output.add_triangle_mesh("boxes", rbs.back(), mesh.vertices, mesh.conn);
	}

	// Boundary Conditions
	const Eigen::Vector3d bottom_corner = { -0.5 * size, -0.5 * size, -0.5 * size };
	const Eigen::Vector3d top_corner = { +0.5 * size, -0.5 * size, +0.5 * size };

	//// Pin the first box
	simulation.rigidbodies->add_constraint_global_point(rbs[0], bottom_corner);

	//// Ball joints
	for (int i = 1; i < n; i++) {
		if (i % 2) {
			simulation.rigidbodies->add_constraint_point(rbs[i - 1], rbs[i], rbs[i].transform_local_to_global_point(top_corner) );
		}
		else {
			simulation.rigidbodies->add_constraint_point(rbs[i - 1], rbs[i], rbs[i].transform_local_to_global_point(bottom_corner) );
		}
	}

	// Run
	simulation.run();
}
void attachments()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "attachments";
	settings.output.output_directory = OUTPUT_PATH + "/attachments";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.debug.symx_check_for_NaNs = true;

	settings.debug.symx_force_load = true;
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
	const stark::utils::Mesh<3> box_mesh = stark::utils::make_box({ bs, bs, bs });
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
void deformable_box_collisions()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "deformable_box_collisions";
	settings.output.output_directory = OUTPUT_PATH + "/deformable_box_collisions";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.debug.symx_check_for_NaNs = true;
	stark::Simulation simulation(settings);

	// Contact
	simulation.interactions->contact->set_global_params(
		stark::EnergyFrictionalContact::GlobalParams()
			.set_friction_stick_slide_threshold(0.01)
			.set_min_contact_stiffness(1e7)
	);

	// Geometry
	double gap = 0.01;
	int n1 = 5;
	double d1 = 0.25;
	double hd1 = d1 / 2.0;
	int n2 = n1;
	double d2 = 0.1;
	double hd2 = d2 / 2.0;
	double contact_distance_1 = 0.004;
	double contact_distance_2 = 0.001;

	// Deformables
	stark::Volume::Params rubber = stark::Volume::Params::Soft_Rubber();
	
	rubber.contact.contact_thickness = contact_distance_1;
	auto [V1, T1, H1] = simulation.presets->deformables->add_volume_grid("boxes", { d1, d1, d1 }, {n1, n1, n1}, rubber);
	H1.point_set.add_displacement({ 0.0, 0.0, hd1 + gap });

	rubber.contact.contact_thickness = contact_distance_2;
	auto [V2, T2, H2] = simulation.presets->deformables->add_volume_grid("boxes", { d2, d2, d2 }, {n2, n2, n2}, rubber);
	H2.point_set.add_displacement({ 0.13*d2 , 0.07*d2, d1 + hd2 + 2*gap});

	// Rigid
	double fs = 1.0;
	double fh = 0.1;
	double contact_distance_floor = gap;
	auto [V, C, floor] = simulation.presets->rigidbodies->add_box("floor", 1.0, { fs, fs, fh }, stark::ContactParams().set_contact_thickness(gap));
	floor.rigidbody.set_translation({ 0.0, 0.0, -0.5*fh });
	simulation.rigidbodies->add_constraint_fix(floor.rigidbody);

	// BC
	auto bc = simulation.deformables->prescribed_positions->add_inside_aabb(H1.point_set, { 0.0, 0.0, -hd1 }, { d1, d1, 0.001 }, 
		stark::EnergyPrescribedPositions::Params().set_stiffness(1e5)
		);

	// Contact
	double mu = 1.0;
	simulation.interactions->contact->set_friction(floor.contact, H1.contact, mu);
	simulation.interactions->contact->set_friction(H1.contact, H2.contact, mu);

	// Run
	simulation.run();
}
void floor()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "floor";
	settings.output.output_directory = OUTPUT_PATH + "/floor";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.debug.symx_check_for_NaNs = true;
	stark::Simulation simulation(settings);


	// Parameters
	double gap = 0.01;
	double contact_distance = 0.004;
	int n1 = 2;
	double d1 = 0.25;
	double hd1 = d1 / 2.0;
	double mu = 1.0;

	// Contact
	simulation.interactions->contact->set_global_params(
		stark::EnergyFrictionalContact::GlobalParams()
			.set_default_contact_thickness(contact_distance)
			.set_friction_stick_slide_threshold(0.01)
			.set_min_contact_stiffness(1e7)
	);

	// Deformables
	auto [V1, T1, H1] = simulation.presets->deformables->add_volume_grid("box", { d1, d1, d1 }, {n1, n1, n1}, stark::Volume::Params::Soft_Rubber());
	H1.point_set.add_displacement({ 0.0, 0.0, hd1 + gap });


#if 1
	// Deformable
	auto [V, T] = stark::utils::generate_triangle_grid({ 0.0, 0.0 }, { 1.0, 1.0 }, { 10, 10 });
	auto HF = simulation.deformables->point_sets->add(V);
	auto BC = simulation.deformables->prescribed_positions->add(HF, HF.all(), 
		stark::EnergyPrescribedPositions::Params()
			.set_stiffness(1e6)
			//.set_tolerance(0.002)
	);
	simulation.deformables->output->add_triangle_mesh("floor", HF, T);
	auto contact_handle_floor = simulation.interactions->contact->add_triangles(HF, T, { contact_distance });

#else
	// Rigid
	double fs = 1.0;
	double fh = 0.1;
	auto floor = simulation.rigidbodies->add_with_box_inertia(1.0, { fs, fs, fh })
		.set_translation({ 0.0, 0.0, -0.5*fh });
	stark::utils::Mesh<3> floor_mesh = stark::utils::make_box({ fs, fs, fh });
	simulation.rigidbodies->output.add_triangle_mesh("floor", floor, floor_mesh.vertices, floor_mesh.conn);
	simulation.rigidbodies->add_constraint_fix(floor);

#endif


	simulation.interactions->contact->set_friction(H1.contact, contact_handle_floor, mu);

	// Script
	const double start = 2.0;
	const double duration = 0.1;
	//std::vector<Eigen::Vector3d> target;
	//for (const auto& point : V) {
	//	// Calculate azimuthal angle (phi) from the x-y plane
	//	double phi = std::atan2(point.y(), point.x());

	//	// Assuming a mapping that projects the points onto the upper hemisphere,
	//	// calculate the polar angle (theta) based on the distance from the origin
	//	// in the x-y plane since z is always 0 in the input.
	//	double xyDistance = std::sqrt(point.x() * point.x() + point.y() * point.y());
	//	double theta = std::acos(xyDistance / std::sqrt(2)); // Ensuring the point is on the positive z-side

	//	// Convert spherical coordinates to Cartesian coordinates
	//	double x = std::sqrt(2) * std::sin(theta) * std::cos(phi);
	//	double y = std::sqrt(2) * std::sin(theta) * std::sin(phi);
	//	double z = std::sqrt(2) * std::cos(theta);

	//	target.push_back(Eigen::Vector3d(x, y, z));
	//}

	simulation.add_time_event(start, start + duration,
	[&](double t) 
	{
		const double z = stark::utils::blend(0.0, d1 - 2.0 * gap, start, start + duration, t, stark::utils::BlendType::Linear);
		BC.set_transformation({ 0.0, 0.0, z });
	}
	);

	// Run
	simulation.run();
}
void grasp_test()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "grasp_test";
	settings.output.output_directory = OUTPUT_PATH + "/grasp_test";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.debug.symx_check_for_NaNs = true;
	settings.simulation.gravity = { 0.0, 0.0, 0.0 };
	stark::Simulation simulation(settings);


	// Parameters
	int n = 2;
	double s = 0.2;
	double hs = s/2.0;
	double gap = 0.02;
	double contact_thickness = 0.005;
	double mass = 1.0;
	double gravity = -10.0;
	double pressure = 10.0; // [N]
	double mu_sticking = 1.05;
	double mu_sliding = 0.95;

	// Contact
	simulation.interactions->contact->set_global_params(
		stark::EnergyFrictionalContact::GlobalParams()
		.set_default_contact_thickness(contact_thickness)
		.set_friction_stick_slide_threshold(0.001)
		.set_min_contact_stiffness(1e7)
	);

	// obj
#if 1  // Rigid
	auto [obj, obj_c] = [&]() 
	{
		auto [V, C, H] = simulation.presets->rigidbodies->add_box("rigid", mass, s);
		return std::make_tuple( H.rigidbody, H.contact );
	}();
#else  // Deformable
	auto [obj, obj_c] = [&]()
		{
		auto params = stark::Volume::Params::Soft_Rubber();
		params.inertia.damping = 0.0;
		params.inertia.density = mass/std::pow(s, 3);
		params.strain.elasticity_only = true;
		params.strain.youngs_modulus = 1e7;
		auto [vertices, tets, obj] = simulation.deformables->add_volume_grid({ s, s, s }, { n, n, n }, params, "deformable");
		auto [tris, tri_tet_map] = stark::utils::extract_surface(vertices, tets);
		auto contact = simulation.interactions->contact->add_triangles(obj.point_set, contact_distance, tris, tri_tet_map);
		return std::make_tuple( obj, contact );
	}();
#endif


	// Hand
	auto [hand, hand_c] = [&]()
	{
		auto [V, C, H] = simulation.presets->rigidbodies->add_box("hand", mass, { 3*s, 3*s, 3*s });
		H.rigidbody.set_translation({ 0.0, -(3 * hs + hs + gap), 0.0 });
		return std::make_tuple(H.rigidbody, H.contact);
	}();

	// Fingers
	const Eigen::Vector3d fingers_size = { 0.5 * s, 2 * s, 2 * s };
	auto [left_finger, left_finger_c] = [&]()
	{
		auto [V, C, H] = simulation.presets->rigidbodies->add_box("hand", mass, fingers_size);
		H.rigidbody.set_translation({ -(hs + 0.5 * hs + gap), -gap, 0.0 });
		return std::make_tuple(H.rigidbody, H.contact);
	}();
	auto [right_finger, right_finger_c] = [&]()
	{
		auto [V, C, H] = simulation.presets->rigidbodies->add_box("hand", mass, fingers_size);
		H.rigidbody.set_translation({ (hs + 0.5 * hs + gap), -gap, 0.0 });
		return std::make_tuple(H.rigidbody, H.contact);
	}();

	// Disable collisions
	simulation.interactions->contact->disable_collision(hand_c, left_finger_c);
	simulation.interactions->contact->disable_collision(hand_c, right_finger_c);

	// BC
	const Eigen::Vector3d p = Eigen::Vector3d::Zero();
	const Eigen::Vector3d d = Eigen::Vector3d::UnitX();

	simulation.rigidbodies->add_constraint_fix(hand);
	simulation.rigidbodies->add_constraint_prismatic_press(hand, left_finger, p, d, 1.0, 0.5*pressure);
	simulation.rigidbodies->add_constraint_prismatic_press(hand, right_finger, p, d, -1.0, 0.5*pressure);

	// Script
	simulation.add_time_event(1.0, 5.0, [&](double t) { simulation.set_gravity({ 0.0, 0.0, gravity }); });
	simulation.add_time_event(1.0, 3.0, 
		[&](double t) 
		{ 
			simulation.interactions->contact->set_friction(left_finger_c, obj_c, mu_sticking);
			simulation.interactions->contact->set_friction(right_finger_c, obj_c, mu_sticking);
		}
	);
	simulation.add_time_event(3.0, 5.0, 
		[&](double t) 
		{ 
			simulation.interactions->contact->set_friction(left_finger_c, obj_c, mu_sliding);
			simulation.interactions->contact->set_friction(right_finger_c, obj_c, mu_sliding);
		}
	);

	// Run
	simulation.run();
}
void inflation()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "inflation";
	settings.output.output_directory = OUTPUT_PATH + "/inflation";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.debug.symx_check_for_NaNs = true;
	settings.simulation.init_frictional_contact = false;
	stark::Simulation simulation(settings);

	// Contact
	simulation.interactions->contact->set_global_params(
		stark::EnergyFrictionalContact::GlobalParams()
		.set_default_contact_thickness(0.002)
		.set_friction_enabled(false)
		.set_min_contact_stiffness(1e7)
	);

	// Cloth
	int n = 50;
	double d = 1.0;
	double hd = d / 2.0;
	double gap = 0.001;

	stark::Surface::Params material = stark::Surface::Params::Cotton_Fabric();
	material.inertia.damping = 1.0;
	material.strain.damping = 0.1;
	//material.bending.damping = 1e-4;
	material.bending.damping = 5e-6;
	auto [V, T, H] = simulation.presets->deformables->add_surface_grid("cloth", { d, d }, { n, n }, material);

	// BC
	std::vector<int> prescribed_indices;
	for (int i = 0; i < (int)V.size(); i++) {
		const Eigen::Vector3d& p = V[i];
		const double x_ = std::abs(p.x());
		const double y_ = std::abs(p.y());

		// Perimeter
		if (std::max(x_, y_) > hd - gap) {
			prescribed_indices.push_back(i);
		} 
		
		// Cross
		else if (std::min(x_, y_) < gap) {
			prescribed_indices.push_back(i);
		}

		// Diagonal
		else if (std::abs(p.x() - p.y()) < gap) {
			prescribed_indices.push_back(i);
		}
	}
	simulation.deformables->prescribed_positions->add(H.point_set, prescribed_indices, stark::EnergyPrescribedPositions::Params());

	// Run
	simulation.run();
}
void parachute()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "parachute";
	settings.output.output_directory = OUTPUT_PATH + "/parachute";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 5.0;
	settings.debug.symx_check_for_NaNs = true;
	settings.simulation.init_frictional_contact = false;
	stark::Simulation simulation(settings);

	// Cloth
	int n = 50;
	double d = 1.0;
	double hd = d / 2.0;
	double gap = 0.15;

	stark::Surface::Params material = stark::Surface::Params::Cotton_Fabric();
	material.inertia.damping = 1.0;
	material.strain.damping = 0.1;
	material.strain.inflation = 1e4;
	material.bending.damping = 5e-6;
	auto [V, T, H] = simulation.presets->deformables->add_surface_grid("cloth", { d, d }, { n, n }, material);

	// BC
	auto bc_params = stark::EnergyPrescribedPositions::Params();
	simulation.deformables->prescribed_positions->add_inside_aabb(H.point_set, { hd, hd, 0.0 }, { gap, gap, gap }, bc_params);
	simulation.deformables->prescribed_positions->add_inside_aabb(H.point_set, { hd, -hd, 0.0 }, { gap, gap, gap }, bc_params);
	simulation.deformables->prescribed_positions->add_inside_aabb(H.point_set, { -hd, -hd, 0.0 }, { gap, gap, gap }, bc_params);
	simulation.deformables->prescribed_positions->add_inside_aabb(H.point_set, { -hd, hd, 0.0 }, { gap, gap, gap }, bc_params);

	// Run
	simulation.run();
}


int main()
{
	//hanging_net();
	//hanging_cloth();
	//hanging_box();
	//attachments();
	//ball_joints();
	//inflation();
	//parachute();

	deformable_box_collisions();
	//grasp_test();
	//floor();
}
