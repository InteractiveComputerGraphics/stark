#include "RoboticHand.h"

stark::RoboticHand::RoboticHand(const std::string& name, Simulation& simulation, const ContactParams& contact_params)
	: simulation(simulation), name(name)
{
	auto add_box = [&](double mass, const Eigen::Vector3d& size, const Eigen::Vector3d& translation)
		{
			auto [V, C, H] = simulation.presets->rigidbodies->add_box("hidden", mass, size, contact_params);
			H.rigidbody.set_translation(translation);
			return H;
		};
	auto add_with_box_inertia = [&](const std::string& path, double mass, const Eigen::Vector3d& size, const Eigen::Vector3d& translation)
		{
			Mesh<3> render_mesh = load_obj(path + ".obj")[0];
			auto [vertices, triangles] = clean_triangle_mesh(render_mesh.vertices, render_mesh.conn, /*merge_by_distance=*/1e-4);
			move(vertices, -translation);
			const Eigen::Matrix3d inertia = inertia_tensor_box(mass, size);
			auto H = simulation.presets->rigidbodies->add(name, mass, inertia, vertices, triangles, contact_params);
			H.rigidbody.set_translation(translation);
			return H;
		};

	auto add_hinge_v = [&](RigidBody::Handler& a, RigidBody::Handler& b, const Eigen::Vector3d& center)
		{
			auto hinge = simulation.rigidbodies->add_constraint_hinge(a.rigidbody, b.rigidbody, center, Eigen::Vector3d::UnitZ());
			auto direction = simulation.rigidbodies->add_constraint_direction(a.rigidbody, b.rigidbody, Eigen::Vector3d::UnitY());
			return std::make_pair(hinge, direction);
		};
	auto add_hinge_h = [&](RigidBody::Handler& a, RigidBody::Handler& b, const Eigen::Vector3d& center)
		{
			auto hinge = simulation.rigidbodies->add_constraint_hinge(a.rigidbody, b.rigidbody, center, Eigen::Vector3d::UnitX());
			auto direction = simulation.rigidbodies->add_constraint_direction(a.rigidbody, b.rigidbody, Eigen::Vector3d::UnitY());
			return std::make_pair(hinge, direction);
		};
	auto add_hinge = [&](RigidBody::Handler& a, RigidBody::Handler& b, const Eigen::Vector3d& center, const Eigen::Vector3d& hinge_dir)
		{
			const Eigen::Vector3d not_hinge_dir = { hinge_dir[1], hinge_dir[2], hinge_dir[0] };
			auto hinge = simulation.rigidbodies->add_constraint_hinge(a.rigidbody, b.rigidbody, center, hinge_dir.normalized());
			auto direction = simulation.rigidbodies->add_constraint_direction(a.rigidbody, b.rigidbody, hinge_dir.cross(not_hinge_dir).normalized());
			return std::make_pair(hinge, direction);
		};

	auto disable_collision = [&](RigidBody::Handler& a, RigidBody::Handler& b)
		{
			simulation.interactions->contact->disable_collision(a.contact, b.contact);
		};

	// Default constraint stiffness must be soft to not overpower contact forces
	simulation.rigidbodies->set_default_constraint_stiffness(5e3);
	simulation.rigidbodies->set_default_constraint_angle_tolerance(50.0);  // Large value to avoid constraint violation
	simulation.rigidbodies->set_default_constraint_distance_tolerance(1.0);  // Large value to avoid constraint violation


	// Bodies
	this->palm = add_with_box_inertia("../models/robotic_hand/palm", 0.1, { 0.084, 0.084, 0.03 }, { 0.002388, 0.043911, -0.0079 });
	this->fingers[0][0] = add_box(0.01, { 0.018, 0.018, 0.018 }, { -0.024328, 0.076543, -0.011774 });   // Base for spread pivoting
	this->fingers[0][1] = add_with_box_inertia("../models/robotic_hand/finger_1_1", 0.01, { 0.018, 0.018, 0.018 }, { -0.024328, 0.10221, -0.011774 });
	this->fingers[0][2] = add_with_box_inertia("../models/robotic_hand/finger_1_2", 0.01, { 0.017, 0.022, 0.017 }, { -0.024859, 0.12688, -0.012643 });
	this->fingers[0][3] = add_with_box_inertia("../models/robotic_hand/finger_1_3", 0.01, { 0.013, 0.031, 0.013 }, { -0.024859, 0.15547, -0.01336 });

	this->fingers[1][0] = add_box(0.01, { 0.018, 0.018, 0.018 }, { -0.003009, 0.076543, -0.011774 });   // Base for spread pivoting
	this->fingers[1][1] = add_with_box_inertia("../models/robotic_hand/finger_2_1", 0.01, { 0.017, 0.020, 0.017 }, { -0.003473, 0.103110, -0.011792 });
	this->fingers[1][2] = add_with_box_inertia("../models/robotic_hand/finger_2_2", 0.01, { 0.015, 0.023, 0.015 }, { -0.003967, 0.129510, -0.012598 });
	this->fingers[1][3] = add_with_box_inertia("../models/robotic_hand/finger_2_3", 0.01, { 0.012, 0.033, 0.012 }, { -0.003967, 0.160100, -0.013265 });

	this->fingers[2][0] = add_box(0.01, { 0.018, 0.018, 0.018 }, { 0.016419, 0.075, -0.011774 });   // Base for spread pivoting
	this->fingers[2][1] = add_with_box_inertia("../models/robotic_hand/finger_3_1", 0.01, { 0.018, 0.018, 0.018 }, { 0.016419, 0.096883, -0.011774 });
	this->fingers[2][2] = add_with_box_inertia("../models/robotic_hand/finger_3_2", 0.01, { 0.017, 0.022, 0.017 }, { 0.015887, 0.121550, -0.012643 });
	this->fingers[2][3] = add_with_box_inertia("../models/robotic_hand/finger_3_3", 0.01, { 0.013, 0.031, 0.013 }, { 0.015887, 0.150140, -0.013360 });

	this->fingers[3][0] = add_box(0.01, { 0.014, 0.014, 0.014 }, { 0.035284, 0.07, -0.009473 });   // Base for spread pivoting
	this->fingers[3][1] = add_with_box_inertia("../models/robotic_hand/finger_4_1", 0.01, { 0.014, 0.014, 0.014 }, { 0.035284, 0.088866, -0.009473 });
	this->fingers[3][2] = add_with_box_inertia("../models/robotic_hand/finger_4_2", 0.01, { 0.013, 0.017, 0.013 }, { 0.034881, 0.107540, -0.010131 });
	this->fingers[3][3] = add_with_box_inertia("../models/robotic_hand/finger_4_3", 0.01, { 0.010, 0.024, 0.010 }, { 0.034881, 0.129180, -0.010674 });

	this->thumb[0] = add_box(0.01, { 0.031, 0.031, 0.031 }, { -0.0268, 0.034899, 0.00697 });   // Base for spread pivoting
	this->thumb[1] = add_box(0.01, { 0.031, 0.031, 0.031 }, { -0.0268, 0.034899, 0.00697 });   // Base for spread pivoting
	this->thumb[2] = add_with_box_inertia("../models/robotic_hand/thumb_1", 0.01, { 0.031, 0.031, 0.031 }, { -0.0268, 0.034899, 0.00697 });
	this->thumb[3] = add_with_box_inertia("../models/robotic_hand/thumb_2", 0.01, { 0.016, 0.016, 0.016 }, { -0.046861, 0.058065, 0.019958 });
	this->thumb[4] = add_with_box_inertia("../models/robotic_hand/thumb_3", 0.01, { 0.016, 0.016, 0.016 }, { -0.059656, 0.086962, 0.028543 });


	// Joints
	{
		simulation.rigidbodies->add_constraint_fix(this->palm.rigidbody).set_stiffness(1e6);

		this->finger_hinges[0][0] = add_hinge_v(this->palm, this->fingers[0][0], { -0.023179, 0.091589, -0.013087 });
		this->finger_hinges[0][1] = add_hinge_h(this->fingers[0][0], this->fingers[0][1], { -0.023179, 0.091589, -0.0115 });
		this->finger_hinges[0][2] = add_hinge_h(this->fingers[0][1], this->fingers[0][2], { -0.024345, 0.116860, -0.0115 });
		this->finger_hinges[0][3] = add_hinge_h(this->fingers[0][2], this->fingers[0][3], { -0.024501, 0.142990, -0.0115 });

		this->finger_hinges[1][0] = add_hinge_v(this->palm, this->fingers[1][0], { -0.00415, 0.086321, -0.012405 });
		this->finger_hinges[1][1] = add_hinge_h(this->fingers[1][0], this->fingers[1][1], { -0.00415, 0.086321, -0.012405 });
		this->finger_hinges[1][2] = add_hinge_h(this->fingers[1][1], this->fingers[1][2], { -0.00415, 0.11639, -0.012129 });
		this->finger_hinges[1][3] = add_hinge_h(this->fingers[1][2], this->fingers[1][3], { -0.00415, 0.14432, -0.012682 });

		this->finger_hinges[2][0] = add_hinge_v(this->palm, this->fingers[2][0], { 0.015903, 0.082274, -0.012405 });
		this->finger_hinges[2][1] = add_hinge_h(this->fingers[2][0], this->fingers[2][1], { 0.015903, 0.082274, -0.012405 });
		this->finger_hinges[2][2] = add_hinge_h(this->fingers[2][1], this->fingers[2][2], { 0.0903, 0.10946, -0.012129 });
		this->finger_hinges[2][3] = add_hinge_h(this->fingers[2][2], this->fingers[2][3], { 0.015903, 0.13679, -0.012682 });

		this->finger_hinges[3][0] = add_hinge_v(this->palm, this->fingers[3][0], { 0.034877, 0.078228, -0.009752 });
		this->finger_hinges[3][1] = add_hinge_h(this->fingers[3][0], this->fingers[3][1], { 0.034877, 0.078228, -0.009752 });
		this->finger_hinges[3][2] = add_hinge_h(this->fingers[3][1], this->fingers[3][2], { 0.034877, 0.09903, -0.010056 });
		this->finger_hinges[3][3] = add_hinge_h(this->fingers[3][2], this->fingers[3][3], { 0.034877, 0.11971, -0.010609 });

		this->thumb_hinges[0] = add_hinge_v(this->palm, this->thumb[0], { -0.016937, 0.031673, 0.007215 });
		this->thumb_hinges[1] = add_hinge_h(this->thumb[0], this->thumb[1], { -0.016937, 0.031673, 0.007215 });
		//this->thumb_hinges[1] = add_hinge(this->thumb[0], this->thumb[1], { -0.016937, 0.031673, 0.007215 }, Eigen::Vector3d({ 2.0, 1.0, -0.5 }).normalized());
		this->thumb_hinges[2] = add_hinge(this->thumb[1], this->thumb[2], { -0.016937, 0.031673, 0.007215 }, Eigen::Vector3d({ -0.008985, 0.006728, -0.014115 }).normalized());
		this->thumb_hinges[3] = add_hinge(this->thumb[2], this->thumb[3], { -0.039389, 0.047162, 0.019492 }, Eigen::Vector3d({ -0.008985, 0.006728, -0.014115 }).normalized());
		this->thumb_hinges[4] = add_hinge(this->thumb[3], this->thumb[4], { -0.049011, 0.070826, 0.024790 }, Eigen::Vector3d({ -0.008985, 0.006728, -0.014115 }).normalized());
	}

	// Disable collisions
	for (int finger_i = 0; finger_i < 4; finger_i++) {

		// Consecutive phalanges
		for (int phalanx_j = 0; phalanx_j < 3; phalanx_j++) {
			disable_collision(this->palm, this->fingers[finger_i][phalanx_j]);
			for (int phalanx_k = phalanx_j + 1; phalanx_k < 4; phalanx_k++) {
				disable_collision(this->fingers[finger_i][phalanx_j], this->fingers[finger_i][phalanx_j]);
			}
		}
	}

	for (int phalanx_j = 0; phalanx_j < 3; phalanx_j++) {
		disable_collision(this->palm, this->thumb[phalanx_j]);
		for (int phalanx_k = phalanx_j + 1; phalanx_k < 5; phalanx_k++) {
			disable_collision(this->thumb[phalanx_j], this->thumb[phalanx_k]);
		}
	}
}

void stark::RoboticHand::set_finger_angle(double angle_deg)
{
	const Eigen::Vector3d REF_HINGE_DIR = Eigen::Vector3d::UnitX();

	for (int i = 0; i < 4; i++) {
		for (int j = 1; j < 4; j++) {
			auto& [hinge, dir] = this->finger_hinges[i][j];
			dir.set_opening_angle_deg(-angle_deg, REF_HINGE_DIR);
		}
	}
	
	this->thumb_hinges[2].second.set_opening_angle_deg(-0.55*angle_deg, Eigen::Vector3d({ -0.008985,  0.006728, -0.014115 }).normalized());
	this->thumb_hinges[3].second.set_opening_angle_deg(-0.55*angle_deg, Eigen::Vector3d({ -0.008985,  0.006728, -0.014115 }).normalized());
	this->thumb_hinges[4].second.set_opening_angle_deg(-0.55*angle_deg, Eigen::Vector3d({ -0.008985,  0.006728, -0.014115 }).normalized());
}

void stark::RoboticHand::set_finger_spread_angle(double angle_deg)
{
	const Eigen::Vector3d REF_HINGE_ORTH_DIR = Eigen::Vector3d::UnitZ();

	std::array<double, 4> muliplier = { -2.0, -1.0, 1.0, 2.0 };
	for (int i = 0; i < 4; i++) {
		this->finger_hinges[i][0].second.set_opening_angle_deg(muliplier[i] * angle_deg, REF_HINGE_ORTH_DIR);
	}
	this->thumb_hinges[0].second.set_opening_angle_deg(-0.0 * angle_deg, REF_HINGE_ORTH_DIR);
	this->thumb_hinges[1].second.set_opening_angle_deg(-12.0 * angle_deg, Eigen::Vector3d::UnitX());
}

void stark::RoboticHand::set_friction(const ContactHandler& contact, double friction)
{
	auto set_friction = [&](RigidBody::Handler& here, const ContactHandler& other, double friction)
		{
			here.contact.set_friction(other, friction);
		};

	set_friction(this->palm, contact, friction);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			set_friction(this->fingers[i][j], contact, friction);
		}
	}
	for (int i = 0; i < 4; i++) {
		set_friction(this->thumb[i], contact, friction);
	}
}
