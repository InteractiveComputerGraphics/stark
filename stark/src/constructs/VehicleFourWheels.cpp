#include "VehicleFourWheels.h"

// Constructors
stark::VehicleFourWheels::Parametrization stark::VehicleFourWheels::Parametrization::sedan()
{
	Parametrization p;
	p.chassis.length = 4.7;
	p.chassis.width = 1.8;
	p.chassis.roof_height = 1.43;
	p.chassis.floor_height = 0.15;
	p.chassis.mass = 1500.0;

	p.wheels.radius = 0.22;
	p.wheels.width = 0.2;
	p.wheels.width_inset = 0.15;
	p.wheels.wheelbase = 2.8;
	p.wheels.front_wheels_inset = 0.88;
	p.wheels.mass = 20.0;

	p.suspension.spring_length = 0.2;
	p.suspension.rear_spring_stiffness = 1e5;
	p.suspension.front_spring_stiffness = 1.5e5;
	p.suspension.damping = 10.0;

	p.engine.position = { 0.0, 1.4, 0.7 };
	p.engine.mass = 500.0;
	p.engine.max_torque = 1000.0;
	p.engine.delay = 0.01;
	p.engine.is_front_wheel_drive = false;
	p.engine.is_rear_wheel_drive = true;

	return p;
}


stark::VehicleFourWheels::VehicleFourWheels(Simulation& simulation, Parametrization& params, std::string label)
{
	/*
		TODO:
			All constraints are too weak. Set some reasonable stiffness.
	*/

	this->params = params;
	this->label = label;

	// Convenient lambdas
	auto disable_collision = [&simulation](std::shared_ptr<RigidBodyHandler> a, std::shared_ptr<RigidBodyHandler> b)
		{
			simulation.interactions->disable_collision(*a, *b);
		};

	// Create components
	//// Chassis
	this->chassis = std::make_shared<RigidBodyHandler>(
		simulation.rigidbodies->add_box(
			params.chassis.mass, 
			{ params.chassis.width, params.chassis.length, params.chassis.roof_height - params.chassis.floor_height }
		)
		.set_translation({ 0.0, 0.0, (params.chassis.roof_height - params.chassis.floor_height) / 2.0 + params.chassis.floor_height })
		.add_to_output_label(label + "_chassis")
	);

	//// Engine
	this->engine = std::make_shared<RigidBodyHandler>(
		simulation.rigidbodies->add_box(
			params.engine.mass,
			{ 0.8, 0.8, 0.5 }
		)
		.set_translation(params.engine.position)
		.add_to_output_label(label + "_engine")
	);
	simulation.rigidbodies->add_constraint_attachment(*this->chassis, *this->engine);
	disable_collision(this->chassis, this->engine);

	//// Wheels
	auto add_wheel = [&](int idx, const Eigen::Vector3d& wheel_position, const double max_torque, const double spring_stiffness)
		{
			// Add wheel
			this->wheels[idx] = std::make_shared<RigidBodyHandler>(
				simulation.rigidbodies->add_cylinder(
					params.wheels.mass,
					params.wheels.radius,
					params.wheels.width,
					32 /* slices */
				)
				.set_rotation(90.0, Eigen::Vector3d::UnitY())
				.set_translation(wheel_position)
				.add_to_output_label(label + "_wheels")
			);

			// Add suspension block
			this->suspension_blocks[idx] = std::make_shared<RigidBodyHandler>(
				simulation.rigidbodies->add_box(params.wheels.mass, params.wheels.radius)
				.set_rotation(90.0, Eigen::Vector3d::UnitY())
				.set_translation(wheel_position)
				.add_to_output_label(label + "_suspension")
			);

			// Add constraints
			this->wheel_direction[idx] = std::make_shared<RBCDirectionHandler>(
				simulation.rigidbodies->add_constraint_prismatic_slider(*this->chassis, *this->suspension_blocks[idx],
					wheel_position,
					Eigen::Vector3d::UnitZ()
				).get_direction_lock()
			);
			simulation.rigidbodies->add_constraint_spring(*this->chassis, *this->suspension_blocks[idx],
				wheel_position,
				wheel_position + params.suspension.spring_length * Eigen::Vector3d::UnitZ(),
				spring_stiffness,
				params.suspension.damping);
			this->wheel_motors[idx] = std::make_shared<RBCAngularVelocityHandler>(
				simulation.rigidbodies->add_constraint_motor(*this->suspension_blocks[idx], *this->wheels[idx], 
					wheel_position, 
					-Eigen::Vector3d::UnitX(), 
					0.0, // Target angular velocity
					params.engine.max_torque, 
					params.engine.delay)
				.get_angular_velocity_constraint().enable(false)
			);
			// Disable collisions
			disable_collision(this->chassis, this->wheels[idx]);
			disable_collision(this->chassis, this->suspension_blocks[idx]);
			disable_collision(this->suspension_blocks[idx], this->wheels[idx]);
		};

	////// Common
	if (params.engine.is_front_wheel_drive) {
		this->is_wheel_powered[0] = true;
		this->is_wheel_powered[1] = true;
	}
	if (params.engine.is_rear_wheel_drive) {
		this->is_wheel_powered[2] = true;
		this->is_wheel_powered[3] = true;
	}
	const int n_powered_wheels = 2*(int)params.engine.is_front_wheel_drive + 2*(int)params.engine.is_rear_wheel_drive;
	const double max_wheel_torque = params.engine.max_torque / n_powered_wheels;
	const double wheel_distance_on_width = 0.5 * params.chassis.width - params.wheels.width_inset;

	////// Add wheels
	Eigen::Vector3d wheel_position = { -wheel_distance_on_width, 0.5 * params.chassis.length - params.wheels.front_wheels_inset, params.wheels.radius };
	add_wheel(0, wheel_position, max_wheel_torque, params.suspension.front_spring_stiffness);  // Front left
	wheel_position[0] = wheel_distance_on_width;
	add_wheel(1, wheel_position, max_wheel_torque, params.suspension.front_spring_stiffness);  // Front right
	wheel_position[0] = -wheel_distance_on_width;
	wheel_position[1] -= params.wheels.wheelbase;
	add_wheel(2, wheel_position, max_wheel_torque, params.suspension.rear_spring_stiffness);  // Rear left
	wheel_position[0] = wheel_distance_on_width;
	add_wheel(3, wheel_position, max_wheel_torque, params.suspension.rear_spring_stiffness);  // Rear right
}

void stark::VehicleFourWheels::brake()
{
	for (int i = 0; i < 4; i++) {
		this->wheel_motors[i]->set_target_angular_velocity_in_deg_per_s(0.01);
		this->wheel_motors[i]->enable(true);
	}
}

void stark::VehicleFourWheels::set_target_velocity_in_m_per_s(double v)
{
	// Get radians per second target angular velocity from wheels radius
	const double w_rad_s = v / this->params.wheels.radius;
	const double w_deg_s = utils::rad2deg(w_rad_s);

	for (int i = 0; i < 4; i++) {
		if (this->is_wheel_powered[i]) {
			this->wheel_motors[i]->set_target_angular_velocity_in_deg_per_s(w_deg_s);
			this->wheel_motors[i]->enable(true);
		}
		else {
			this->wheel_motors[i]->enable(false);
		}
	}
}

void stark::VehicleFourWheels::set_target_velocity_in_km_per_h(double v)
{
	this->set_target_velocity_in_m_per_s(v / 3.6);
}

double stark::VehicleFourWheels::get_linear_velocity_in_m_per_s() const
{
	return this->chassis->get_velocity().norm();
}

double stark::VehicleFourWheels::get_linear_velocity_in_km_per_h() const
{
	return this->get_linear_velocity_in_m_per_s() * 3.6;
}
