#include "VehicleFourWheels.h"

// Constructors
stark::VehicleFourWheels::Parametrization stark::VehicleFourWheels::Parametrization::sedan()
{
	Parametrization p;
	p.chassis.length = 4.7;
	p.chassis.width = 1.8;
	p.chassis.roof_height = 1.0; // 1.43
	p.chassis.floor_height = 0.15;
	p.chassis.mass = 1500.0;

	p.wheels.radius = 0.3;
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
	p.engine.max_torque = 600.0;
	p.engine.gear_ratio = 3.5;  // First gear of sports car
	p.engine.delay = 0.1;
	p.engine.is_front_wheel_drive = false;
	p.engine.is_rear_wheel_drive = true;

	return p;
}


stark::VehicleFourWheels::VehicleFourWheels(std::shared_ptr<Simulation> simulation, Parametrization& params, std::string label)
	: simulation(simulation), params(params), label(label)
{
	// Constants
	const double TOLERANCE_IN_M = 0.005;
	const double TOLERANCE_IN_DEG = 2.0;
	const double HARD_STIFFNESS = 1e6; // TODO: Find a more appropriate way to set this value

	// Log car stuff into the logger as a permanent event
	simulation->get_script().add_recurring_event([this](EventInfo&) { this->_append_to_logger(); });

	// Convenient lambdas
	auto disable_collision = [&simulation](std::shared_ptr<RigidBodyHandler> a, std::shared_ptr<RigidBodyHandler> b)
		{
			simulation->interactions->disable_collision(*a, *b);
		};

	// Create components
	//// Chassis
	this->chassis = std::make_shared<RigidBodyHandler>(
		simulation->rigidbodies->add_box(
			params.chassis.mass, 
			{ params.chassis.width, params.chassis.length, params.chassis.roof_height - params.chassis.floor_height }
		)
		.set_translation({ 0.0, 0.0, (params.chassis.roof_height - params.chassis.floor_height) / 2.0 + params.chassis.floor_height })
		.add_to_output_label(label + "_chassis")
	);

	//// Engine
	this->engine = std::make_shared<RigidBodyHandler>(
		simulation->rigidbodies->add_box(
			params.engine.mass,
			{ 0.8, 0.8, 0.5 }
		)
		.set_translation(params.engine.position)
		.add_to_output_label(label + "_engine")
	);
	simulation->rigidbodies->add_constraint_attachment(*this->chassis, *this->engine)
		.set_tolerance_in_m(TOLERANCE_IN_M)
		.set_stiffness(HARD_STIFFNESS);
	disable_collision(this->chassis, this->engine);

	//// Wheels
	auto add_wheel = [&](int idx, const Eigen::Vector3d& wheel_position, const double max_torque, const double spring_stiffness)
		{
			// Add wheel
			this->wheels[idx] = std::make_shared<RigidBodyHandler>(
				simulation->rigidbodies->add_cylinder(
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
				simulation->rigidbodies->add_box(params.wheels.mass, params.wheels.radius)
				.set_translation(wheel_position)
				.add_to_output_label(label + "_suspension")
			);

			// Add constraints
			//// Slider
			simulation->rigidbodies->add_constraint_slider(*this->chassis, *this->suspension_blocks[idx],
				wheel_position,
				Eigen::Vector3d::UnitZ()
			)
			.set_tolerance_in_m(TOLERANCE_IN_M)
			.set_stiffness(HARD_STIFFNESS);

			//// Direction
			this->wheel_direction[idx] = std::make_shared<RBCDirectionHandler>(
				simulation->rigidbodies->add_constraint_direction(*this->chassis, *this->suspension_blocks[idx],
					Eigen::Vector3d::UnitY()  // Forward direction
				)
				.set_tolerance_in_deg(TOLERANCE_IN_DEG)
				.set_stiffness(HARD_STIFFNESS)
			);

			//// Spring-damper
			this->spring_dampers[idx] = std::make_shared<RBCDampedSpringHandler>(
				simulation->rigidbodies->add_constraint_spring(*this->chassis, *this->suspension_blocks[idx],
					wheel_position,
					wheel_position + params.suspension.spring_length * Eigen::Vector3d::UnitZ(),
					spring_stiffness,
					params.suspension.damping)
			);

			//// Motor
			this->wheel_motors[idx] = std::make_shared<RBCAngularVelocityHandler>(
				simulation->rigidbodies->add_constraint_motor(*this->suspension_blocks[idx], *this->wheels[idx], 
					wheel_position, 
					-Eigen::Vector3d::UnitX(), 
					0.0, // Target angular velocity
					max_torque,
					params.engine.delay)
				.set_tolerance_in_m(TOLERANCE_IN_M)
				.set_stiffness(HARD_STIFFNESS)
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
	const double max_wheel_torque = params.engine.gear_ratio * params.engine.max_torque / n_powered_wheels;
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
		this->wheel_motors[i]->set_target_angular_velocity_in_deg_per_s(0.0);
		this->wheel_motors[i]->enable(true);
	}
}

void stark::VehicleFourWheels::set_target_velocity_in_kmh(double v_km_per_h)
{
	const double v = v_km_per_h / 3.6;

	// Get target radians per second from wheels radius
	const double w_rad_s = v / this->params.wheels.radius;

	for (int i = 0; i < 4; i++) {
		if (this->is_wheel_powered[i]) {
			this->wheel_motors[i]->set_target_angular_velocity_in_rad_per_s(w_rad_s);
			this->wheel_motors[i]->enable(true);
		}
		else {
			this->wheel_motors[i]->enable(false);
		}
	}
}

Eigen::Vector3d stark::VehicleFourWheels::get_forward_velocity_in_km_per_h() const
{
	const Eigen::Vector3d chassis_global_direction = this->chassis->local_to_global_direction(this->FORWARD);
	return chassis_global_direction.dot(this->get_absolute_velocity_in_km_per_h()) * chassis_global_direction;
}

Eigen::Vector3d stark::VehicleFourWheels::get_absolute_velocity_in_km_per_h() const
{
	return 3.6 * this->chassis->get_velocity();
}

void stark::VehicleFourWheels::set_steering_in_deg(double angle_deg, std::array<bool, 4> wheels)
{
	for (int i = 0; i < 4; i++) {
		if (wheels[i]) {
			this->_set_steering(i, angle_deg, /* global = */ false);
		}
	}
}
void stark::VehicleFourWheels::set_global_steering_in_deg(double angle_deg, std::array<bool, 4> wheels)
{
	for (int i = 0; i < 4; i++) {
		if (wheels[i]) {
			this->_set_steering(i, angle_deg, /* global = */ true);
		}
	}
}
double stark::VehicleFourWheels::get_steering_in_deg(int wheel_idx) const
{
	const Eigen::Vector3d d = this->wheel_direction[wheel_idx]->get_local_direction_body_b();
	const double angle_rad = std::acos(d.dot(this->FORWARD)); // Forward is +Y. Negative x because the opposite of the angle must be looking forward.
	return utils::rad2deg(angle_rad);
}

double stark::VehicleFourWheels::get_global_steering_in_deg(int wheel_idx) const
{
	const Eigen::Vector3d d = this->wheel_direction[wheel_idx]->get_local_direction_body_b();
	const Eigen::Vector3d d_global = this->wheel_direction[wheel_idx]->get_body_b().local_to_global_direction(d);
	const double angle_rad = std::acos(d_global.dot(this->FORWARD)); // Forward is +Y. Negative x because the opposite of the angle must be looking forward.
	return utils::rad2deg(angle_rad);
}

void stark::VehicleFourWheels::add_event_brake(double t0, double t1)
{
	this->simulation->add_time_event(t0, t1,
		[=](double t)
		{
			this->brake();
		}
	);
}
void stark::VehicleFourWheels::add_event_target_velocity_in_kmh(double t0, double t1, double begin_kmh, double end_kmh, utils::BlendType blend)
{
	this->simulation->add_time_event(t0, t1,
		[=](double t)
		{
			this->set_target_velocity_in_kmh(stark::utils::blend(begin_kmh, end_kmh, t0, t1, t, blend));
		}
	);
}
void stark::VehicleFourWheels::add_event_steering_in_deg(double t0, double t1, double begin_deg, double end_deg, utils::BlendType blend)
{
	this->simulation->add_time_event(t0, t1,
		[=](double t)
		{
			this->set_steering_in_deg(stark::utils::blend(begin_deg, end_deg, t0, t1, t, blend));
		}
	);
}
void stark::VehicleFourWheels::add_event_global_steering_in_deg(double t0, double t1, double begin_deg, double end_deg, utils::BlendType blend)
{
	this->simulation->add_time_event(t0, t1,
		[=](double t)
		{
			this->set_global_steering_in_deg(stark::utils::blend(begin_deg, end_deg, t0, t1, t, blend));
		}
	);
}

void stark::VehicleFourWheels::_append_to_logger() const
{
	auto& logger = this->simulation->get_logger();

	logger.append_to_series(this->label + "_time", this->simulation->get_time());
	const double v = this->get_absolute_velocity_in_km_per_h().norm();
	logger.append_to_series(this->label + "_abs_velocity_kmh", v);

	for (size_t i = 0; i < 4; i++){
		const double w_rad_s = this->wheels[i]->get_angular_velocity().norm();
		const double wheel_v_hm_h = 3.6 * this->params.wheels.radius*w_rad_s;
		logger.append_to_series(this->label + "_wheel_" + std::to_string(i) + "_w_rad_s", w_rad_s);
		logger.append_to_series(this->label + "_wheel_" + std::to_string(i) + "_sliding_v_m_s", v - wheel_v_hm_h);
		
		auto motor_status = this->wheel_motors[i]->get_signed_angular_velocity_violation_in_deg_per_s_and_torque();
		logger.append_to_series(this->label + "_wheel_" + std::to_string(i) + "_torque", motor_status[1]);
		
		auto damper_status = this->spring_dampers[i]->get_signed_damper_velocity_and_force();
		logger.append_to_series(this->label + "_damper_" + std::to_string(i) + "_v_m_s", damper_status[0]);
		logger.append_to_series(this->label + "_damper_" + std::to_string(i) + "_force", damper_status[1]);

		auto spring_status = this->spring_dampers[i]->get_signed_spring_displacement_in_m_and_force();
		logger.append_to_series(this->label + "_spring_" + std::to_string(i) + "_dx", spring_status[0]);
		logger.append_to_series(this->label + "_spring_" + std::to_string(i) + "_force", spring_status[1]);

		// TODO: Wheel direction
	}
}

void stark::VehicleFourWheels::_set_steering(int wheel_idx, double angle_deg, bool global)
{
	// Forward direction is along Y axis positive
	const double x = std::sin(utils::deg2rad(angle_deg));
	const double y = std::cos(utils::deg2rad(angle_deg));
	const double z = 0.0;
	if (global) {
		auto& chassis = this->wheel_direction[wheel_idx]->get_body_a();
		auto& suspension_block = this->wheel_direction[wheel_idx]->get_body_b();

		const Eigen::Vector3d target = { x, y, z };
		const Eigen::Vector3d chassis_forward = chassis.local_to_global_direction(this->FORWARD);
		const double ang_rad = std::acos(target.dot(chassis_forward));
		this->_set_steering(wheel_idx, utils::rad2deg(ang_rad), false);
	}
	else {
		const Eigen::Vector3d target = { -x, y, z };  // Negative x because the opposite of the angle must be looking forward.
		this->wheel_direction[wheel_idx]->set_local_direction_body_b(target);
	}
}
