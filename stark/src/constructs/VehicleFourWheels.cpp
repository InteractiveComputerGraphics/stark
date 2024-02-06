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
	p.suspension.spring_stiffness = 1e6;
	p.suspension.damping = 1e3;

	p.engine.position = { 0.0, 1.4, 0.7 };
	p.engine.mass = 500.0;
	p.engine.max_torque = 300.0;
	p.engine.delay = 0.1;
	p.engine.is_front_wheel_drive = true;
	p.engine.is_rear_wheel_drive = true;

	return p;
}


stark::VehicleFourWheels::VehicleFourWheels(Simulation& simulation, Parametrization& params, std::string label)
{
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
	// Front left
	Eigen::Vector3d wheel_position = { -(0.5 * params.chassis.width - params.wheels.width_inset), 0.5 * params.chassis.length - params.wheels.front_wheels_inset, params.wheels.radius };
	this->wheels[0] = std::make_shared<RigidBodyHandler>(
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
	this->suspension_blocks[0] = std::make_shared<RigidBodyHandler>(
		simulation.rigidbodies->add_box(params.wheels.mass, params.wheels.radius)
		.set_rotation(90.0, Eigen::Vector3d::UnitY())
		.set_translation(wheel_position)
		.add_to_output_label(label + "_suspension")
	);
	simulation.rigidbodies->add_constraint_prismatic_slider(*this->chassis, *this->suspension_blocks[0], 
		wheel_position, 
		Eigen::Vector3d::UnitZ());
	simulation.rigidbodies->add_constraint_spring(*this->chassis, *this->suspension_blocks[0], 
		wheel_position, 
		wheel_position + params.suspension.spring_length*Eigen::Vector3d::UnitZ(), 
		params.suspension.spring_stiffness, 
		params.suspension.damping);
	simulation.rigidbodies->add_constraint_motor(*this->suspension_blocks[0], *this->wheels[0], wheel_position, Eigen::Vector3d::UnitX(), 0.0, params.engine.max_torque, params.engine.delay)
		.get_angular_velocity_constraint().enable(false);
	disable_collision(this->chassis, this->wheels[0]);
	disable_collision(this->chassis, this->suspension_blocks[0]);
	disable_collision(this->suspension_blocks[0], this->wheels[0]);
}
