#pragma once

#include "../models/include_ui.h"
#include "../utils/blends.h"

namespace stark
{
	class VehicleFourWheels
	{
	public:
		// Parametrization
		struct Parametrization
		{
			struct Chassis
			{
				double length = 0.0;
				double width = 0.0;
				double roof_height = 0.0;
				double floor_height = 0.0;
				double mass = 0.0;
			};
			struct Wheels
			{
				double radius = 0.0;
				double width = 0.0;
				double width_inset = 0.0;  // How much the wheel is inset from being flushed to the chassis sides
				double wheelbase = 0.0;  // Distance between the front and rear wheels
				double front_wheels_inset = 0.0;  // How much the front wheels are inset from the front of the chassis
				double mass = 0.0;
			};
			struct Suspension
			{
				double spring_length = 0.0;
				double rear_spring_stiffness = 0.0;
				double front_spring_stiffness = 0.0;
				double damping = 0.0;
			};
			struct Engine
			{
				Eigen::Vector3d position;
				double mass = 0.0;
				double max_torque = 0.0;
				double gear_ratio = 0.0;
				double delay = 0.1;
				bool is_front_wheel_drive = false;
				bool is_rear_wheel_drive = false;
			};

			Chassis chassis;
			Wheels wheels;
			Suspension suspension;
			Engine engine;

			// Constructors
			static Parametrization sedan();
		};

		// Fields
		std::shared_ptr<RigidBodyHandler> chassis;
		std::array<std::shared_ptr<RigidBodyHandler>, 4> wheels;
		std::array<std::shared_ptr<RigidBodyHandler>, 4> suspension_blocks;
		std::shared_ptr<RigidBodyHandler> engine;
		std::array<std::shared_ptr<RBCDampedSpringHandler>, 4> spring_dampers;
		std::array<std::shared_ptr<RBCAngularVelocityHandler>, 4> wheel_motors;
		std::array<std::shared_ptr<RBCDirectionHandler>, 4> wheel_direction;
		std::array<bool, 4> is_wheel_powered = { false, false, false, false };
		
		// Methods
		VehicleFourWheels(std::shared_ptr<Simulation> simulation, Parametrization& params, std::string label);

		//// Transformations
		VehicleFourWheels& rotate_deg(double angle, const Eigen::Vector3d& axis);
		VehicleFourWheels& set_position(const Eigen::Vector3d& position);
		VehicleFourWheels& move(const Eigen::Vector3d& displacement);

		// Set behavior
		template<typename RBHandler>
		void set_wheels_friction(const RBHandler& object, double friction);
		template<typename RBHandler>
		void set_chassis_friction(const RBHandler& object, double friction);
		void brake();
		void set_target_velocity_in_km_per_h(double v);
		Eigen::Vector3d get_forward_velocity_in_km_per_h() const;
		Eigen::Vector3d get_absolute_velocity_in_km_per_h() const;
		void set_steering_in_deg(double angle_deg, std::array<bool, 4> wheels = { true, true, false, false });
		double get_steering_in_deg(int wheel_idx) const;

		// Script behavior
		void append_to_steering_script(double prev_angle_deg, double target_angle_deg, double duration, std::array<bool, 4> wheels = { true, true, false, false }, utils::BlendType blend = utils::BlendType::Linear, std::function<bool()> exit_early_when = nullptr);
		void append_to_velocity_script__target_velocity_kmh(double prev_velocity_in_kmh, double target_velocity_in_kmh, double duration, utils::BlendType blend = utils::BlendType::Instant, std::function<bool()> exit_early_when = nullptr);
		void append_to_velocity_script__brake(double duration, std::function<bool()> exit_early_when = nullptr);


	private:
		/* Fields */
		std::shared_ptr<Simulation> simulation;
		Parametrization params;
		std::string label;
		const Eigen::Vector3d LOCAL_FORWARD = Eigen::Vector3d::UnitY();
		
		// Scripting
		int velocity_action_queue_idx = -1;
		int steering_action_queue_idx = -1;

		/* Methods */
		void _append_to_logger() const;
		void _set_steering(int wheel_idx, double angle_deg);
		std::function<bool(EventInfo&)> _generate_stop_at_lambda(double duration, std::function<bool()> exit_early_when);
	};





	// Template implementations
	template<typename RBHandler>
	void VehicleFourWheels::set_wheels_friction(const RBHandler& object, double friction)
	{
		for (int i = 0; i < 4; i++) {
			this->simulation->interactions->set_friction(object, *this->wheels[i], friction);
		}
	}
	template<typename RBHandler>
	inline void VehicleFourWheels::set_chassis_friction(const RBHandler& object, double friction)
	{
		this->simulation->interactions->set_friction(object, *this->chassis, friction);
	}
}