#pragma once

#include "../models/include_ui.h"


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
		VehicleFourWheels(Simulation& simulation, Parametrization& params, std::string label);
		void brake();
		void set_target_velocity_in_m_per_s(double v);
		void set_target_velocity_in_km_per_h(double v);
		double get_linear_velocity_in_m_per_s() const;
		double get_linear_velocity_in_km_per_h() const;
		template<typename RBHandler>
		void set_wheels_friction(Simulation& simulation, const RBHandler& object, double friction);
		void add_to_logger(Simulation& simulation) const;
		void rotate_deg(double angle, const Eigen::Vector3d& axis);
		void set_position(const Eigen::Vector3d& position);
		void move(const Eigen::Vector3d& displacement);

	private:
		/* Fields */
		Parametrization params;
		std::string label;
	};


	// Template implementations
	template<typename RBHandler>
	void VehicleFourWheels::set_wheels_friction(Simulation& simulation, const RBHandler& object, double friction)
	{
		for (int i = 0; i < 4; i++) {
			simulation.interactions->set_friction(object, *this->wheels[i], friction);
		}
	}
}