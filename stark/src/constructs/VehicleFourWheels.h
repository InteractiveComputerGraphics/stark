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
				double spring_stiffness = 0.0;
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

		// Methods
		VehicleFourWheels(Simulation& simulation, Parametrization& params, std::string label);
		void rotate_deg(double angle, const Eigen::Vector3d& axis);
		void set_position(const Eigen::Vector3d& position);
		void move(const Eigen::Vector3d& displacement);

	private:
		// Shorthand
		using spRBH = std::shared_ptr<RigidBodyHandler>;

		/* Fields */
		Parametrization params;
		std::string label;
		spRBH chassis;
		std::array<spRBH, 4> wheels;
		std::array<spRBH, 4> suspension_blocks;
		spRBH engine;
	};
}