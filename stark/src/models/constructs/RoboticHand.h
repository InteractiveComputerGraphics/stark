#pragma once
#include "../Simulation.h"

namespace stark
{
	class RoboticHand
	{
	public:
		RoboticHand(const std::string& name, Simulation& simulation, const ContactParams& contact_params = ContactParams());

		void set_finger_angle(double angle_deg);
		void set_finger_spread_angle(double angle_deg);
		void set_friction(const ContactHandler& contact, double friction);

	private:
		Simulation& simulation;
		std::string name;
		RigidBody::Handler palm;
		std::array<std::array<RigidBody::Handler, 4>, 4> fingers;
		std::array<std::array<std::pair<RBCHingeJointHandler, RBCDirectionHandler>, 4>, 4> finger_hinges;
		std::array<RigidBody::Handler, 5> thumb;
		std::array<std::pair<RBCHingeJointHandler, RBCDirectionHandler>, 5> thumb_hinges;
	};
}