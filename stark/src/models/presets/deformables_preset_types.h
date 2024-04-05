#pragma once
#include "../deformables/Deformables.h"
#include "../interactions/Interactions.h"


namespace stark
{
	namespace Line
	{
		enum Preset
		{
			Elastic_Rubberband
		};

		struct Params
		{
			EnergyLumpedInertia::Params inertia;
			EnergySegmentStrain::Params strain;
			ContactParams contact;

			Params() = default;
			Params(const Preset& preset);
			static Params Elastic_Rubberband();
		};

		struct Handler
		{
			PointSetHandler point_set;
			EnergyLumpedInertia::Handler inertia;
			EnergySegmentStrain::Handler strain;
			ContactHandler contact;
		};

		/**
		* @brief Vertex-Connectivity-Handler
		*/
		struct VCH
		{
			std::vector<Eigen::Vector3d> vertices;
			std::vector<std::array<int, 2>> segments;
			Line::Handler handler;
		};
	}

	namespace Surface
	{
		enum Preset
		{
			Cotton_Fabric
		};

		struct Params
		{
			EnergyLumpedInertia::Params inertia;
			EnergyTriangleStrain::Params strain;
			EnergyDiscreteShells::Params bending;
			ContactParams contact;

			Params() = default;
			Params(const Preset& preset);
			static Params Cotton_Fabric();
		};

		struct Handler
		{
			PointSetHandler point_set;
			EnergyLumpedInertia::Handler inertia;
			EnergyTriangleStrain::Handler strain;
			EnergyDiscreteShells::Handler bending;
			ContactHandler contact;
		};

		/**
		* @brief Vertex-Connectivity-Handler
		*/
		struct VCH
		{
			std::vector<Eigen::Vector3d> vertices;
			std::vector<std::array<int, 3>> triangles;
			Surface::Handler handler;
		};
	}

	namespace PrescribedSurface
	{
		struct Params
		{
			EnergyPrescribedPositions::Params prescribed;
			ContactParams contact;

			Params() = default;
		};

		struct Handler
		{
			PointSetHandler point_set;
			EnergyPrescribedPositions::Handler prescribed;
			ContactHandler contact;
		};
	}



	namespace Volume
	{
		enum Preset
		{
			Soft_Rubber
		};

		struct Params
		{
			EnergyLumpedInertia::Params inertia;
			EnergyTetStrain::Params strain;
			ContactParams contact;

			Params() = default;
			Params(const Preset& preset);
			static Params Soft_Rubber();
		};

		struct Handler
		{
			PointSetHandler point_set;
			EnergyLumpedInertia::Handler inertia;
			EnergyTetStrain::Handler strain;
			ContactHandler contact;
		};

		/**
		* @brief Vertex-Connectivity-Handler
		*/
		struct VCH
		{
			std::vector<Eigen::Vector3d> vertices;
			std::vector<std::array<int, 4>> tets;
			Volume::Handler handler;
		};
	}
}
