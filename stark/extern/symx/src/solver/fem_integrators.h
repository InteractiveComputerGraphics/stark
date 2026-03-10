#pragma once
#include <type_traits>
#include "../compile/MappedWorkspace.h"

// Source: https://www.researchgate.net/publication/267082822_Unified_Isoparametric_3D_LagrangeFinite_Elements

namespace symx
{
	enum class FEM_Element { Tet4, Tet10, Hex8, Hex27 };
	std::string get_name(const FEM_Element& element);
	int get_stride(const FEM_Element& element);
	int get_stride(const FEM_Element& element);

	template<typename T>
	T fem_interpolation(const FEM_Element& element, std::vector<T>& vh, const Vector& xi);
	Matrix fem_jacobian(const FEM_Element& element, std::vector<Vector>& xh, const Vector& xi);
	std::vector<std::array<double, 4>> get_integration_rule(const FEM_Element& element); // Returns {xi0, xi1, xi2, weight} quadrature points for the element type

	// Registers a Summation in the MappedWorkspace so that the compiled kernel is called
	// once per quadrature point. The summand lambda receives the weight w and reference
	// coordinates xi, and returns the integrand scalar. The result is the weighted sum
	// over all integration points (Gauss quadrature).
	Scalar fem_integrator(MappedWorkspace<double>& mws, const FEM_Element& element, std::function<Scalar(Scalar& w, Vector& xi)> summand);
	

	template<typename T>
	T fem_interpolation(const FEM_Element& element, std::vector<T>& vh, const Vector& xi)
	{
		if constexpr (!std::is_same_v<T, Scalar> && !std::is_same_v<T, Vector> && !std::is_same_v<T, Matrix>) {
			std::cout << "SymX error: fem_interpolation<T>() got not symx type as input (Scalar, Vector, Matrix)." << std::endl;
			exit(-1);
		}

		auto summation = [](std::vector<Scalar>& Nh, std::vector<T>& vh)
		{
			T x = Nh[0] * vh[0];
			for (int i = 1; i < (int)Nh.size(); i++) {
				x += Nh[i] * vh[i];
			}
			return x;
		};


		if (element == FEM_Element::Tet4) {
			if (vh.size() != 4) {
				std::cout << "symx error: fem_interpolation() got arrays of size != 4 for Tet4." << std::endl;
				exit(-1);
			}

			std::vector<Scalar> Nh = { 1.0 - xi[0] - xi[1] - xi[2],
				xi[0],
				xi[1],
				xi[2] };
			return summation(Nh, vh);
		}
		else if (element == FEM_Element::Tet10) {
			if (vh.size() != 10) {
				std::cout << "symx error: fem_interpolation() got arrays of size != 10 for Tet10." << std::endl;
				exit(-1);
			}

			Scalar N0 = 1.0 - xi[0] - xi[1] - xi[2];
			Scalar N1 = xi[0];
			Scalar N2 = xi[1];
			Scalar N3 = xi[2];
			std::vector<Scalar> Nh = { 
				N0 * (2.0 * N0 - 1.0),
				N1 * (2.0 * N1 - 1.0),
				N2 * (2.0 * N2 - 1.0),
				N3 * (2.0 * N3 - 1.0),
				4.0 * N0 * N1,
				4.0 * N1 * N2,
				4.0 * N2 * N0,
				4.0 * N0 * N3,
				4.0 * N1 * N3,
				4.0 * N2 * N3 };
			return summation(Nh, vh);
		}
		else if (element == FEM_Element::Hex8) {
			if (vh.size() != 8) {
				std::cout << "symx error: fem_interpolation() got arrays of size != 8 for Hex8." << std::endl;
				exit(-1);
			}
			Scalar NXm = 0.5 * (1.0 - xi[0]);
			Scalar NXp = 0.5 * (1.0 + xi[0]);
			Scalar NYm = 0.5 * (1.0 - xi[1]);
			Scalar NYp = 0.5 * (1.0 + xi[1]);
			Scalar NZm = 0.5 * (1.0 - xi[2]);
			Scalar NZp = 0.5 * (1.0 + xi[2]);
			std::vector<Scalar> Nh = { 
				NXm * NYm * NZm,
				NXp * NYm * NZm,
				NXp * NYp * NZm,
				NXm * NYp * NZm,
				NXm * NYm * NZp,
				NXp * NYm * NZp,
				NXp * NYp * NZp,
				NXm * NYp * NZp };
			return summation(Nh, vh);
		}
		else if (element == FEM_Element::Hex27) {
			if (vh.size() != 27) {
				std::cout << "symx error: fem_interpolation() got arrays of size != 27 for Hex27." << std::endl;
				exit(-1);
			}

			auto N1m = [](const Scalar& x) { return x * (x - 1.0); };
			auto N1p = [](const Scalar& x) { return x * (x + 1.0); };
			auto N2 = [](const Scalar& x) { return (1.0 - x.powN(2)); };
			const Scalar& x = xi[0];
			const Scalar& y = xi[1];
			const Scalar& z = xi[2];

			std::vector<Scalar> Nh = {
				// Corner nodes
				N1m(x)*N1m(y)*N1m(z)/8.0,
				N1p(x)*N1m(y)*N1m(z)/8.0,
				N1p(x)*N1p(y)*N1m(z)/8.0,
				N1m(x)*N1p(y)*N1m(z)/8.0,
				N1m(x)*N1m(y)*N1p(z)/8.0,
				N1p(x)*N1m(y)*N1p(z)/8.0,
				N1p(x)*N1p(y)*N1p(z)/8.0,
				N1m(x)*N1p(y)*N1p(z)/8.0,

				// Mid-edge nodes
				N2(x)*N1m(y)*N1m(z)/4.0,
				N1p(x)*N2(y)*N1m(z)/4.0,
				N2(x)*N1p(y)*N1m(z)/4.0,
				N1m(x)*N2(y)*N1m(z)/4.0,
				N2(x)*N1m(y)*N1p(z)/4.0,
				N1p(x)*N2(y)*N1p(z)/4.0,
				N2(x)*N1p(y)*N1p(z)/4.0,
				N1m(x)*N2(y)*N1p(z)/4.0,
				N1m(x)*N1m(y)*N2(z)/4.0,
				N1p(x)*N1m(y)*N2(z)/4.0,
				N1p(x)*N1p(y)*N2(z)/4.0,
				N1m(x)*N1p(y)*N2(z)/4.0,

				// Mid-face nodes
				N1m(x)*N2(y)*N2(z)/2.0,
				N1p(x)*N2(y)*N2(z)/2.0,
				N2(x)*N1p(y)*N2(z)/2.0,
				N2(x)*N1m(y)*N2(z)/2.0,
				N2(x)*N2(y)*N1m(z)/2.0,
				N2(x)*N2(y)*N1p(z)/2.0,

				// Mid-volume node
				N2(x)*N2(y)*N2(z)
			};
			return summation(Nh, vh);
		}
		else {
			std::cout << "symx error: fem_interpolation() got unhandled element type." << std::endl;
			exit(-1);
		}
	}
}
