#include "fem_integrators.h"

#include "../symbol/diff.h"

using namespace symx;

std::string symx::get_name(const FEM_Element& element)
{
	switch (element)
	{
	case FEM_Element::Tet4:
		return "Tet4";
	case FEM_Element::Tet10:
		return "Tet10";
	case FEM_Element::Hex8:
		return "Hex8";
	case FEM_Element::Hex27:
		return "Hex27";
	default:
		std::cout << "symx error: get_name() found unknown FEM_Element type." << std::endl;
		exit(-1);
	}
}
int symx::get_stride(const FEM_Element& element)
{
	switch (element)
	{
	case FEM_Element::Tet4:
		return 4;
	case FEM_Element::Tet10:
		return 10;
	case FEM_Element::Hex8:
		return 8;
	case FEM_Element::Hex27:
		return 27;
	default:
		std::cout << "symx error: get_name() found unknown FEM_Element type." << std::endl;
		exit(-1);
	}
}
Matrix symx::fem_jacobian(const FEM_Element& element, std::vector<Vector>& xh, const Vector& xi)
{
	Vector x = fem_interpolation(element, xh, xi);
	std::vector<Scalar> values;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			values.push_back(diff(x[i], xi[j]));
		}
	}
	return Matrix(values, { 3, 3 });
}
std::vector<std::array<double, 4>> symx::get_integration_rule(const FEM_Element& element)
{
	if (element == FEM_Element::Tet4) {
		return 	
			{
				{1.0/6.0, 0.25, 0.25, 0.25}
			};
	}
	else if (element == FEM_Element::Tet10) {
		return 	
			{
				{0.041666667, 0.58541020, 0.13819660, 0.13819660},
				{0.041666667, 0.13819660, 0.58541020, 0.13819660},
				{0.041666667, 0.13819660, 0.13819660, 0.58541020},
				{0.041666667, 0.13819660, 0.13819660, 0.13819660}
			};
	}
	else if (element == FEM_Element::Hex8) {
		double p = 1.0 / std::sqrt(3);
		return 	
			{
				{1.0, -p, -p, -p},
				{1.0, p, -p, -p},
				{1.0, p, p, -p},
				{1.0, -p, p, -p},
				{1.0, -p, -p, p},
				{1.0, p, -p, p},
				{1.0, p, p, p},
				{1.0, -p, p, p}
			};
	}
	else if (element == FEM_Element::Hex27) {
		double p = 0.7745966692;
		return
		{
			{ 0.1714677641, -p, -p, -p }, // 1
			{ 0.1714677641, p, -p, -p },
			{ 0.1714677641, -p, p, -p },
			{ 0.1714677641, p, p, -p },
			{ 0.1714677641, -p, -p, p },
			{ 0.1714677641, p, -p, p },
			{ 0.1714677641, -p, p, p },
			{ 0.1714677641, p, p, p },
			{ 0.2743484225, 0.0, -p, -p },  // 9
			{ 0.2743484225, -p, 0.0, -p },
			{ 0.2743484225, p, 0.0, -p },
			{ 0.2743484225, 0.0, p, -p },
			{ 0.2743484225, -p, -p, 0.0 },
			{ 0.2743484225, p, -p, 0.0 },
			{ 0.2743484225, -p, p, 0.0 },
			{ 0.2743484225, p, p, 0.0 },
			{ 0.2743484225, 0.0, -p, p }, // 17
			{ 0.2743484225, -p, 0.0, p },
			{ 0.2743484225, p, 0.0, p },
			{ 0.2743484225, 0.0, p, p },
			{ 0.4389574760, 0.0, 0.0, -p }, // 21
			{ 0.4389574760, 0.0, -p, 0.0 },  
			{ 0.4389574760, -p, 0.0, 0.0 },
			{ 0.4389574760, p, 0.0, 0.0 },
			{ 0.4389574760, 0.0, p, 0.0 },
			{ 0.4389574760, 0.0, 0.0, p },
			{ 0.7023319616, 0.0, 0.0, 0.0 },  // 27
		};
	}
	else {
		std::cout << "symx error: get_integration_rule() got unhandled element type." << std::endl;
		exit(-1);
	}
}
Scalar symx::fem_integrator(const FEM_Element& element, MappedWorkspace<double>& mws, std::function<Scalar(Scalar& w, Vector& xi)> summand)
{
	std::vector<std::array<double, 4>> gp = get_integration_rule(element);
	Scalar summation = mws.add_for_each(gp,
		[&](Vector& rule)
		{
			Scalar w = rule[0];
			Vector xi = Vector({ rule[1], rule[2], rule[3] });
			return summand(w, xi);
		}
	);
	return summation;
}
