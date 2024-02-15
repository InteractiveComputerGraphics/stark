#include "EnergyTriangleStrain.h"

#include "../time_integration.h"
#include "../../utils/mesh_utils.h"


symx::Scalar largest_eigenvalue_sym_2x2(const symx::Matrix& A)
{
	// https://hal.science/hal-01501221/document
	const symx::Scalar& a = A(0, 0);
	const symx::Scalar& b = A(1, 1);
	const symx::Scalar& c = A(0, 1);

	const symx::Scalar delta = symx::sqrt(4.0*c.powN(2) + (a - b).powN(2));
	return 0.5 * (a + b + delta);
}

stark::models::EnergyTriangleStrain::EnergyTriangleStrain(stark::core::Stark& stark, spPointDynamics dyn)
	: dyn(dyn)
{
	stark.global_energy.add_energy("EnergyTriangleStrain_D_SL", this->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Unpack connectivity
			std::vector<symx::Index> triangle = conn.slice(2, 5);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, triangle);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, triangle);
			symx::Matrix DXinv = energy.make_matrix(this->DXinv, { 2, 2 }, conn["idx"]);
			symx::Scalar rest_area = energy.make_scalar(this->triangle_area_rest, conn["idx"]);
			symx::Scalar thickness = energy.make_scalar(this->thickness, conn["group"]);
			symx::Scalar e = energy.make_scalar(this->young_modulus, conn["group"]);
			symx::Scalar nu = energy.make_scalar(this->poisson_ratio, conn["group"]);
			symx::Scalar strain_damping = energy.make_scalar(this->strain_damping, conn["group"]);
			symx::Scalar strain_limit = energy.make_scalar(this->strain_limit, conn["group"]);
			symx::Scalar strain_limit_stiffness = energy.make_scalar(this->strain_limit_stiffness, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Kinematics
			symx::Matrix Dx1_32 = symx::Matrix(symx::gather({ x1[1] - x1[0], x1[2] - x1[0] }), { 2, 3 }).transpose();
			symx::Matrix F1_32 = Dx1_32 * DXinv;  // 3x2
			symx::Matrix C1 = F1_32.transpose() * F1_32;
			symx::Matrix E1 = 0.5*(C1 - energy.make_identity_matrix(2));

			symx::Matrix Dx0_32 = symx::Matrix(symx::gather({ x0[1] - x0[0], x0[2] - x0[0] }), { 2, 3 }).transpose();
			symx::Matrix F0_32 = Dx0_32 * DXinv;  // 3x2
			symx::Matrix E0 = 0.5*(F0_32.transpose() * F0_32 - energy.make_identity_matrix(2));

			symx::Matrix dE_dt = (E1 - E0) / dt;

			// Hardening for strain limiting
			symx::Scalar s1 = largest_eigenvalue_sym_2x2(E1);
			symx::Scalar dl = s1 - (strain_limit + 1.0);
			e += symx::branch(dl > 0.0, strain_limit_stiffness * dl.powN(3) / 3.0, 0.0);

			// Stable Neo-Hookean strain energy
			symx::Scalar mu = e / (2.0 * (1.0 + nu));
			symx::Scalar lambda = (e * nu) / ((1.0 + nu) * (1.0 - nu));  // 2D !!
			symx::Scalar area = 0.5 * ((x1[0] - x1[2]).cross3(x1[1] - x1[2])).norm();
			symx::Scalar J = area / rest_area;
			symx::Scalar Ic = C1.trace();
			symx::Scalar logJ = symx::log(J);
			symx::Scalar elastic_energy_density = 0.5*mu*(Ic - 2.0) - mu*logJ + 0.5*lambda*logJ.powN(2);

			// Damping
			symx::Scalar damping_energy_density = 0.5 * strain_damping * dE_dt.frobenius_norm_sq();

			// Total
			symx::Scalar Energy = thickness * rest_area * (elastic_energy_density + damping_energy_density);
			energy.set(Energy);
		}
	);
}
void stark::models::EnergyTriangleStrain::add(Id& id, const std::vector<std::array<int, 3>>& triangles, const double thickness, const double young_modulus, const double poisson_ratio, const double strain_damping, const double strain_limit, const double strain_limit_stiffness)
{
	if (strain_limit < std::numeric_limits<double>::epsilon()) {
		std::cout << "stark error: strain_limit must be larger than epsilon." << std::endl;
		exit(-1);
	}

	const int group = (int)this->thickness.size();
	this->thickness.push_back(thickness);
	this->young_modulus.push_back(young_modulus);
	this->poisson_ratio.push_back(poisson_ratio);
	this->strain_damping.push_back(strain_damping);
	this->strain_limit.push_back(strain_limit);
	this->strain_limit_stiffness.push_back(strain_limit_stiffness);

	// Initialize structures
	for (int tri_i = 0; tri_i < (int)triangles.size(); tri_i++) {

		// Connectivity
		const std::array<int, 3>& conn = triangles[tri_i];
		const std::array<int, 3> conn_glob = this->dyn->X.get_global_indices(id.get_global_idx(), conn);
		this->conn.numbered_push_back({ group, conn_glob[0], conn_glob[1], conn_glob[2] });

		// Fetch coordinates
		const Eigen::Vector3d& A = this->dyn->X[conn_glob[0]];
		const Eigen::Vector3d& B = this->dyn->X[conn_glob[1]];
		const Eigen::Vector3d& C = this->dyn->X[conn_glob[2]];

		// Area
		this->triangle_area_rest.push_back(utils::triangle_area(A, B, C));

		// DXinv
		//// Projection matrix
		const Eigen::Vector3d u = (B - A).normalized();
		const Eigen::Vector3d n = u.cross(C - A);
		const Eigen::Vector3d v = u.cross(n).normalized();
		Eigen::Matrix<double, 2, 3> P;
		P.row(0) = u;
		P.row(1) = v;

		//// Projection
		const Eigen::Vector2d A_ = P*A;
		const Eigen::Vector2d B_ = P*B;
		const Eigen::Vector2d C_ = P*C;

		//// DX
		Eigen::Matrix2d DX;
		DX.col(0) = B_ - A_;
		DX.col(1) = C_ - A_;
		Eigen::Matrix2d DXinv = DX.inverse();
		this->DXinv.push_back({DXinv(0, 0), DXinv(0, 1), DXinv(1, 0), DXinv(1, 1)});
	}

	id.set_local_idx("EnergyTriangleStrain", group);
}

void stark::models::EnergyTriangleStrain::set_thickness(const Id& id, const double thickness)
{
	this->thickness[this->get_index(id)] = thickness;
}
void stark::models::EnergyTriangleStrain::set_young_modulus(const Id& id, const double young_modulus)
{
	this->young_modulus[this->get_index(id)] = young_modulus;
}
void stark::models::EnergyTriangleStrain::set_poisson_ratio(const Id& id, const double poisson_ratio)
{
	this->poisson_ratio[this->get_index(id)] = poisson_ratio;
}
void stark::models::EnergyTriangleStrain::set_strain_damping(const Id& id, const double strain_damping)
{
	this->strain_damping[this->get_index(id)] = strain_damping;
}
void stark::models::EnergyTriangleStrain::set_strain_limit(const Id& id, const double strain_limit)
{
	if (strain_limit < std::numeric_limits<double>::epsilon()) {
		std::cout << "stark error: strain_limit must be larger than epsilon." << std::endl;
		exit(-1);
	}

	this->strain_limit[this->get_index(id)] = strain_limit;
}
void stark::models::EnergyTriangleStrain::set_strain_limit_stiffness(const Id& id, const double strain_limit_stiffness)
{
	this->strain_limit_stiffness[this->get_index(id)] = strain_limit_stiffness;
}
double stark::models::EnergyTriangleStrain::get_thickness(const Id& id)
{
	return this->thickness[this->get_index(id)];
}
double stark::models::EnergyTriangleStrain::get_young_modulus(const Id& id)
{
	return this->young_modulus[this->get_index(id)];
}
double stark::models::EnergyTriangleStrain::get_poisson_ratio(const Id& id)
{
	return this->poisson_ratio[this->get_index(id)];
}
double stark::models::EnergyTriangleStrain::get_strain_damping(const Id& id)
{
	return this->strain_damping[this->get_index(id)];
}
double stark::models::EnergyTriangleStrain::get_strain_limit(const Id& id)
{
	return this->strain_limit[this->get_index(id)];
}
double stark::models::EnergyTriangleStrain::get_strain_limit_stiffness(const Id& id)
{
	return this->strain_limit_stiffness[this->get_index(id)];
}
int stark::models::EnergyTriangleStrain::get_index(const Id& id) const
{
	return id.get_local_idx("EnergyTriangleStrain");
}
