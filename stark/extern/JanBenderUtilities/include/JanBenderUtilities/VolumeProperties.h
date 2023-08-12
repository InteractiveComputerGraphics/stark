/**
 * The code in this file is mostly an adaption of
 *  https://people.eecs.berkeley.edu/~jfc/mirtich/massProps.html
 */

#include <cstdio>
#include <vector>
#include <array>

using namespace std;
using namespace Eigen;

#define SQR(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))

using RealVector3 = Eigen::Vector3d;
using Real = double;

namespace JanBenderUtilities
{
	class VolumeIntegration
	{

	private:
		/** alpha */
		int A;
		/** beta */
		int B;
		/** gamma */
		int C;

		/** projection integrals */
		Real P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;
		/** face integrals */
		Real Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;
		/** volume integrals */
		Real T0;
		std::array<Real, 3> T1, T2, TP;

	public:

		VolumeIntegration(std::vector<RealVector3> const& vertices, std::vector<std::array<int, 3>> const& triangles);

		void compute_mass_properties(Real density);

		Real mass() const { return m_mass; }
		Real volume() const { return m_volume; }
		Eigen::Matrix3d const& inertia() const { return m_theta; }
		RealVector3 const& center_of_mass() const { return m_r; }

	private:

		void volume_integrals();
		void face_integrals(int i);
		void projection_integrals(int i);


		std::vector<RealVector3> const& m_vertices;
		std::vector<std::array<int, 3>> const& m_triangles;

		std::vector<RealVector3> m_face_normals;
		std::vector<Real> m_weights;

		Real m_mass, m_volume;
		Eigen::Matrix3d m_theta;
		RealVector3 m_r;
	};

	inline VolumeIntegration::VolumeIntegration(vector<RealVector3> const& vertices, vector<array<int, 3>> const& triangles)
		: m_vertices(vertices), m_triangles(triangles), m_face_normals(triangles.size()), m_weights(triangles.size())
	{
		for (int i = 0u; i < triangles.size(); ++i)
		{
			auto const& f = triangles[i];
			RealVector3 d1 = vertices[f[1]] - vertices[f[0]];
			RealVector3 d2 = vertices[f[2]] - vertices[f[1]];
			m_face_normals[i] = d1.cross(d2);
			if (m_face_normals[i].isZero(1.e-10)) m_face_normals[i].setZero();
			else m_face_normals[i].normalize();

			m_weights[i] = -m_face_normals[i].dot(vertices[f[0]]);

		}
	}

	inline void VolumeIntegration::compute_mass_properties(Real density)
	{
		volume_integrals();
		m_volume = T0;

		m_mass = density * T0;

		/* compute center of mass */
		m_r[0] = T1[0] / T0;
		m_r[1] = T1[1] / T0;
		m_r[2] = T1[2] / T0;

		/* compute inertia tensor */
		m_theta(0, 0) = density * (T2[1] + T2[2]);
		m_theta(1, 1) = density * (T2[2] + T2[0]);
		m_theta(2, 2) = density * (T2[0] + T2[1]);
		m_theta(0, 1) = m_theta(1, 0) = -density * TP[0];
		m_theta(1, 2) = m_theta(2, 1) = -density * TP[1];
		m_theta(2, 0) = m_theta(0, 2) = -density * TP[2];

		/* move inertia tensor to center of mass */
		m_theta(0, 0) -= m_mass * (m_r[1] * m_r[1] + m_r[2] * m_r[2]);
		m_theta(1, 1) -= m_mass * (m_r[2] * m_r[2] + m_r[0] * m_r[0]);
		m_theta(2, 2) -= m_mass * (m_r[0] * m_r[0] + m_r[1] * m_r[1]);
		m_theta(0, 1) = m_theta(1, 0) += m_mass * m_r[0] * m_r[1];
		m_theta(1, 2) = m_theta(2, 1) += m_mass * m_r[1] * m_r[2];
		m_theta(2, 0) = m_theta(0, 2) += m_mass * m_r[2] * m_r[0];
	}

	/** Compute various integrations over projection of face
	*/
	inline void VolumeIntegration::projection_integrals(int f)
	{
		Real a0, a1, da;
		Real b0, b1, db;
		Real a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
		Real a1_2, a1_3, b1_2, b1_3;
		Real C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
		Real Cab, Kab, Caab, Kaab, Cabb, Kabb;

		P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

		for (int i = 0; i < 3; i++)
		{
			auto const& face = m_triangles[f];
			a0 = m_vertices[face[i]][A];
			b0 = m_vertices[face[i]][B];
			a1 = m_vertices[face[(i + 1) % 3]][A];
			b1 = m_vertices[face[(i + 1) % 3]][B];

			da = a1 - a0;
			db = b1 - b0;
			a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
			b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
			a1_2 = a1 * a1; a1_3 = a1_2 * a1;
			b1_2 = b1 * b1; b1_3 = b1_2 * b1;

			C1 = a1 + a0;
			Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
			Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
			Cab = 3 * a1_2 + 2 * a1*a0 + a0_2; Kab = a1_2 + 2 * a1*a0 + 3 * a0_2;
			Caab = a0*Cab + 4 * a1_3; Kaab = a1*Kab + 4 * a0_3;
			Cabb = 4 * b1_3 + 3 * b1_2*b0 + 2 * b1*b0_2 + b0_3;
			Kabb = b1_3 + 2 * b1_2*b0 + 3 * b1*b0_2 + 4 * b0_3;

			P1 += db*C1;
			Pa += db*Ca;
			Paa += db*Caa;
			Paaa += db*Caaa;
			Pb += da*Cb;
			Pbb += da*Cbb;
			Pbbb += da*Cbbb;
			Pab += db*(b1*Cab + b0*Kab);
			Paab += db*(b1*Caab + b0*Kaab);
			Pabb += da*(a1*Cabb + a0*Kabb);
		}

		P1 /= 2.0;
		Pa /= 6.0;
		Paa /= 12.0;
		Paaa /= 20.0;
		Pb /= -6.0;
		Pbb /= -12.0;
		Pbbb /= -20.0;
		Pab /= 24.0;
		Paab /= 60.0;
		Pabb /= -60.0;
	}

	inline void VolumeIntegration::face_integrals(int f)
	{
		Real w;
		RealVector3 n;
		Real k1, k2, k3, k4;

		projection_integrals(f);

		w = m_weights[f];
		n = m_face_normals[f];
		k1 = 1 / n[C]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

		Fa = k1 * Pa;
		Fb = k1 * Pb;
		Fc = -k2 * (n[A] * Pa + n[B] * Pb + w*P1);

		Faa = k1 * Paa;
		Fbb = k1 * Pbb;
		Fcc = k3 * (SQR(n[A])*Paa + 2 * n[A] * n[B] * Pab + SQR(n[B])*Pbb
			+ w*(2 * (n[A] * Pa + n[B] * Pb) + w*P1));

		Faaa = k1 * Paaa;
		Fbbb = k1 * Pbbb;
		Fccc = -k4 * (CUBE(n[A])*Paaa + 3 * SQR(n[A])*n[B] * Paab
			+ 3 * n[A] * SQR(n[B])*Pabb + CUBE(n[B])*Pbbb
			+ 3 * w*(SQR(n[A])*Paa + 2 * n[A] * n[B] * Pab + SQR(n[B])*Pbb)
			+ w*w*(3 * (n[A] * Pa + n[B] * Pb) + w*P1));

		Faab = k1 * Paab;
		Fbbc = -k2 * (n[A] * Pabb + n[B] * Pbbb + w*Pbb);
		Fcca = k3 * (SQR(n[A])*Paaa + 2 * n[A] * n[B] * Paab + SQR(n[B])*Pabb
			+ w*(2 * (n[A] * Paa + n[B] * Pab) + w*Pa));
	}

	inline void VolumeIntegration::volume_integrals()
	{
		Real nx, ny, nz;

		T0 = T1[0] = T1[1] = T1[2]
			= T2[0] = T2[1] = T2[2]
			= TP[0] = TP[1] = TP[2] = 0;

		for (int i = 0; i < m_triangles.size(); ++i)
		{
			RealVector3 const& n = m_face_normals[i];
			nx = std::abs(n[0]);
			ny = std::abs(n[1]);
			nz = std::abs(n[2]);
			if (nx > ny && nx > nz)
				C = 0;
			else
				C = (ny > nz) ? 1 : 2;
			A = (C + 1) % 3;
			B = (A + 1) % 3;

			face_integrals(i);

			T0 += n[0] * ((A == 0) ? Fa : ((B == 0) ? Fb : Fc));

			T1[A] += n[A] * Faa;
			T1[B] += n[B] * Fbb;
			T1[C] += n[C] * Fcc;
			T2[A] += n[A] * Faaa;
			T2[B] += n[B] * Fbbb;
			T2[C] += n[C] * Fccc;
			TP[A] += n[A] * Faab;
			TP[B] += n[B] * Fbbc;
			TP[C] += n[C] * Fcca;
		}

		T1[0] /= 2; T1[1] /= 2; T1[2] /= 2;
		T2[0] /= 3; T2[1] /= 3; T2[2] /= 3;
		TP[0] /= 2; TP[1] /= 2; TP[2] /= 2;
	}
	
	
	
	struct VolumeProperties
	{
		double volume;
		double mass;
		Eigen::Vector3d center_of_mass;
		Eigen::Matrix3d inertia_tensor;
	};
	
	inline VolumeProperties compute_volume_properties(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const double density)
	{
		VolumeIntegration volume_integrator(vertices, triangles);
		volume_integrator.compute_mass_properties(density);

		VolumeProperties result;
		result.volume = volume_integrator.volume();
		result.mass = volume_integrator.mass();
		result.inertia_tensor = volume_integrator.inertia();
		result.center_of_mass = volume_integrator.center_of_mass();
		return result;
	}
}

