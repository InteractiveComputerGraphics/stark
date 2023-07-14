#pragma once
#include <array>
#include <cmath>

namespace tmcd
{
	// Small vector 3D class class
	template<typename FLOAT>
	class Vec3
	{
	public:
		std::array<FLOAT, 3> v;

		Vec3() {};
		template<typename FLOAT_I>
		Vec3(const FLOAT_I& x, const FLOAT_I& y, const FLOAT_I& z) { v[0] = static_cast<FLOAT>(x); v[1] = static_cast<FLOAT>(y); v[2] = static_cast<FLOAT>(z); }
		template<typename SIZE_T>
		const FLOAT& operator[](const SIZE_T& i) const { return v[i]; }
		template<typename SIZE_T>
		FLOAT& operator[](const SIZE_T& i) { return v[i]; }
		FLOAT dot(const Vec3& u) const { return v[0] * u[0] + v[1] * u[1] + v[2] * u[2]; }
		Vec3<FLOAT> cross(const Vec3& u) const { return Vec3(v[1] * u[2] - v[2] * u[1], -v[0] * u[2] + v[2] * u[0], v[0] * u[1] - v[1] * u[0]); }
		Vec3<FLOAT> operator+(const Vec3& u) const { return Vec3(v[0] + u[0], v[1] + u[1], v[2] + u[2]); }
		Vec3<FLOAT> operator-(const Vec3& u) const { return Vec3(v[0] - u[0], v[1] - u[1], v[2] - u[2]); }
		void operator+=(const Vec3& u) { v[0] += u[0]; v[1] += u[1]; v[2] += u[2]; }
		template<typename FLOAT_I>
		Vec3<FLOAT> operator*(const FLOAT_I& a) const { return Vec3(static_cast<FLOAT>(a) * v[0], static_cast<FLOAT>(a) * v[1], static_cast<FLOAT>(a) * v[2]); }
		template<typename FLOAT_I>
		Vec3<FLOAT> operator/(const FLOAT_I& a) const { return Vec3(v[0] / static_cast<FLOAT>(a), v[1] / static_cast<FLOAT>(a), v[2] / static_cast<FLOAT>(a)); }
		template<typename FLOAT_I>
		void operator/=(const FLOAT_I& a) { v[0] /= static_cast<FLOAT>(a); v[1] /= static_cast<FLOAT>(a); v[2] /= static_cast<FLOAT>(a); }
		FLOAT squaredNorm() const { return this->dot(*this); }
		FLOAT norm() const { return std::sqrt(this->squaredNorm()); }
		Vec3<FLOAT> normalized() const { return (*this) / this->norm(); }
		void normalize() { const FLOAT norm = this->norm(); v[0] /= norm; v[1] /= norm; v[2] /= norm; }
	};
	template<typename FLOAT, typename FLOAT_I>
	static inline Vec3<FLOAT> operator*(const FLOAT_I& a, const Vec3<FLOAT>& v) { return Vec3<FLOAT>(static_cast<FLOAT>(a) * v[0], static_cast<FLOAT>(a) * v[1], static_cast<FLOAT>(a) * v[2]); }
	using Vec3d = Vec3<double>;
	using Vec3f = Vec3<float>;
}