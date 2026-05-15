#include "utils.h"

using namespace symx;

std::vector<Scalar> symx::collect_scalars(const std::vector<Vector>& vectors)
{
	std::vector<Scalar> scalars;
	for (const Vector& v : vectors) {
		for (const Scalar& s : v.values()) {
			scalars.push_back(s);
		}
	}
	return scalars;
}

std::vector<Scalar> symx::collect_scalars(const std::vector<std::vector<Scalar>> &vectors)
{
	std::vector<Scalar> scalars;
	for (const std::vector<Scalar>& v : vectors) {
		for (const Scalar& s : v) {
			scalars.push_back(s);
		}
	}
	return scalars;
}

std::string symx::get_checksum(const std::vector<Scalar> &exprs)
{
	std::string cache_id;
	for (const Scalar& s : exprs) {
		cache_id += s.get_checksum();
	} 
	return cache_id;
}
