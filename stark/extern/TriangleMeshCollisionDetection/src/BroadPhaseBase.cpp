#include "BroadPhaseBase.h"

#include <omp.h>

void tmcd::internals::BroadPhaseBase::set_n_threads(const int32_t n_threads)
{
	this->n_threads = n_threads;
}
int tmcd::internals::BroadPhaseBase::get_n_threads() const
{
	return this->n_threads;
}
void tmcd::internals::BroadPhaseBase::set_recursion_cap(const int32_t cap)
{
	this->octree.set_recursion_cap(cap);
}
void tmcd::internals::BroadPhaseBase::set_max_recursion(const int32_t max_recursion)
{
	this->octree.set_max_recursion(max_recursion);
}
int32_t tmcd::internals::BroadPhaseBase::get_n_meshes() const
{
	return this->meshes.get_n_meshes();
}
void tmcd::internals::BroadPhaseBase::_init_threads()
{
	if (this->n_threads < 0) {
		this->n_threads = omp_get_max_threads() / 2;
	}
	this->thread_buffers.resize(this->n_threads);
}
