#pragma once
#include <vector>
#include <limits>
#include <new>
#ifndef _MSC_VER
#include <cstdlib>
#endif

namespace symx::simd
{
	// Aligned allocator so that vectorized types can be used in std containers
	// Modernized C++17 version
	template <typename T, std::size_t N = 32>
	struct AlignmentAllocator {
		using value_type = T;
		using size_type = std::size_t;
		using difference_type = std::ptrdiff_t;
		using pointer = T*;
		using const_pointer = const T*;
		using reference = T&;
		using const_reference = const T&;

		template <typename U>
		struct rebind {
			using other = AlignmentAllocator<U, N>;
		};

		AlignmentAllocator() = default;

		template <typename T2>
		constexpr AlignmentAllocator(const AlignmentAllocator<T2, N>&) noexcept {}

		[[nodiscard]] T* allocate(std::size_t n) {
			if (n > std::numeric_limits<std::size_t>::max() / sizeof(T))
				throw std::bad_alloc();
#ifdef _MSC_VER
			if (auto p = static_cast<T*>(_aligned_malloc(n * sizeof(T), N))) return p;
#else
			const auto req_size = n * sizeof(T);
			const auto alignment_size = (req_size / N) * N;
			const auto alloc_size = alignment_size < req_size ? alignment_size + N : alignment_size;
			if (auto p = static_cast<T*>(std::aligned_alloc(N, alloc_size))) return p;
#endif
			throw std::bad_alloc();
		}

		void deallocate(T* p, std::size_t) noexcept {
#ifdef _MSC_VER
			_aligned_free(p);
#else
			std::free(p);
#endif
		}

		bool operator==(const AlignmentAllocator&) const { return true; }
		bool operator!=(const AlignmentAllocator&) const { return false; }
	};

	template<typename T, size_t SIZE>
	using avector = std::vector<T, AlignmentAllocator<T, SIZE>>;
}
