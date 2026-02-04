#pragma once
#include <vector>
#include <array>
#include <iostream>

#ifdef __linux__
#include <malloc.h>
#endif

namespace symx
{
	/**
	* Collection of fixed sized dynamically allocated memory chunks, used
	* to store the neighbor lists.
	* It's purpose it to avoid the reallocations that would happen by using
	* an std::vector<> when the capacity is reached. In chunked_vector<>,
	* another memory chunk is allocated but no reallocations occur.
	*/
	template<typename T, size_t CHUNKSIZE = 10000>
	class chunked_vector
	{
	private:
		/* Fields */
		// Each chunk holds a pair: [begin pointer, current "cursor" pointer]
		std::vector<std::array<T*, 2>> chunks;
		int current = 0; // Currect chunk in use

	public:
		/* Methods */
		chunked_vector(const chunked_vector<T, CHUNKSIZE>& other) = delete;       // Non-copyable
		chunked_vector(chunked_vector<T, CHUNKSIZE>&& other) noexcept = default;  // Movable
		chunked_vector()
		{
			T* ptr = new T[CHUNKSIZE];
			this->chunks.push_back({ ptr, ptr });
			this->current = 0;
		};
		~chunked_vector()
		{
			for (std::array<T*, 2>&chunk : this->chunks) {
				delete[] chunk[0];
			}
		}
		static constexpr std::size_t chunk_size() noexcept
		{
			return CHUNKSIZE;
		}
		T* get_cursor_with_space_to_write(const size_t n)
		{
			if (n > CHUNKSIZE) {
				std::cout << "chunked_vector compare: Cannot allow_to_append n > CHUNKSIZE (" << n << " < " << CHUNKSIZE << ")." << std::endl;
				exit(-1);
			}

			const size_t space_left = CHUNKSIZE - (size_t)std::distance(this->chunks[this->current][0], this->chunks[this->current][1]) - 1;
			if (n > space_left) {

				// No space left in the current chunk. Move to the next one.
				this->current++;

				// If the next chunk does not exist, create it
				if (this->chunks.size() == this->current) {
					T* ptr = new T[CHUNKSIZE];
					this->chunks.push_back({ ptr, ptr });
				}

				// Reset the cursor to the beginning of the chunk
				this->chunks[this->current][1] = this->chunks[this->current][0];
			}

			// Cursor logic
			T* return_cursor = this->chunks[this->current][1];
			this->chunks[this->current][1] += n;

			return return_cursor;
		}
		void clear()
		{
			this->chunks[0][1] = this->chunks[0][0];
			this->current = 0;
		}
		size_t n_bytes() const
		{
			return this->chunks.size() * CHUNKSIZE * static_cast<size_t>(sizeof(T));
		}
	};
}
