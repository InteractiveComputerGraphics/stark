#pragma once
#include <array>
#include <cstdlib>

namespace tmcd
{
	namespace internals
	{
		template<typename NODE_BUFFER, size_t N>
		class RecursiveBuffer
		{
		public:
			// Fiedls
			NODE_BUFFER buffer;
			RecursiveBuffer* parent = nullptr;
			std::array<RecursiveBuffer*, N> children;

			// Methods
			inline RecursiveBuffer()
			{
				for (int i = 0; i < N; i++) {
					this->children[i] = nullptr;
				}
			};
			inline ~RecursiveBuffer()
			{
				this->delete_children();
			};
			inline RecursiveBuffer& get_child(const size_t i)
			{
				if (this->children[i] == nullptr) {
					this->children[i] = new RecursiveBuffer();
					this->children[i]->parent = this;
				}
				return *this->children[i];
			};
			inline void populate_children()
			{
				if (this->children[0] == nullptr) {
					for (int i = 0; i < N; i++) {
						this->children[i] = new RecursiveBuffer();
						this->children[i]->parent = this;
					}
				}
			}
			inline void delete_children()
			{
				if (this->children[0] != nullptr) {
					for (int i = 0; i < N; i++) {
						delete this->children[i];
						this->children[i] = nullptr;
					}
				}
			}
		};
	}
}