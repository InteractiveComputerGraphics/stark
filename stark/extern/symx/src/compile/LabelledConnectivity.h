#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <string>

#include "Element.h"


namespace symx
{
	/*
	 * A connectivity table with N named entries per element/stencil/row.
	 *
	 * Typical use:
	 *   LabelledConnectivity<4> conn({"elem_idx", "v0", "v1", "v2"});
	 *   conn.numbered_push_back({v0, v1, v2});  // prepends element index automatically
	 *   auto [mws, elem] = MappedWorkspace<double>::create(conn);
	 *   auto x = mws->make_vector(positions, elem["v0"]);
	 */
	template<std::size_t N>
	struct LabelledConnectivity
	{
		/* Fields */
		std::array<std::string, N> labels;
		std::vector<std::array<int32_t, N>> data;

		/* Methods */
		LabelledConnectivity(const std::array<std::string, N>& labels) : labels(labels) {};
		inline Element get_element() const 
		{ 
			Element elem((int)this->labels.size());
			elem.set_labels(std::vector<std::string>(this->labels.begin(), this->labels.end()));
			return elem;
		};

		inline std::array<int32_t, N>& operator[](const int idx) { return this->data[idx]; };
		inline const std::array<int32_t, N>& operator[](const int idx) const { return this->data[idx]; };
		inline void push_back(const std::array<int32_t, N>& v) { this->data.push_back(v); };
		inline void clear() { this->data.clear(); };
		inline std::size_t size() const { return this->data.size(); };
		inline int32_t get_label_idx(const std::string& label) const 
		{
			for (size_t i = 0; i < N; i++) {
				if (this->labels[i] == label) {
					return (int32_t)i;
				}
			}
			
			// Error
			std::cout << "symx error: LabelledConnectivity.get_label_idx() Label not found" << std::endl;
			exit(-1);
		};
		inline std::array<int32_t, N> get_label_indices(const std::array<std::string, N>& labels) const
		{
			std::array<int32_t, N> indices;
			for (size_t i = 0; i < N; i++) {
				indices[i] = this->get_label_idx(labels[i]);
			}
			return indices;
		};
		template<std::size_t M>
		inline std::array<int32_t, M> get(const int idx, const std::array<int32_t, M>& entries) const
		{
			std::array<int32_t, M> v;
			for (size_t i = 0; i < M; i++) {
				v[i] = this->data[idx][entries[i]];
			}
			return v;
		};
		// Push a row without an element index, prepending the current row count as column 0.
		// Useful when the first column of the connectivity is the element's own id.
		inline int32_t numbered_push_back(const std::array<int32_t, N-1>& v)
		{ 
			std::array<int32_t, N> v_;
			v_[0] = (int32_t)this->data.size();
			for (size_t i = 0; i < N-1; i++) {
				v_[i + 1] = v[i];
			}
			this->push_back(v_);
			return v_[0];
		};
	};
}