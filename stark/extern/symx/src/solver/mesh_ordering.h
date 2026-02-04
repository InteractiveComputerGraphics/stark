#pragma once
#include <vector>
#include <array>

namespace symx 
{
    struct ReorderResult 
    {
        std::vector<int> old_to_new;
        std::vector<int> new_to_old;
    };
    ReorderResult reorder_connectivity_RCM(int* conn, int conn_stride, int n_elements, int num_nodes, bool also_reorder_elements = true);
    std::vector<std::vector<int>> build_adjacency(int* conn, int conn_stride, int n_elements, int num_nodes);

    template<std::size_t N>
    ReorderResult reorder_connectivity_RCM(std::vector<std::array<int, N>>& conn, int num_nodes, bool also_reorder_elements = true)
    {
        if (conn.size() == 0) {
            return ReorderResult{ {}, {} };
        }
        return reorder_connectivity_RCM(const_cast<int*>(conn[0].data()), N, static_cast<int>(conn.size()), num_nodes, also_reorder_elements);
    }

    template <class T>
    inline void apply_permutation_inplace(std::vector<T>& a, const std::vector<int>& p)
    {
        auto orig = a;
        for (size_t old_i = 0; old_i < p.size(); ++old_i) {
            const size_t new_i = p[old_i];
            a[new_i] = orig[old_i];
        }
    }
}
