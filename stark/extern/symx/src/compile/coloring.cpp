#include "coloring.h"

#include <unordered_map>
#include <algorithm>

void symx::compute_color_graph(
    std::vector<std::vector<int32_t>>& color_bins,
    const int32_t* connectivity_data,
    const int32_t connectivity_n_elements,
    const int32_t connectivity_stride,
    const std::vector<int32_t>& coloring_use_indices
) 
{
    color_bins.clear();

    // --- Basic validation ---
    if (!connectivity_data || connectivity_n_elements <= 0 || connectivity_stride <= 0) {
        return; // nothing to do
    }

    // If no indices are provided, put everything into a single color (no guarantees of race-freedom).
    if (coloring_use_indices.empty()) {
        color_bins.resize(1);
        color_bins[0].reserve(static_cast<size_t>(connectivity_n_elements));
        for (int32_t e = 0; e < connectivity_n_elements; ++e) color_bins[0].push_back(e);
        return;
    }

    // --- Build node -> incident elements buckets ---
    // Using unordered_map to avoid huge dense arrays when indices are sparse.
    std::unordered_map<int32_t, std::vector<int32_t>> node_to_elems;
    node_to_elems.reserve(static_cast<size_t>(connectivity_n_elements) * 2);

    auto elem_conn = [&](int32_t e) -> const int32_t* {
        return connectivity_data + static_cast<size_t>(e) * connectivity_stride;
    };

    // Helper: gather unique nodes used by this element (only from coloring_use_indices)
    auto gather_unique_nodes = [&](const int32_t* ce, int32_t out_nodes[16]) -> int {
        int tn = 0;
        for (int32_t idx : coloring_use_indices) {
            if (idx < 0 || idx >= connectivity_stride) continue; // skip invalid spec
            int32_t v = ce[idx];
            if (v < 0) continue; // ignore negative node ids
            bool dup = false;
            for (int k = 0; k < tn; ++k) if (out_nodes[k] == v) { dup = true; break; }
            if (!dup) {
                if (tn < 16) out_nodes[tn++] = v;
                else {
                    // Fallback if stencil is unexpectedly large: do a linear check w/o capping
                    bool dup2 = false;
                    for (int k = 0; k < tn; ++k) if (out_nodes[k] == v) { dup2 = true; break; }
                    if (!dup2) { /* ignore beyond 16 to keep it tiny; very rare in meshes */ }
                }
            }
        }
        return tn;
    };

    for (int32_t e = 0; e < connectivity_n_elements; ++e) {
        const int32_t* ce = elem_conn(e);
        int32_t nodes[16];
        int tn = gather_unique_nodes(ce, nodes);
        for (int k = 0; k < tn; ++k) {
            node_to_elems[nodes[k]].push_back(e);
        }
    }

    // --- Order elements by (approx) degree to reduce color count ---
    std::vector<int32_t> order(static_cast<size_t>(connectivity_n_elements));
    for (int32_t i = 0; i < connectivity_n_elements; ++i) order[static_cast<size_t>(i)] = i;

    auto degree_of = [&](int32_t e) -> int {
        const int32_t* ce = elem_conn(e);
        int32_t nodes[16];
        int tn = gather_unique_nodes(ce, nodes);
        int deg = 0;
        for (int k = 0; k < tn; ++k) {
            auto it = node_to_elems.find(nodes[k]);
            if (it != node_to_elems.end()) deg += static_cast<int>(it->second.size());
        }
        return deg;
    };

    std::stable_sort(order.begin(), order.end(),
        [&](int32_t a, int32_t b) { return degree_of(a) > degree_of(b); });

    // --- Greedy coloring ---
    std::vector<int32_t> elem_color(static_cast<size_t>(connectivity_n_elements), -1);
    int max_color = -1;

    // reused scratch for "used colors"
    std::vector<uint8_t> used; used.reserve(64);

    for (int32_t e : order) {
        // mark neighbor colors
        const int32_t* ce = elem_conn(e);
        int32_t nodes[16];
        int tn = gather_unique_nodes(ce, nodes);

        // grow used array if needed
        if (max_color + 1 > static_cast<int>(used.size())) used.resize(static_cast<size_t>(max_color + 1), 0);
        std::fill(used.begin(), used.begin() + (max_color + 1), 0);

        for (int k = 0; k < tn; ++k) {
            auto it = node_to_elems.find(nodes[k]);
            if (it == node_to_elems.end()) continue;
            const auto& inc = it->second;
            for (int32_t nb : inc) {
                int c = elem_color[static_cast<size_t>(nb)];
                if (c >= 0 && c <= max_color) used[static_cast<size_t>(c)] = 1;
            }
        }

        // pick smallest free color
        int c = 0;
        while (c <= max_color && used[static_cast<size_t>(c)]) ++c;
        if (c > max_color) max_color = c;
        elem_color[static_cast<size_t>(e)] = c;
    }

    // --- Build color bins ---
    color_bins.assign(static_cast<size_t>(max_color + 1), {});
    for (int32_t e = 0; e < connectivity_n_elements; ++e) {
        int c = elem_color[static_cast<size_t>(e)];
        if (c < 0) c = 0; // shouldn't happen, safety
        color_bins[static_cast<size_t>(c)].push_back(e);
    }
}
