#include "mesh_ordering.h"
#include <queue>
#include <algorithm>
#include <numeric>
#include <limits>
#include <cstddef>
#include <cstring>


// BFS returning levels and the furthest node visited
std::pair<std::vector<int>, int> bfs_levels(const std::vector<std::vector<int>>& adj, int src)
{
    const int n = static_cast<int>(adj.size());
    std::vector<int> level(n, -1);
    std::queue<int> q;
    q.push(src);
    level[src] = 0;
    int last = src;
    while (!q.empty()) {
        int u = q.front(); q.pop();
        last = u;
        for (int v : adj[u]) if (level[v] == -1) {
            level[v] = level[u] + 1;
            q.push(v);
        }
    }
    return {level, last};
}


// Reverse Cuthill–McKee ordering (RCM) that returns new_index_of_old (size n)
std::vector<int> rcm_order(const std::vector<std::vector<int>>& adj)
{
    const int n = static_cast<int>(adj.size());
    std::vector<int> old_to_new(n, -1);
    std::vector<char> visited(n, 0);
    std::vector<int> order;
    order.reserve(n);

    // Process each connected component
    for (int comp = 0; comp < n; ++comp) {
        if (visited[comp]) continue;
        int root = comp;
        // Prefer a pseudo-peripheral node as the BFS root for each component
        {
            // Find a node in this component (linear scan from comp)
            int any = -1;
            for (int i = comp; i < n; ++i) if (!visited[i]) { any = i; break; }
            if (any == -1) break;
            // Flood to mark the component and gather nodes
            std::vector<int> component; component.reserve(64);
            std::queue<int> q; q.push(any); visited[any] = 2; // 2 means discovered for comp
            while (!q.empty()) {
                int u = q.front(); q.pop();
                component.push_back(u);
                for (int v : adj[u]) if (!visited[v]) { visited[v] = 2; q.push(v); }
            }
            // reset to unvisited for RCM run, but keep the set (we'll gate by a membership flag)
            for (int u : component) visited[u] = 0;
            // build membership flag
            std::vector<char> in_comp(n, 0);
            for (int u : component) in_comp[u] = 1;
            // pick a pseudo-peripheral node within this component
            // simple local search: start from min-degree node in component
            int start = component[0];
            for (int u : component) if (adj[u].size() < adj[start].size()) start = u;
            // two sweeps to approximate pseudo-peripheral inside the subgraph
            auto far1 = bfs_levels(adj, start).second;
            auto far2 = bfs_levels(adj, far1).second;
            root = far2;
            // BFS from root with increasing degree neighbor ordering
            std::queue<int> qb; qb.push(root); visited[root] = 1;
            std::vector<int> level_order;
            while (!qb.empty()) {
                int u = qb.front(); qb.pop();
                level_order.push_back(u);
                // neighbors are already sorted by (degree, id) in build_adjacency
                for (int v : adj[u]) if (!visited[v] && in_comp[v]) { visited[v] = 1; qb.push(v); }
            }
            // Reverse for RCM
            std::reverse(level_order.begin(), level_order.end());
            order.insert(order.end(), level_order.begin(), level_order.end());
        }
    }

    // Assign new indices 0..n-1 according to the concatenated RCM order
    for (int new_idx = 0; new_idx < n; ++new_idx) old_to_new[order[new_idx]] = new_idx;
    return old_to_new;
}

std::vector<int> invert_permutation(const std::vector<int>& p)
{
    std::vector<int> inv(p.size());
    for (size_t i = 0; i < p.size(); ++i) inv[p[i]] = static_cast<int>(i);
    return inv;
}

// Apply a node permutation to the connectivity array (old->new mapping).
void apply_node_permutation_to_conn(int* conn, int conn_stride, int n_elements, const std::vector<int>& old_to_new)
{
    for (int i = 0; i < n_elements*conn_stride; ++i) {
        conn[i] = old_to_new[conn[i]];
    }
}

// Build an element order by BFS on the element dual graph
// (two elements are adjacent if they share at least one node).
std::vector<int> build_element_order_dual_bfs(int* conn, int conn_stride, int n_elements, int num_nodes)
{
    // incident elements per node
    std::vector<std::vector<int>> inc(num_nodes);
    for (int e = 0; e < n_elements; ++e) {
        const int* c = &conn[e*conn_stride];
        for (int i = 0; i < conn_stride; ++i) inc[c[i]].push_back(e);
    }

    // element adjacency (dual)
    std::vector<std::vector<int>> adj(n_elements);
    for (int n = 0; n < num_nodes; ++n) {
        const auto& el = inc[n];
        for (int i = 0; i < (int)el.size(); ++i) {
            int a = el[i];
            for (int j = i+1; j < (int)el.size(); ++j) {
                int b = el[j];
                adj[a].push_back(b);
                adj[b].push_back(a);
            }
        }
    }
    for (int e = 0; e < n_elements; ++e) {
        auto& nb = adj[e];
        std::sort(nb.begin(), nb.end());
        nb.erase(std::unique(nb.begin(), nb.end()), nb.end());
    }

    // degree + deterministic seed key
    std::vector<int> deg(n_elements), seed_key(n_elements);
    for (int e = 0; e < n_elements; ++e) {
        deg[e] = (int)adj[e].size();
        const int* c = &conn[e*conn_stride];
        int mn = c[0]; long long sum = 0;
        for (int i = 0; i < conn_stride; ++i) { mn = std::min(mn, c[i]); sum += c[i]; }
        seed_key[e] = (mn << 16) ^ int(sum / conn_stride);
    }

    std::vector<int> order; order.reserve(n_elements);
    std::vector<char> vis(n_elements, 0);

    for (;;) {
        // pick next unvisited seed with smallest (deg, seed_key)
        int seed = -1;
        for (int e = 0; e < n_elements; ++e) if (!vis[e]) {
            if (seed == -1 || deg[e] < deg[seed] || (deg[e] == deg[seed] && seed_key[e] < seed_key[seed])) seed = e;
        }
        if (seed == -1) break;

        std::queue<int> q; q.push(seed); vis[seed] = 1;
        while (!q.empty()) {
            int u = q.front(); q.pop();
            order.push_back(u);

            auto nb = adj[u];
            // prefer neighbors sharing more nodes with u, then lower degree, then id
            std::stable_sort(nb.begin(), nb.end(), [&](int a, int b){
                int oa = 0, ob = 0;
                const int* cu = &conn[u*conn_stride];
                const int* ca = &conn[a*conn_stride];
                const int* cb = &conn[b*conn_stride];
                for (int i = 0; i < conn_stride; ++i)
                    for (int j = 0; j < conn_stride; ++j) {
                        oa += (cu[i]==ca[j]); ob += (cu[i]==cb[j]);
                    }
                if (oa != ob) return oa > ob;
                if (deg[a] != deg[b]) return deg[a] < deg[b];
                return a < b;
            });

            for (int v : nb) if (!vis[v]) { vis[v] = 1; q.push(v); }
        }
    }
    return order; // vector of old_e in new order
}

void reorder_elements_dual_bfs(int* conn, int conn_stride, int n_elements, int num_nodes)
{
    std::vector<int> order = build_element_order_dual_bfs(conn, conn_stride, n_elements, num_nodes);

    std::vector<int> conn2(n_elements * conn_stride);
    for (int new_e = 0; new_e < n_elements; ++new_e) {
        int old_e = order[new_e];
        for (int i = 0; i < conn_stride; ++i)
            conn2[new_e*conn_stride + i] = conn[old_e*conn_stride + i];
    }
    
    // memcpy and reverse order
    for (int elem_i = 0; elem_i < n_elements; ++elem_i) {
        for (int i = 0; i < conn_stride; i++) {
            conn[elem_i * conn_stride + i] = conn2[ (n_elements - elem_i - 1) * conn_stride + i];
        }
    }
}


// Build symmetric adjacency from element connectivity (conn_stride nodes per element)
std::vector<std::vector<int>> symx::build_adjacency(int* conn, int conn_stride, int n_elements, int num_nodes)
{
    std::vector<std::vector<int>> adj(num_nodes);
    for (int e = 0; e < n_elements; ++e) {
        const int* c = &conn[e * conn_stride];
        for (int i = 0; i < conn_stride; ++i) {
            const int a = c[i];
            for (int j = i + 1; j < conn_stride; ++j) {
                const int b = c[j];
                if (a == b) continue;
                adj[a].push_back(b);
                adj[b].push_back(a);
            }
        }
    }
    // deduplicate neighbors and sort by degree-friendly order (ascending degree, then id)
    std::vector<int> deg(num_nodes);
    for (int i = 0; i < num_nodes; ++i) {
        auto& nbrs = adj[i];
        std::sort(nbrs.begin(), nbrs.end());
        nbrs.erase(std::unique(nbrs.begin(), nbrs.end()), nbrs.end());
        deg[i] = static_cast<int>(nbrs.size());
    }
    for (int i = 0; i < num_nodes; ++i) {
        auto& nbrs = adj[i];
        std::stable_sort(nbrs.begin(), nbrs.end(), [&](int a, int b){
            if (deg[a] != deg[b]) return deg[a] < deg[b];
            return a < b;
        });
    }
    return adj;
}

symx::ReorderResult symx::reorder_connectivity_RCM(int* conn, int conn_stride, int n_elements, int num_nodes, bool also_reorder_elements)
{
    std::vector<std::vector<int>> adj = build_adjacency(conn, conn_stride, n_elements, num_nodes);

    auto old_to_new = rcm_order(adj);
    auto new_to_old = invert_permutation(old_to_new);

    apply_node_permutation_to_conn(conn, conn_stride, n_elements, old_to_new);

    if (also_reorder_elements) {
        reorder_elements_dual_bfs(conn, conn_stride, n_elements, num_nodes);
    }

    return {old_to_new, new_to_old};
}
