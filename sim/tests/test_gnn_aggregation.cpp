// The fourth workload class: GNN neighbor aggregation.
// Per node u and feature channel c: mean = (Σ_{v∈N(u)} feat[v][c]) / deg(u) and
// max-pool = max_{v∈N(u)} feat[v][c], computed on-device via REDUCTION commands
// gathering through channel-specific neighbor index lists. Same hardware as every
// other sparse workload — only the host-built index lists are graph-shaped.
#include <random>
#include <vector>
#include "../config_builder.hpp"
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_gnn_aggregation) {
    std::mt19937 rng(31);
    std::uniform_int_distribution<int32_t> dist(-500, 500);

    // small graph: 6 nodes, directed adjacency (aggregating over in-neighbors)
    const std::vector<std::vector<uint32_t>> adj = {
        {1, 2, 5},      // node 0
        {0, 3},         // node 1
        {0, 1, 3, 4},   // node 2
        {2},            // node 3
        {0, 1, 2, 5},   // node 4
        {4, 3},         // node 5
    };
    const int n_nodes = int(adj.size()), n_ch = 4;

    const uint16_t feat_base = 0;           // feat[node][ch] row-major, 6×4
    const uint16_t idx_base = 256;
    const uint16_t dst = 512;

    std::vector<int32_t> feat(n_nodes * n_ch);
    for (auto& v : feat) v = dist(rng);
    for (int i = 0; i < n_nodes * n_ch; i++)
        host.mem_write(uint16_t(feat_base + i), uint32_t(feat[i]));

    // ---- mean aggregation (SACC sum on device, divide by degree on host) ----
    host.download_config(ConfigBuilder().set_all(PE_SACC, SRC_SRAM_A).build());
    for (int u = 0; u < n_nodes; u++) {
        for (int c = 0; c < n_ch; c++) {
            // channel-specific neighbor index list: address of feat[v][c]
            for (size_t m = 0; m < adj[u].size(); m++)
                host.mem_write(uint16_t(idx_base + m),
                               adj[u][m] * uint32_t(n_ch) + uint32_t(c));
            host.submit_sparse(OPC_REDUCTION, feat_base, 0,
                               uint16_t(dst + u * n_ch + c), idx_base,
                               uint16_t(adj[u].size()), 1);
            CHECK(host.wait_for_done());
        }
    }
    for (int u = 0; u < n_nodes; u++) {
        for (int c = 0; c < n_ch; c++) {
            int64_t sum = 0;
            for (uint32_t v : adj[u]) sum += feat[v * n_ch + c];
            int32_t got_sum = int32_t(host.mem_read(uint16_t(dst + u * n_ch + c)));
            CHECK_EQ(got_sum, int32_t(sum));
            // host-side mean (documented division-by-degree step)
            int32_t mean_ref = int32_t(sum / int(adj[u].size()));
            CHECK_EQ(got_sum / int32_t(adj[u].size()), mean_ref);
        }
    }

    // ---- max aggregation ----
    host.download_config(ConfigBuilder().set_all(PE_MAX, SRC_ACC).build());
    for (int u = 0; u < n_nodes; u++) {
        for (int c = 0; c < n_ch; c++) {
            for (size_t m = 0; m < adj[u].size(); m++)
                host.mem_write(uint16_t(idx_base + m),
                               adj[u][m] * uint32_t(n_ch) + uint32_t(c));
            host.submit_sparse(OPC_REDUCTION, feat_base, 0,
                               uint16_t(dst + 64 + u * n_ch + c), idx_base,
                               uint16_t(adj[u].size()), 1);
            CHECK(host.wait_for_done());
        }
    }
    for (int u = 0; u < n_nodes; u++) {
        for (int c = 0; c < n_ch; c++) {
            int32_t mx = feat[adj[u][0] * n_ch + c];
            for (uint32_t v : adj[u])
                mx = feat[v * n_ch + c] > mx ? feat[v * n_ch + c] : mx;
            CHECK_EQ(int32_t(host.mem_read(uint16_t(dst + 64 + u * n_ch + c))), mx);
        }
    }
}
