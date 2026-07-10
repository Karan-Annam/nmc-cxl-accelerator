// CXL-traffic reduction for the remaining two workload classes, measured the
// same way as attention/SpMV: PERF_CXL_RD + PERF_CXL_WR (HDM words) inside a
// quiesced window, against an analytic host-baseline.
//
// GNN mean-aggregation: features AND the graph-shaped index lists are
// preloaded (excluded) — graph structure is loaded once and reused across
// layers/passes, the standard GNN reuse argument. Per pass the host only
// reads back N*C aggregated values; the CPU baseline must pull every
// neighbor's feature word: E*C words per pass.
//
// Embedding-bag: the table is resident (excluded); per batch the host ships
// B ids + B weights and reads back one pooled dim-vector, while the CPU
// baseline pulls all B selected rows (B*dim words).
//
// Both also emit ctrl-inclusive variants counting CXL.io slots (16B = 4 link
// words) spent on command/config/polling — the baselines carry no control
// term, so the headline excludes it; reporting both keeps the comparison
// honest.
#include <random>
#include <vector>
#include "../config_builder.hpp"
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

static void open_window(CxlHostModel& host) {
    host.pump(100);
    host.reset_perf();
    while (host.get_cxl_writes() != 0 || host.get_cxl_reads() != 0)
        host.pump(10);
}

TEST(test_gnn_perf) {
    std::mt19937 rng(77);

    // ---- GNN mean aggregation: N=64 nodes, deg=8, C=8 channels ----
    {
        const int N = 64, DEG = 8, C = 8;
        const uint16_t feat_base = 0;        // 512 words
        const uint16_t idx_base  = 1024;     // N*C lists of DEG addresses
        const uint16_t dst       = 5200;     // 512 results

        std::uniform_int_distribution<int32_t> dist(-500, 500);
        std::uniform_int_distribution<uint32_t> pick(0, uint32_t(N - 1));

        std::vector<int32_t> feat(N * C);
        for (auto& v : feat) v = dist(rng);
        std::vector<std::vector<uint32_t>> adj(static_cast<size_t>(N));
        for (int u = 0; u < N; u++)
            for (int d = 0; d < DEG; d++) adj[size_t(u)].push_back(pick(rng));

        // preload features + channel-specific index lists (graph structure)
        std::vector<uint32_t> fraw(feat.begin(), feat.end());
        host.mem_write_burst(feat_base, fraw);
        std::vector<uint32_t> ilists;
        for (int u = 0; u < N; u++)
            for (int c = 0; c < C; c++)
                for (int d = 0; d < DEG; d++)
                    ilists.push_back(adj[size_t(u)][size_t(d)] * uint32_t(C) +
                                     uint32_t(c));
        host.mem_write_burst(idx_base, ilists);
        host.download_config(ConfigBuilder().set_all(PE_SACC, SRC_SRAM_A).build());

        open_window(host);
        uint32_t io_req0 = host.stat_io_req_slots, io_rx0 = host.stat_io_slots_rx;
        for (int u = 0; u < N; u++)
            for (int c = 0; c < C; c++) {
                host.submit_sparse(OPC_REDUCTION, feat_base, 0,
                                   uint16_t(dst + u * C + c),
                                   uint16_t(idx_base + (u * C + c) * DEG),
                                   uint16_t(DEG), 1);
                CHECK(host.wait_for_done());
            }
        std::vector<uint32_t> out;
        host.mem_read_burst(dst, out, size_t(N * C));
        uint32_t nmc_words = host.get_cxl_reads() + host.get_cxl_writes();
        uint32_t ctrl_slots = (host.stat_io_req_slots - io_req0) +
                              (host.stat_io_slots_rx - io_rx0);

        // spot-check sums against the golden graph
        for (int u = 0; u < N; u += 13)
            for (int c = 0; c < C; c += 3) {
                int64_t sum = 0;
                for (uint32_t v : adj[size_t(u)]) sum += feat[v * uint32_t(C) + uint32_t(c)];
                CHECK_EQ(int32_t(out[size_t(u * C + c)]), int32_t(sum));
            }

        const double baseline = double(N * DEG * C);   // every neighbor word
        CHECK_EQ(nmc_words, uint32_t(N * C));          // readback only
        metric("gnn.nodes", N);
        metric("gnn.edges", N * DEG);
        metric("gnn.channels", C);
        metric("gnn.nmc_words", nmc_words);
        metric("gnn.baseline_words", baseline);
        metric("gnn.reduction", baseline / double(nmc_words));
        metric("gnn.nmc_words_ctrl_incl", double(nmc_words) + 4.0 * ctrl_slots);
        metric("gnn.reduction_ctrl_incl",
               baseline / (double(nmc_words) + 4.0 * ctrl_slots));
        std::printf("    gnn: NMC=%u words  baseline=%.0f  reduction=%.1fx"
                    "  (ctrl-incl %.1fx)\n",
                    nmc_words, baseline, baseline / double(nmc_words),
                    baseline / (double(nmc_words) + 4.0 * ctrl_slots));
    }

    // ---- embedding bag: 256x64 table, batch of 64 weighted lookups ----
    {
        const int ROWS = 256, DIM = 64, B = 64;
        const uint16_t tab   = 8192;    // 16384 words, resident
        const uint16_t ids   = 25000;
        const uint16_t wgt   = 25100;
        const uint16_t dst   = 25300;

        std::uniform_int_distribution<int32_t> dist(-100, 100);
        std::uniform_int_distribution<uint32_t> pick(0, uint32_t(ROWS - 1));

        std::vector<int32_t> T(ROWS * DIM);
        for (auto& v : T) v = dist(rng);
        std::vector<uint32_t> traw(T.begin(), T.end());
        host.mem_write_burst(tab, traw);
        host.download_config(ConfigBuilder().set_all(PE_MACC, SRC_SRAM_A).build());

        std::vector<uint32_t> batch(B), w(B);
        for (auto& v : batch) v = pick(rng);
        std::vector<int32_t> wsig(B);
        for (int i = 0; i < B; i++) { wsig[i] = dist(rng); w[size_t(i)] = uint32_t(wsig[i]); }

        open_window(host);
        uint32_t io_req0 = host.stat_io_req_slots, io_rx0 = host.stat_io_slots_rx;
        host.mem_write_burst(ids, batch);
        host.mem_write_burst(wgt, w);
        host.submit_sparse(OPC_WGATHER, tab, wgt, dst, ids, uint16_t(B),
                           uint16_t(DIM));
        CHECK(host.wait_for_done());
        std::vector<uint32_t> out;
        host.mem_read_burst(dst, out, size_t(DIM));
        uint32_t nmc_words = host.get_cxl_reads() + host.get_cxl_writes();
        uint32_t ctrl_slots = (host.stat_io_req_slots - io_req0) +
                              (host.stat_io_slots_rx - io_rx0);

        for (int d = 0; d < DIM; d += 7) {
            int64_t ref = 0;
            for (int i = 0; i < B; i++)
                ref += int64_t(wsig[i]) * T[batch[size_t(i)] * uint32_t(DIM) + uint32_t(d)];
            CHECK_EQ(int32_t(out[size_t(d)]), int32_t(ref));
        }

        const double baseline = double(B * DIM);       // every selected row word
        CHECK_EQ(nmc_words, uint32_t(2 * B + DIM));    // ids + weights in, row out
        metric("embed.batch", B);
        metric("embed.dim", DIM);
        metric("embed.nmc_words", nmc_words);
        metric("embed.baseline_words", baseline);
        metric("embed.reduction", baseline / double(nmc_words));
        metric("embed.nmc_words_ctrl_incl", double(nmc_words) + 4.0 * ctrl_slots);
        metric("embed.reduction_ctrl_incl",
               baseline / (double(nmc_words) + 4.0 * ctrl_slots));
        std::printf("    embed: NMC=%u words  baseline=%.0f  reduction=%.1fx"
                    "  (ctrl-incl %.1fx)\n",
                    nmc_words, baseline, baseline / double(nmc_words),
                    baseline / (double(nmc_words) + 4.0 * ctrl_slots));
    }
}
