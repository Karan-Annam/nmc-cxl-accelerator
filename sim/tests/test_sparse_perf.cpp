// The 8-wide sparse row walk: throughput metrics and the alignment cases the
// wide datapath must get right. The lane→bank rotation is constant per row
// but different for the A-stream, B-stream and destination whenever their
// bases differ mod 8 — so every correctness case here deliberately misaligns
// all three and uses a row length that is not a multiple of 8 (tail-chunk
// lane masking). Throughput floor checks gate the wide walk's existence, not
// exact cycle counts.
#include <cstdint>
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

TEST(test_sparse_perf) {
    std::mt19937 rng(21);
    std::uniform_int_distribution<int32_t> dist(-40, 40);

    // ---- 1. SPARSE dot, everything misaligned, dk not a multiple of 8 ----
    {
        const int rows = 12, dk = 13;
        const uint16_t k_base = 3;      // bank 3
        const uint16_t q_base = 1029;   // bank 5
        const uint16_t idx_b  = 1200;
        const uint16_t dst    = 1310;   // bank 6

        std::vector<int32_t> K(rows * dk), Q(dk);
        for (auto& v : K) v = dist(rng);
        for (auto& v : Q) v = dist(rng);
        for (int i = 0; i < rows * dk; i++)
            host.mem_write(uint16_t(k_base + i), uint32_t(K[i]));
        for (int d = 0; d < dk; d++)
            host.mem_write(uint16_t(q_base + d), uint32_t(Q[d]));

        const std::vector<uint32_t> idx = {11, 0, 7, 3, 11, 5};
        for (size_t m = 0; m < idx.size(); m++)
            host.mem_write(uint16_t(idx_b + m), idx[m]);

        host.download_config(ConfigBuilder().set_all(PE_MACC, SRC_SRAM_A).build());
        host.submit_sparse(OPC_SPARSE, k_base, q_base, dst, idx_b,
                           uint16_t(idx.size()), dk);
        CHECK(host.wait_for_done());
        for (size_t m = 0; m < idx.size(); m++) {
            int64_t ref = 0;
            for (int d = 0; d < dk; d++) ref += int64_t(K[idx[m] * dk + d]) * Q[d];
            CHECK_EQ(int32_t(host.mem_read(uint16_t(dst + m))), int32_t(ref));
        }
    }

    // ---- 2. WGATHER misaligned + short rows (stride=2 tree/row stress) ----
    {
        const int rows = 10, dk = 13;
        const uint16_t v_base = 2053;   // bank 5
        const uint16_t w_base = 2501;   // bank 5 (weights, indexed by m)
        const uint16_t idx_b  = 2601;
        const uint16_t dst    = 2703;   // bank 7

        std::vector<int32_t> V(rows * dk), W(4);
        for (auto& v : V) v = dist(rng);
        for (auto& v : W) v = dist(rng);
        for (int i = 0; i < rows * dk; i++)
            host.mem_write(uint16_t(v_base + i), uint32_t(V[i]));
        for (size_t m = 0; m < W.size(); m++)
            host.mem_write(uint16_t(w_base + m), uint32_t(W[m]));

        const std::vector<uint32_t> idx = {9, 2, 4, 9};
        for (size_t m = 0; m < idx.size(); m++)
            host.mem_write(uint16_t(idx_b + m), idx[m]);

        host.download_config(ConfigBuilder().set_all(PE_MACC, SRC_SRAM_A).build());
        host.submit_sparse(OPC_WGATHER, v_base, w_base, dst, idx_b,
                           uint16_t(idx.size()), dk);
        CHECK(host.wait_for_done());
        for (int d = 0; d < dk; d++) {
            int64_t ref = 0;
            for (size_t m = 0; m < idx.size(); m++)
                ref += int64_t(W[m]) * V[idx[m] * dk + d];
            CHECK_EQ(int32_t(host.mem_read(uint16_t(dst + d))), int32_t(ref));
        }

        // back-to-back 2-element rows: rows turn over every few cycles
        host.submit_sparse(OPC_WGATHER, v_base, w_base, dst, idx_b,
                           uint16_t(idx.size()), 2);
        CHECK(host.wait_for_done());
        for (int d = 0; d < 2; d++) {
            int64_t ref = 0;
            for (size_t m = 0; m < idx.size(); m++)
                ref += int64_t(W[m]) * V[idx[m] * 2 + d];
            CHECK_EQ(int32_t(host.mem_read(uint16_t(dst + d))), int32_t(ref));
        }
    }

    // ---- 3. EMBEDDING misaligned row copy ----
    {
        const int nrow = 9, dim = 11;
        const uint16_t tab   = 3001;    // bank 1
        const uint16_t idx_b = 3201;
        const uint16_t dst   = 3306;    // bank 2

        std::vector<int32_t> T(nrow * dim);
        for (auto& v : T) v = dist(rng);
        for (int i = 0; i < nrow * dim; i++)
            host.mem_write(uint16_t(tab + i), uint32_t(T[i]));
        const std::vector<uint32_t> ids = {8, 0, 5};
        for (size_t m = 0; m < ids.size(); m++)
            host.mem_write(uint16_t(idx_b + m), ids[m]);

        host.download_config(ConfigBuilder().set_all(PE_PASS_A, SRC_SRAM_A).build());
        host.submit_sparse(OPC_EMBEDDING, tab, 0, dst, idx_b,
                           uint16_t(ids.size()), dim);
        CHECK(host.wait_for_done());
        for (size_t m = 0; m < ids.size(); m++)
            for (int d = 0; d < dim; d++)
                CHECK_EQ(int32_t(host.mem_read(uint16_t(dst + m * dim + d))),
                         T[ids[m] * dim + d]);
    }

    // ---- 4. throughput: SPARSE and WGATHER, 128 rows x dk=64 ----
    {
        const int rows = 128, dk = 64;
        const uint16_t k_base = 0;              // 8192 words
        const uint16_t q_base = 8192;
        const uint16_t idx_b  = 8300;
        const uint16_t dst    = 8500;

        std::vector<uint32_t> big(rows * dk);
        for (auto& v : big) v = uint32_t(dist(rng));
        host.mem_write_burst(k_base, big);
        // 128 words: Q[d] for the SPARSE pass (first dk), weights W[m] for the
        // WGATHER pass (first rows)
        std::vector<uint32_t> q(rows);
        for (auto& v : q) v = uint32_t(dist(rng));
        host.mem_write_burst(q_base, q);
        std::vector<uint32_t> idx(rows);
        for (int m = 0; m < rows; m++) idx[m] = uint32_t((m * 37) % rows);
        host.mem_write_burst(idx_b, idx);

        host.download_config(ConfigBuilder().set_all(PE_MACC, SRC_SRAM_A).build());

        open_window(host);
        host.submit_sparse(OPC_SPARSE, k_base, q_base, dst, idx_b,
                           uint16_t(rows), dk);
        CHECK(host.wait_for_done());
        uint64_t cyc = host.get_perf_cycles();
        uint32_t ops = host.get_perf_ops();
        CHECK_EQ(ops, uint32_t(rows * dk));
        double epc = double(ops) / double(cyc);
        metric("sparse.elems_per_cycle", epc);
        metric("sparse.row64_cycles", double(cyc) / rows);
        CHECK(epc > 6.0);   // wide walk + prefetch + pipelined tree; 0.48 before

        open_window(host);
        host.submit_sparse(OPC_WGATHER, k_base, q_base, dst, idx_b,
                           uint16_t(rows), dk);
        CHECK(host.wait_for_done());
        cyc = host.get_perf_cycles();
        ops = host.get_perf_ops();
        CHECK_EQ(ops, uint32_t(rows * dk));
        epc = double(ops) / double(cyc);
        metric("wgather.elems_per_cycle", epc);
        CHECK(epc > 6.0);
    }
}
