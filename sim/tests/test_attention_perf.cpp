// The headline measurement: CXL word traffic per sparse-attention query vs a
// CPU baseline that must pull every selected K/V row across the link.
// seq_len=256, d_k=64, sparsity 50% and 90%. The counted metric is
// host-initiated HDM word transactions (PERF_CXL_RD/WR); MMIO control traffic
// is deliberately excluded (control plane, not the bottleneck being studied).
#include <cmath>
#include <random>
#include <vector>
#include "../config_builder.hpp"
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_attention_perf) {
    std::mt19937 rng(55);
    std::uniform_real_distribution<double> dist(-0.5, 0.5);

    const int seq = 256, dk = 64;
    const uint16_t K_BASE = 0;          // 256×64 = 16384
    const uint16_t V_BASE = 16384;      // 16384
    const uint16_t Q_BASE = 33000;
    const uint16_t IDXB   = 33100;
    const uint16_t SCORES = 34000;
    const uint16_t WEIGHTS = 34400;
    const uint16_t OUTB   = 34800;

    // one-time preload of the KV cache (stays on device — not per-query traffic)
    std::vector<uint32_t> kv(seq * dk);
    for (auto& v : kv) v = uint32_t(int32_t(std::lround(dist(rng) * 256.0)));
    host.mem_write_burst(K_BASE, kv);
    for (auto& v : kv) v = uint32_t(int32_t(std::lround(dist(rng) * 256.0)));
    host.mem_write_burst(V_BASE, kv);

    for (double sparsity : {0.5, 0.9}) {
        const int nnz = int(seq * (1.0 - sparsity));
        // per-query working set: Q vector + index list in, output vector back.
        // Quiesce the link before opening the measurement window: CXL.io and
        // CXL.mem dispatch from independent queues, so the posted PERF_RESET
        // must not race in-flight HDM traffic on either side of the boundary.
        host.pump(100);
        host.reset_perf();
        while (host.get_cxl_writes() != 0 || host.get_cxl_reads() != 0)
            host.pump(10);   // read-after-reset: window provably open at zero

        std::vector<uint32_t> q(dk);
        for (auto& v : q) v = uint32_t(int32_t(std::lround(dist(rng) * 256.0)));
        host.mem_write_burst(Q_BASE, q);

        std::vector<uint32_t> idx(nnz);
        for (int m = 0; m < nnz; m++) idx[m] = uint32_t((m * seq) / nnz);
        host.mem_write_burst(IDXB, idx);

        host.download_config(ConfigBuilder().set_all(PE_MACC, SRC_SRAM_A).build());
        host.submit_sparse(OPC_SPARSE, K_BASE, Q_BASE, SCORES, IDXB,
                           uint16_t(nnz), dk);
        CHECK(host.wait_for_done());
        host.submit_softmax(SCORES, WEIGHTS, uint16_t(nnz));
        CHECK(host.wait_for_done());
        host.submit_sparse(OPC_WGATHER, V_BASE, WEIGHTS, OUTB, IDXB,
                           uint16_t(nnz), dk);
        CHECK(host.wait_for_done());

        std::vector<uint32_t> outv;
        host.mem_read_burst(OUTB, outv, dk);

        uint32_t nmc_rd = host.get_cxl_reads();
        uint32_t nmc_wr = host.get_cxl_writes();
        uint32_t nmc_words = nmc_rd + nmc_wr;

        // CPU baseline: every selected K row and V row crosses the link per query
        uint32_t baseline_words = uint32_t(nnz) * dk * 2;

        double reduction = double(baseline_words) / double(nmc_words);
        std::printf(
            "    sparsity=%.0f%%  nnz=%d  NMC=%u words (rd %u, wr %u)  "
            "baseline=%u words  reduction=%.1fx\n",
            sparsity * 100, nnz, nmc_words, nmc_rd, nmc_wr, baseline_words,
            reduction);

        int tag = int(sparsity * 100);
        metric("attention.s" + std::to_string(tag) + ".nmc_words", nmc_words);
        metric("attention.s" + std::to_string(tag) + ".baseline_words",
               baseline_words);
        metric("attention.s" + std::to_string(tag) + ".reduction", reduction);
        metric("attention.s" + std::to_string(tag) + ".nnz", nnz);

        CHECK(nmc_words < baseline_words);       // NMC must win
        // expected per-query traffic: Q(dk wr) + idx(nnz wr) + out(dk rd)
        CHECK_EQ(nmc_wr, uint32_t(dk + nnz));
        CHECK_EQ(nmc_rd, uint32_t(dk));
    }
    metric("attention.seq_len", seq);
    metric("attention.d_k", dk);
}
