// CXL word traffic for SpMV with the
// matrix + vector resident on-device, vs a CPU baseline that streams every value,
// column index and gathered vector element across the link. 256×256 matrix at
// 1%, 5%, 10% density.
#include <random>
#include <vector>
#include "../config_builder.hpp"
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_spmv_perf) {
    std::mt19937 rng(123);
    std::uniform_int_distribution<int32_t> vdist(-9, 9);
    std::uniform_real_distribution<double> p(0.0, 1.0);
    const int N = 256;

    for (double density : {0.01, 0.05, 0.10}) {
        std::vector<std::vector<uint32_t>> cols(N);
        std::vector<std::vector<int32_t>>  vals(N);
        uint32_t nnz = 0;
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                if (p(rng) < density) {
                    cols[i].push_back(uint32_t(j));
                    vals[i].push_back(vdist(rng));
                    nnz++;
                }

        const uint16_t X_BASE = 0, POOL = 512, RES = 40000;

        // preload matrix + vector (one-time, excluded from per-multiply traffic)
        std::vector<uint32_t> xv(N);
        for (auto& v : xv) v = uint32_t(vdist(rng));
        host.mem_write_burst(X_BASE, xv);
        uint16_t cur = POOL;
        std::vector<uint16_t> vals_at(N), cols_at(N);
        for (int i = 0; i < N; i++) {
            std::vector<uint32_t> vraw(vals[i].begin(), vals[i].end());
            vals_at[i] = cur;
            if (!vraw.empty()) host.mem_write_burst(cur, vraw);
            cur = uint16_t(cur + vraw.size());
            cols_at[i] = cur;
            if (!cols[i].empty()) host.mem_write_burst(cur, cols[i]);
            cur = uint16_t(cur + cols[i].size());
        }

        // measured region: submit N row commands, read back N results
        host.reset_perf();
        host.download_config(ConfigBuilder().set_all(PE_MACC, SRC_SRAM_A).build());
        for (int i = 0; i < N; i++) {
            if (cols[i].empty()) continue;
            host.submit_sparse(OPC_WGATHER, X_BASE, vals_at[i], uint16_t(RES + i),
                               cols_at[i], uint16_t(cols[i].size()), 1);
            CHECK(host.wait_for_done());
        }
        std::vector<uint32_t> res;
        host.mem_read_burst(RES, res, N);

        uint32_t nmc_words = host.get_cxl_reads() + host.get_cxl_writes();
        // baseline: vals + col indices + gathered x elements all cross the link
        uint32_t baseline_words = 3 * nnz;
        double reduction = double(baseline_words) / double(nmc_words);

        int tag = int(density * 100 + 0.5);
        std::printf(
            "    density=%d%%  nnz=%u  NMC=%u words  baseline=%u words  "
            "reduction=%.1fx\n",
            tag, nnz, nmc_words, baseline_words, reduction);
        metric("spmv.d" + std::to_string(tag) + ".nnz", nnz);
        metric("spmv.d" + std::to_string(tag) + ".nmc_words", nmc_words);
        metric("spmv.d" + std::to_string(tag) + ".baseline_words", baseline_words);
        metric("spmv.d" + std::to_string(tag) + ".reduction", reduction);
        CHECK(nmc_words < baseline_words);
    }
    metric("spmv.n", N);
}
