// CSR sparse-matrix × dense-vector on-device. Per row i the
// host issues one WGATHER(stride=1): result[i] = Σ_k vals[i][k] · x[col[i][k]],
// with the column-index list driving the gather of x. Exact integer match vs a
// C++ CSR reference. Same MACC hardware config as attention — no RTL change.
#include <random>
#include <vector>
#include "../config_builder.hpp"
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_spmv) {
    std::mt19937 rng(99);
    std::uniform_int_distribution<int32_t> vdist(-20, 20);
    std::uniform_real_distribution<double> p(0.0, 1.0);

    const int N = 64;
    const double density = 0.1;

    // CSR build
    std::vector<std::vector<uint32_t>> cols(N);
    std::vector<std::vector<int32_t>>  vals(N);
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            if (p(rng) < density) {
                cols[i].push_back(uint32_t(j));
                vals[i].push_back(vdist(rng));
            }

    std::vector<int32_t> x(N);
    for (auto& v : x) v = vdist(rng);

    // HDM layout: x vector, then per-row vals and col lists packed sequentially
    const uint16_t X_BASE = 0;
    const uint16_t POOL   = 256;      // vals+cols pool
    const uint16_t RES    = 8192;

    for (int i = 0; i < N; i++) host.mem_write(uint16_t(X_BASE + i), uint32_t(x[i]));

    std::vector<uint16_t> vals_at(N), cols_at(N);
    uint16_t cur = POOL;
    for (int i = 0; i < N; i++) {
        vals_at[i] = cur;
        for (size_t k = 0; k < vals[i].size(); k++)
            host.mem_write(uint16_t(cur + k), uint32_t(vals[i][k]));
        cur = uint16_t(cur + vals[i].size());
        cols_at[i] = cur;
        for (size_t k = 0; k < cols[i].size(); k++)
            host.mem_write(uint16_t(cur + k), cols[i][k]);
        cur = uint16_t(cur + cols[i].size());
    }

    // per-row WGATHER: A = x (gathered via col list), B = row values (by m)
    host.download_config(ConfigBuilder().set_all(PE_MACC, SRC_SRAM_A).build());
    for (int i = 0; i < N; i++) {
        if (cols[i].empty()) {
            host.mem_write(uint16_t(RES + i), 0);   // empty row → 0
            continue;
        }
        host.submit_sparse(OPC_WGATHER, X_BASE, vals_at[i], uint16_t(RES + i),
                           cols_at[i], uint16_t(cols[i].size()), 1);
        CHECK(host.wait_for_done());
    }

    // exact reference
    for (int i = 0; i < N; i++) {
        int64_t ref = 0;
        for (size_t k = 0; k < cols[i].size(); k++)
            ref += int64_t(vals[i][k]) * x[cols[i][k]];
        CHECK_EQ(int32_t(host.mem_read(uint16_t(RES + i))), int32_t(ref));
    }
}
