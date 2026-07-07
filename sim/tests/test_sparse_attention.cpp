// Full sparse attention head on-device
// (Q·K scores → softmax → weighted V gather) vs a double-precision golden
// reference. Fixed-point plan: Q,K,V stored as Q8.8 so the PE's raw 32-bit
// multiply naturally yields Q16.16 scores; weights come back Q16.16; the
// WGATHER output is scale 2^24 (Q16.16 × Q8.8), rescaled on the host.
#include <cmath>
#include <random>
#include <vector>
#include "../config_builder.hpp"
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

namespace {
int32_t to_q8(double x) { return int32_t(std::lround(x * 256.0)); }
}

TEST(test_sparse_attention) {
    std::mt19937 rng(77);
    std::uniform_real_distribution<double> dist(-0.8, 0.8);

    const int seq = 32, dk = 8, nnz = 8;
    const uint16_t K_BASE = 0;        // 32×8
    const uint16_t V_BASE = 512;      // 32×8
    const uint16_t Q_BASE = 1024;
    const uint16_t IDXB   = 1100;
    const uint16_t SCORES = 1200;
    const uint16_t WEIGHTS = 1300;
    const uint16_t OUTB   = 1400;

    std::vector<double> K(seq * dk), V(seq * dk), Q(dk);
    for (auto& v : K) v = dist(rng);
    for (auto& v : V) v = dist(rng);
    for (auto& v : Q) v = dist(rng);

    for (int i = 0; i < seq * dk; i++) {
        host.mem_write(uint16_t(K_BASE + i), uint32_t(to_q8(K[i])));
        host.mem_write(uint16_t(V_BASE + i), uint32_t(to_q8(V[i])));
    }
    for (int d = 0; d < dk; d++)
        host.mem_write(uint16_t(Q_BASE + d), uint32_t(to_q8(Q[d])));

    const std::vector<uint32_t> idx = {2, 7, 11, 14, 19, 23, 28, 31};
    for (size_t m = 0; m < idx.size(); m++)
        host.mem_write(uint16_t(IDXB + m), idx[m]);

    // 1. sparse Q·K scores (Q8.8 × Q8.8 accumulates to Q16.16)
    host.download_config(ConfigBuilder().set_all(PE_MACC, SRC_SRAM_A).build());
    host.submit_sparse(OPC_SPARSE, K_BASE, Q_BASE, SCORES, IDXB, nnz, dk);
    CHECK(host.wait_for_done());

    // 2. softmax over the scores → weights (Q16.16)
    host.submit_softmax(SCORES, WEIGHTS, nnz);
    CHECK(host.wait_for_done());

    // 3. weighted V gather: out[d] = Σ_m w[m]·V[idx[m]][d]  (scale 2^24)
    host.submit_sparse(OPC_WGATHER, V_BASE, WEIGHTS, OUTB, IDXB, nnz, dk);
    CHECK(host.wait_for_done());

    // golden reference (double)
    std::vector<double> scores(nnz), w(nnz), out(dk, 0.0);
    double s = 0;
    for (int m = 0; m < nnz; m++) {
        double dot = 0;
        for (int d = 0; d < dk; d++) dot += Q[d] * K[idx[m] * dk + d];
        scores[m] = dot;
        w[m] = std::exp(dot);
        s += w[m];
    }
    for (int m = 0; m < nnz; m++) w[m] /= s;
    for (int m = 0; m < nnz; m++)
        for (int d = 0; d < dk; d++) out[d] += w[m] * V[idx[m] * dk + d];

    // device scores: Q16.16
    for (int m = 0; m < nnz; m++) {
        double got = double(int32_t(host.mem_read(uint16_t(SCORES + m)))) / 65536.0;
        CHECK_NEAR(got, scores[m], 0.05);   // Q8.8 quantization of inputs
    }
    // device output: scale 2^24
    double maxerr = 0;
    for (int d = 0; d < dk; d++) {
        double got =
            double(int32_t(host.mem_read(uint16_t(OUTB + d)))) / 16777216.0;
        double e = std::fabs(got - out[d]);
        maxerr = e > maxerr ? e : maxerr;
        CHECK_NEAR(got, out[d], 0.01);      // < 1% of unit scale
    }
    std::printf("    attention max|err| = %.5f\n", maxerr);
    metric("attention.max_abs_err", maxerr);

    // mode-switch sanity: dense command right after the sparse pipeline
    host.download_config(ConfigBuilder().set_all(PE_ADD, SRC_SRAM_A).build());
    host.submit_dense(OPC_DENSE, K_BASE, V_BASE, 2000, 8);
    CHECK(host.wait_for_done());
    for (int i = 0; i < 8; i++)
        CHECK_EQ(int32_t(host.mem_read(uint16_t(2000 + i))),
                 to_q8(K[i]) + to_q8(V[i]));
}
