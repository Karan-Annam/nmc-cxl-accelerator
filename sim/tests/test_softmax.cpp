// End-to-end fixed-point softmax vs a double-precision
// golden reference; < 1% absolute error per element, weights sum ≈ 1.
#include <cmath>
#include <random>
#include <vector>
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

namespace {
int32_t to_q16(double x) { return int32_t(std::lround(x * 65536.0)); }
double  from_q16(int32_t v) { return double(v) / 65536.0; }
}

TEST(test_softmax) {
    std::mt19937 rng(101);
    std::uniform_real_distribution<double> dist(-4.0, 4.0);

    for (int len : {8, 32, 100}) {
        const uint16_t src = 0, dst = 512;
        std::vector<double> x(len);
        for (auto& v : x) v = dist(rng);
        for (int i = 0; i < len; i++)
            host.mem_write(uint16_t(src + i), uint32_t(to_q16(x[i])));

        host.submit_softmax(src, dst, uint16_t(len));
        CHECK(host.wait_for_done());

        // golden softmax
        double mx = x[0];
        for (double v : x) mx = v > mx ? v : mx;
        (void)mx;   // scores already within LUT range; no max-shift needed
        double s = 0;
        std::vector<double> ref(len);
        for (int i = 0; i < len; i++) { ref[i] = std::exp(x[i]); s += ref[i]; }
        for (int i = 0; i < len; i++) ref[i] /= s;

        double wsum = 0, maxerr = 0;
        for (int i = 0; i < len; i++) {
            double w = from_q16(int32_t(host.mem_read(uint16_t(dst + i))));
            wsum += w;
            double e = std::fabs(w - ref[i]);
            maxerr = e > maxerr ? e : maxerr;
            CHECK_NEAR(w, ref[i], 0.01);   // < 1% absolute
        }
        std::printf("    len=%d  max|err|=%.5f  sum=%.5f\n", len, maxerr, wsum);
        CHECK_NEAR(wsum, 1.0, 0.02);
        if (len == 32) metric("softmax.max_abs_err_len32", maxerr);
    }
}
