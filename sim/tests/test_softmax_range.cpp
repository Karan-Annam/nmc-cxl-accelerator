// Softmax numeric robustness: the pass-0 max-subtraction makes results exact
// for logits far outside the exp LUT's [-8,8) coverage — before it, anything
// past +8 saturated at exp(8) and distorted every weight. Softmax is
// shift-invariant, so the double-precision reference needs no special casing.
#include <cmath>
#include <random>
#include <vector>
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

namespace {
int32_t to_q16(double x) { return int32_t(std::lround(x * 65536.0)); }
double  from_q16(int32_t v) { return double(v) / 65536.0; }

void run_case(CxlHostModel& host, const std::vector<double>& x, double& maxerr) {
    const uint16_t src = 0, dst = 512;
    const int len = int(x.size());
    for (int i = 0; i < len; i++)
        host.mem_write(uint16_t(src + i), uint32_t(to_q16(x[i])));
    host.submit_softmax(src, dst, uint16_t(len));
    CHECK(host.wait_for_done());

    double mx = x[0];
    for (double v : x) mx = v > mx ? v : mx;
    double s = 0;
    std::vector<double> ref(static_cast<size_t>(len));
    for (int i = 0; i < len; i++) { ref[i] = std::exp(x[i] - mx); s += ref[i]; }
    for (int i = 0; i < len; i++) ref[i] /= s;

    double wsum = 0;
    for (int i = 0; i < len; i++) {
        double w = from_q16(int32_t(host.mem_read(uint16_t(dst + i))));
        wsum += w;
        double e = std::fabs(w - ref[i]);
        maxerr = e > maxerr ? e : maxerr;
        CHECK_NEAR(w, ref[i], 0.01);
    }
    CHECK_NEAR(wsum, 1.0, 0.02);
}
}  // namespace

TEST(test_softmax_range) {
    std::mt19937 rng(202);
    double maxerr = 0;

    // logits spanning [-20, 20] — far outside the LUT's [-8, 8) coverage
    {
        std::uniform_real_distribution<double> dist(-20.0, 20.0);
        std::vector<double> x(32);
        for (auto& v : x) v = dist(rng);
        run_case(host, x, maxerr);
    }

    // all large-and-equal: must come out uniform, not saturated garbage
    {
        std::vector<double> x(16, 15.0);
        run_case(host, x, maxerr);
    }

    // one dominant logit: weight ≈ 1 for it, ≈ 0 elsewhere
    {
        std::vector<double> x(16, -12.0);
        x[5] = 18.0;
        run_case(host, x, maxerr);
        double w5 = from_q16(int32_t(host.mem_read(uint16_t(512 + 5))));
        CHECK_NEAR(w5, 1.0, 0.01);
    }

    std::printf("    out-of-range logits: max|err|=%.5f\n", maxerr);
    metric("softmax.range.max_abs_err", maxerr);
}
