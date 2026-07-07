// Dense VADD/VMUL end-to-end at multiple lengths,
// unaligned bases, and the 8-lane parallel throughput check.
#include <cstdint>
#include <random>
#include <vector>
#include "../config_builder.hpp"
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_dense_ops) {
    std::mt19937 rng(42);
    std::uniform_int_distribution<int32_t> dist(-1000, 1000);

    for (uint16_t len : {8, 64, 256, 1024}) {
        uint16_t src_a = 0, src_b = 2048, dst = 4096;
        std::vector<int32_t> a(len), b(len);
        std::vector<uint32_t> araw(len), braw(len);
        for (int i = 0; i < len; i++) {
            a[i] = dist(rng); b[i] = dist(rng);
            araw[i] = uint32_t(a[i]); braw[i] = uint32_t(b[i]);
        }
        host.mem_write_burst(src_a, araw);
        host.mem_write_burst(src_b, braw);

        host.download_config(ConfigBuilder().set_all(PE_ADD, SRC_SRAM_A).build());
        host.submit_dense(OPC_DENSE, src_a, src_b, dst, len);
        CHECK(host.wait_for_done());
        for (int i = 0; i < len; i += (len > 64 ? 17 : 1))   // spot-check big arrays
            CHECK_EQ(int32_t(host.mem_read(uint16_t(dst + i))), a[i] + b[i]);

        host.download_config(ConfigBuilder().set_all(PE_MUL, SRC_SRAM_A).build());
        host.submit_dense(OPC_DENSE, src_a, src_b, dst, len);
        CHECK(host.wait_for_done());
        for (int i = 0; i < len; i += (len > 64 ? 13 : 1))
            CHECK_EQ(int32_t(host.mem_read(uint16_t(dst + i))),
                     int32_t(int64_t(a[i]) * b[i]));
    }

    // unaligned bases (src_a % 8 != 0, src_b % 8 != src_a % 8, len not multiple of 8)
    {
        uint16_t src_a = 3, src_b = 2053, dst = 4099;
        uint16_t len = 17;
        std::vector<int32_t> a(len), b(len);
        for (int i = 0; i < len; i++) {
            a[i] = dist(rng); b[i] = dist(rng);
            host.mem_write(uint16_t(src_a + i), uint32_t(a[i]));
            host.mem_write(uint16_t(src_b + i), uint32_t(b[i]));
        }
        host.download_config(ConfigBuilder().set_all(PE_ADD, SRC_SRAM_A).build());
        host.submit_dense(OPC_DENSE, src_a, src_b, dst, len);
        CHECK(host.wait_for_done());
        for (int i = 0; i < len; i++)
            CHECK_EQ(int32_t(host.mem_read(uint16_t(dst + i))), a[i] + b[i]);
    }

    // throughput: 8 PEs in parallel → ~len/8 active cycles (plus small overhead)
    {
        uint16_t len = 1024;
        host.reset_perf();
        host.download_config(ConfigBuilder().set_all(PE_ADD, SRC_SRAM_A).build());
        host.submit_dense(OPC_DENSE, 0, 2048, 4096, len);
        CHECK(host.wait_for_done());
        uint64_t cyc = host.get_perf_cycles();
        double epc = double(len) / double(cyc);
        std::printf("    dense len=%u: %llu engine cycles (%.2f elems/cycle)\n",
                    len, (unsigned long long)cyc, epc);
        metric("dense.len1024_cycles", double(cyc));
        metric("dense.elems_per_cycle", epc);
        CHECK(cyc <= len / 8 + 64);          // 8-lane parallelism really happened
        CHECK_EQ(host.get_perf_ops(), uint32_t(len));
    }
}
