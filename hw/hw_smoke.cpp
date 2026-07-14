// On-hardware bring-up ladder for the Arty A7 board port. Runs the SAME
// CxlHostModel that drives simulation, over the COM port:
//   1. DEVICE_ID MMIO read           (link + MMIO path alive)
//   2. HDM write/read + burst        (memory path + burst slots)
//   3. dense VADD                    (engine + PEs + writeback)
//   4. mini sparse attention row-dot (SG engine + MACC + tree)
//   5. perf counter readout          (the research metric, live from silicon)
//
// Usage: hw_smoke.exe COM4 [baud]        (default 2000000 = 100 MHz / 50)
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "serial_host.hpp"
#include "../sim/config_builder.hpp"

static int fails = 0;
#define CHECKH(c, msg) do { \
    if (c) std::printf("  [ ok ] %s\n", msg); \
    else { std::printf("  [FAIL] %s\n", msg); fails++; } } while (0)

int main(int argc, char** argv) {
    if (argc < 2) { std::printf("usage: hw_smoke COMx [baud]\n"); return 2; }
    uint32_t baud = (argc >= 3) ? uint32_t(std::atoi(argv[2])) : 2000000u;

    try {
        SerialDut dut(argv[1], baud);
        CxlHostModel host(dut.pins());

        std::printf("== 1. device id ==\n");
        CHECKH(host.mmio_read(REG_DEVICE_ID) == 0xCA550001u, "DEVICE_ID == 0xCA550001");

        std::printf("== 2. HDM ==\n");
        host.mem_write(100, 0xDEADBEEFu);
        CHECKH(host.mem_read(100) == 0xDEADBEEFu, "single write/read");
        std::vector<uint32_t> blk(30), rd;
        for (size_t i = 0; i < blk.size(); i++) blk[i] = 0x1000u + uint32_t(i);
        host.mem_write_burst(200, blk);
        host.mem_read_burst(200, rd, blk.size());
        bool burst_ok = rd.size() == blk.size();
        for (size_t i = 0; burst_ok && i < blk.size(); i++) burst_ok = (rd[i] == blk[i]);
        CHECKH(burst_ok, "30-word burst write/readback");

        std::printf("== 3. dense VADD ==\n");
        for (uint16_t i = 0; i < 16; i++) {
            host.mem_write(uint16_t(300 + i), i);
            host.mem_write(uint16_t(320 + i), 100u + i);
        }
        host.download_config(ConfigBuilder().set_all(PE_ADD, SRC_SRAM_A).build());
        host.submit_dense(OPC_DENSE, 300, 320, 340, 16);
        bool done = host.wait_for_done();
        bool vadd_ok = done;
        for (uint16_t i = 0; vadd_ok && i < 16; i++)
            vadd_ok = (host.mem_read(uint16_t(340 + i)) == 100u + 2u * i);
        CHECKH(vadd_ok, "dense VADD 16 elements");

        std::printf("== 4. sparse row dot ==\n");
        // K rows at 400 (4 rows x 8), Q at 480, idx {2,0,3} at 500, out at 520
        for (uint16_t r = 0; r < 4; r++)
            for (uint16_t d = 0; d < 8; d++)
                host.mem_write(uint16_t(400 + r * 8 + d), r + 1);
        for (uint16_t d = 0; d < 8; d++) host.mem_write(uint16_t(480 + d), 2);
        uint32_t idx[3] = {2, 0, 3};
        for (uint16_t m = 0; m < 3; m++) host.mem_write(uint16_t(500 + m), idx[m]);
        host.download_config(ConfigBuilder().set_all(PE_MACC, SRC_SRAM_A).build());
        host.submit_sparse(OPC_SPARSE, 400, 480, 520, 500, 3, 8);
        done = host.wait_for_done();
        bool dot_ok = done;
        for (uint16_t m = 0; dot_ok && m < 3; m++)
            dot_ok = (host.mem_read(uint16_t(520 + m)) == (idx[m] + 1) * 2 * 8);
        CHECKH(dot_ok, "sparse Q.K row dots (idx {2,0,3})");

        std::printf("== 5. perf counters ==\n");
        std::printf("  cycles=%llu ops=%u cxl_rd=%u cxl_wr=%u cmds=%u\n",
                    (unsigned long long)host.get_perf_cycles(), host.get_perf_ops(),
                    host.get_cxl_reads(), host.get_cxl_writes(), host.get_perf_cmds());
        std::printf("  link: crc_errs=%u naks=%u retries=%u tx_flits=%u\n",
                    host.get_lnk(REG_LNK_CRC_ERRS), host.get_lnk(REG_LNK_NAKS),
                    host.get_lnk(REG_LNK_RETRIES), host.get_lnk(REG_LNK_TX_FLITS));
        CHECKH(host.get_lnk(REG_LNK_CRC_ERRS) == 0, "no CRC errors on the wire");

        std::printf(fails ? "\n%d FAILURES\n" : "\nALL OK — board is alive.\n", fails);
        return fails ? 1 : 0;
    } catch (const std::exception& e) {
        std::printf("FATAL: %s\n", e.what());
        return 2;
    }
}
