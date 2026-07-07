// Verilator harness: clock/reset, pin bridge, test dispatch, results.json.
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>

#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vnmc_top.h"

#include "cxl_host_model.hpp"
#include "test_runner.hpp"

static Vnmc_top*      dut = nullptr;
static VerilatedVcdC* vcd = nullptr;
static uint64_t       sim_time = 0;
static bool           wave_on = false;

double sc_time_stamp() { return double(sim_time); }

static void tick_once() {
    dut->clk = 0;
    dut->eval();
    if (wave_on) vcd->dump(sim_time++);
    dut->clk = 1;
    dut->eval();
    if (wave_on) vcd->dump(sim_time++);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    std::string only;
    std::string json_path = "build/results.json";
    bool list_only = false;
    for (int i = 1; i < argc; i++) {
        if (!std::strcmp(argv[i], "--test") && i + 1 < argc) only = argv[++i];
        else if (!std::strcmp(argv[i], "--wave")) wave_on = true;
        else if (!std::strcmp(argv[i], "--list")) list_only = true;
        else if (!std::strcmp(argv[i], "--json") && i + 1 < argc) json_path = argv[++i];
    }

    if (list_only) {
        for (auto& t : test_registry()) std::printf("%s\n", t.name);
        return 0;
    }

    dut = new Vnmc_top;
    if (wave_on) {
        Verilated::traceEverOn(true);
        vcd = new VerilatedVcdC;
        dut->trace(vcd, 99);
        vcd->open("build/waves.vcd");
    }

    int failures = 0;
    int ran = 0;
    for (auto& t : test_registry()) {
        if (!only.empty() && only != t.name) continue;
        ran++;

        // fresh reset per test
        dut->rst_n = 0;
        dut->flit_rx_valid = 0;
        dut->flit_tx_ready = 1;
        for (int c = 0; c < 10; c++) tick_once();
        dut->rst_n = 1;
        for (int c = 0; c < 2; c++) tick_once();

        DutPins pins;
        pins.drive_rx_data = [](const uint8_t* b68) {
            // 544-bit port = 17 x 32-bit words, byte k at word k/4 lane k%4
            std::memcpy(dut->flit_rx_data.data(), b68, FLIT_BYTES);
        };
        pins.drive_rx_valid  = [](int v) { dut->flit_rx_valid = v; };
        pins.sample_rx_ready = []() { return (int)dut->flit_rx_ready; };
        pins.sample_tx_valid = []() { return (int)dut->flit_tx_valid; };
        pins.sample_tx_data  = [](const std::function<void(uint8_t*)>& cb) {
            uint8_t raw[FLIT_BYTES];
            std::memcpy(raw, dut->flit_tx_data.data(), FLIT_BYTES);
            cb(raw);
        };
        pins.drive_tx_ready = [](int v) { dut->flit_tx_ready = v; };
        pins.tick           = []() { tick_once(); };

        CxlHostModel host(std::move(pins));

        g_check_fails() = 0;
        g_current_test() = t.name;
        std::printf("[ RUN  ] %s\n", t.name);
        bool threw = false;
        std::string why;
        try {
            t.fn(host);
        } catch (const std::exception& e) {
            threw = true;
            why = e.what();
        }
        if (threw) {
            std::printf("[ FAIL ] %s (exception: %s)\n", t.name, why.c_str());
            failures++;
        } else if (g_check_fails() != 0) {
            std::printf("[ FAIL ] %s (%d checks)\n", t.name, g_check_fails());
            failures++;
        } else {
            std::printf("[ PASS ] %s\n", t.name);
        }
    }

    if (!g_metrics().empty()) {
        std::ofstream js(json_path);
        js << "{\n";
        size_t i = 0;
        for (auto& kv : g_metrics()) {
            js << "  \"" << kv.first << "\": " << kv.second;
            if (++i != g_metrics().size()) js << ",";
            js << "\n";
        }
        js << "}\n";
    }

    if (wave_on) vcd->close();
    std::printf("\n%d/%d tests passed\n", ran - failures, ran);
    delete dut;
    return failures == 0 ? 0 : 1;
}
