// Serial (Windows COM) backend for CxlHostModel's DutPins interface: the
// same host model — transactions, credits, acks, golden CRC — that drives
// Verilator pins in simulation drives a real board over the UART flit bridge.
//
// Mapping: tick() polls the COM port and feeds a frame assembler
// (0xA5 0x5A + 68 bytes, both directions, matching uart_flit_bridge.sv);
// "tx_valid" means a complete device frame is waiting; driving rx_valid high
// writes the host frame out. rx_ready is always 1 — the UART is ~1000x
// slower than the core clock, so the device always drains, and the OS/FTDI
// buffers absorb any skew. Cycle-denominated timeouts become poll counts.
#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>
#include <windows.h>
#include "../sim/cxl_host_model.hpp"

class SerialPort {
  public:
    SerialPort(const std::string& name, uint32_t baud) {
        std::string path = "\\\\.\\" + name;
        h_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                         OPEN_EXISTING, 0, nullptr);
        if (h_ == INVALID_HANDLE_VALUE)
            throw std::runtime_error("cannot open " + name);
        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        GetCommState(h_, &dcb);
        dcb.BaudRate = baud;
        dcb.ByteSize = 8;
        dcb.Parity   = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary  = TRUE;
        if (!SetCommState(h_, &dcb))
            throw std::runtime_error("SetCommState failed (baud unsupported?)");
        COMMTIMEOUTS to{};
        to.ReadIntervalTimeout        = MAXDWORD;   // nonblocking reads
        to.ReadTotalTimeoutConstant   = 0;
        to.ReadTotalTimeoutMultiplier = 0;
        SetCommTimeouts(h_, &to);
        PurgeComm(h_, PURGE_RXCLEAR | PURGE_TXCLEAR);
    }
    ~SerialPort() { if (h_ != INVALID_HANDLE_VALUE) CloseHandle(h_); }

    void write_all(const uint8_t* d, size_t n) {
        DWORD done = 0;
        while (n) {
            if (!WriteFile(h_, d, DWORD(n), &done, nullptr))
                throw std::runtime_error("serial write failed");
            d += done; n -= done;
        }
    }
    size_t read_some(uint8_t* d, size_t cap) {
        DWORD got = 0;
        if (!ReadFile(h_, d, DWORD(cap), &got, nullptr)) return 0;
        return got;
    }

  private:
    HANDLE h_ = INVALID_HANDLE_VALUE;
};

class SerialDut {
  public:
    SerialDut(const std::string& com, uint32_t baud) : port_(com, baud) {}

    DutPins pins() {
        DutPins p;
        p.drive_rx_data  = [this](const uint8_t* b) { std::memcpy(txbuf_, b, FLIT_BYTES); };
        p.drive_rx_valid = [this](int v) {
            if (v && !rx_valid_) {                 // rising edge: send the frame
                uint8_t hdr[2] = {0xA5, 0x5A};
                port_.write_all(hdr, 2);
                port_.write_all(txbuf_, FLIT_BYTES);
            }
            rx_valid_ = (v != 0);
        };
        p.sample_rx_ready = []() { return 1; };
        p.sample_tx_valid = [this]() { return frames_.empty() ? 0 : 1; };
        p.sample_tx_data  = [this](const std::function<void(uint8_t*)>& cb) {
            cb(frames_.front().data());
            frames_.pop_front();
        };
        p.drive_tx_ready = [](int) {};
        p.tick = [this]() { poll_(); };
        return p;
    }

  private:
    void poll_() {
        uint8_t buf[512];
        size_t n = port_.read_some(buf, sizeof(buf));
        for (size_t i = 0; i < n; i++) {
            uint8_t b = buf[i];
            switch (hunt_) {
                case 0: hunt_ = (b == 0xA5) ? 1 : 0; break;
                case 1: hunt_ = (b == 0x5A) ? 2 : (b == 0xA5 ? 1 : 0);
                        body_.clear(); break;
                case 2:
                    body_.push_back(b);
                    if (int(body_.size()) == FLIT_BYTES) {
                        std::array<uint8_t, FLIT_BYTES> f{};
                        std::memcpy(f.data(), body_.data(), FLIT_BYTES);
                        frames_.push_back(f);
                        hunt_ = 0;
                    }
                    break;
            }
        }
    }

    SerialPort port_;
    uint8_t    txbuf_[FLIT_BYTES] = {};
    bool       rx_valid_ = false;
    int        hunt_ = 0;
    std::vector<uint8_t> body_;
    std::deque<std::array<uint8_t, FLIT_BYTES>> frames_;
};
