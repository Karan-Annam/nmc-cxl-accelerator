// Minimal test framework: registration macro, checks, metrics.
#pragma once
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <string>
#include <vector>

class CxlHostModel;

struct TestCase {
    const char* name;
    std::function<void(CxlHostModel&)> fn;
};

inline std::vector<TestCase>& test_registry() {
    static std::vector<TestCase> r;
    return r;
}

struct TestRegistrar {
    TestRegistrar(const char* name, std::function<void(CxlHostModel&)> fn) {
        test_registry().push_back({name, std::move(fn)});
    }
};

// Per-test failure state
inline int&         g_check_fails()  { static int n = 0; return n; }
inline std::string& g_current_test() { static std::string s; return s; }

// Metrics collected for build/results.json (dotted keys, grouped by the web page)
inline std::map<std::string, double>& g_metrics() {
    static std::map<std::string, double> m;
    return m;
}
inline void metric(const std::string& key, double value) {
    g_metrics()[key] = value;
}

#define TEST(tname)                                                          \
    static void tname(CxlHostModel& host);                                   \
    static TestRegistrar reg_##tname(#tname, tname);                         \
    static void tname(CxlHostModel& host)

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("    CHECK failed: %s (%s:%d)\n", #cond, __FILE__,   \
                        __LINE__);                                           \
            g_check_fails()++;                                               \
        }                                                                    \
    } while (0)

#define CHECK_EQ(a, b)                                                       \
    do {                                                                     \
        auto va = (a);                                                       \
        auto vb = (b);                                                       \
        if (!(va == vb)) {                                                   \
            std::printf("    CHECK_EQ failed: %s=%lld  %s=%lld  (%s:%d)\n",  \
                        #a, (long long)va, #b, (long long)vb, __FILE__,      \
                        __LINE__);                                           \
            g_check_fails()++;                                               \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                \
    do {                                                                     \
        double va = (double)(a);                                             \
        double vb = (double)(b);                                             \
        double d  = va > vb ? va - vb : vb - va;                             \
        if (d > (tol)) {                                                     \
            std::printf("    CHECK_NEAR failed: %s=%g  %s=%g  tol=%g "       \
                        "(%s:%d)\n",                                         \
                        #a, va, #b, vb, (double)(tol), __FILE__, __LINE__);  \
            g_check_fails()++;                                               \
        }                                                                    \
    } while (0)
