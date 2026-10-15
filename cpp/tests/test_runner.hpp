// A minimal test runner: TEST registers a function, CHECK records a failure and carries on.
#pragma once

#include <iostream>
#include <string>
#include <vector>

namespace aegis_test {

struct TestCase {
    const char* name;
    void (*fn)();
};

std::vector<TestCase>& registry();
void report_failure(const char* file, int line, const std::string& what);

struct Registrar {
    Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

}  // namespace aegis_test

#define TEST(name)                                                       \
    static void name();                                                  \
    static const aegis_test::Registrar name##_registrar{#name, &name};   \
    static void name()

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) aegis_test::report_failure(__FILE__, __LINE__, #cond); \
    } while (0)

#define CHECK_EQ(a, b)                                                   \
    do {                                                                 \
        const auto& check_a_ = (a);                                      \
        const auto& check_b_ = (b);                                      \
        if (!(check_a_ == check_b_)) {                                   \
            aegis_test::report_failure(                                  \
                __FILE__, __LINE__,                                      \
                std::string(#a " == " #b " (") + std::to_string(check_a_) + \
                    " vs " + std::to_string(check_b_) + ")");            \
        }                                                                \
    } while (0)
