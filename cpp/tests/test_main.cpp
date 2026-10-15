#include "test_runner.hpp"

#include <cstring>

namespace aegis_test {

static int g_failures = 0;

std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

void report_failure(const char* file, int line, const std::string& what) {
    ++g_failures;
    std::cerr << file << ":" << line << ": CHECK failed: " << what << "\n";
}

}  // namespace aegis_test

// Usage: test_x [name]  (runs every test, or only the one called name)
int main(int argc, char** argv) {
    int run = 0;
    for (const auto& test : aegis_test::registry()) {
        if (argc > 1 && std::strcmp(argv[1], test.name) != 0) continue;
        const int before = aegis_test::g_failures;
        test.fn();
        ++run;
        std::cout << (aegis_test::g_failures == before ? "[ ok ] " : "[FAIL] ") << test.name << "\n";
    }
    std::cout << run << " tests, " << aegis_test::g_failures << " failed checks\n";
    return (run > 0 && aegis_test::g_failures == 0) ? 0 : 1;
}
