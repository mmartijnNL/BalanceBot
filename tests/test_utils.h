#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <iostream>
#include <cmath>
#include <string>

// Global variable to track test failures
inline int g_failures = 0;

// Utility function to check a condition and log failures
inline void expect_true(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "Test failed: " << message << std::endl;
        ++g_failures;
    }
}

// Utility function to check if a string contains a substring
inline void expect_contains(const std::string& str, const std::string& substring, const std::string& message) {
    if (str.find(substring) == std::string::npos) {
        std::cerr << "Test failed: " << message << std::endl;
        ++g_failures;
    }
}

// Utility function to check if two floating-point numbers are approximately equal
inline void expect_near(float value, float expected, float tolerance, const std::string& message) {
    if (std::fabs(value - expected) > tolerance) {
        std::cerr << "Test failed: " << message << " (" << value << " != " << expected << ")" << std::endl;
        ++g_failures;
    }
}

#endif // TEST_UTILS_H