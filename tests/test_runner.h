#ifndef TEST_RUNNER_H
#define TEST_RUNNER_H
#include <stdio.h>
#include <stdint.h>

static int testsRun, testsFailed, currentFailed;

#define TEST(name) static void name(void)
#define CHECK(cond) do { if (!(cond)) { \
    printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); currentFailed = 1; } } while (0)
#define CHECK_EQ(a, b) do { long _a = (long)(a), _b = (long)(b); if (_a != _b) { \
    printf("  FAIL %s:%d: %s == %s (%ld vs %ld)\n", __FILE__, __LINE__, #a, #b, _a, _b); \
    currentFailed = 1; } } while (0)
#define RUN(fn) do { currentFailed = 0; testsRun++; fn(); \
    if (currentFailed) { testsFailed++; printf("FAILED %s\n", #fn); } \
    else { printf("ok     %s\n", #fn); } } while (0)
#define TEST_SUMMARY() (printf("\n%d run, %d failed\n", testsRun, testsFailed), testsFailed ? 1 : 0)
#endif
