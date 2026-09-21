#ifndef IMU_TEST_FRAMEWORK_H
#define IMU_TEST_FRAMEWORK_H

#include <math.h>
#include <stdio.h>

static unsigned test_assertions;
static unsigned test_failures;

#define TEST_ASSERT(condition) do { \
    ++test_assertions; \
    if (!(condition)) { \
        ++test_failures; \
        fprintf(stderr, "%s:%d: assertion failed: %s\n", \
                __FILE__, __LINE__, #condition); \
    } \
} while (0)

/* Evaluate each argument once; NaN/Inf must never compare as near. */
#define TEST_ASSERT_NEAR(actual, expected, tolerance) do { \
    const double test_actual_ = (double)(actual); \
    const double test_expected_ = (double)(expected); \
    const double test_tolerance_ = (double)(tolerance); \
    TEST_ASSERT(isfinite(test_actual_) && isfinite(test_expected_) && \
                isfinite(test_tolerance_) && test_tolerance_ >= 0.0 && \
                fabs(test_actual_ - test_expected_) <= test_tolerance_); \
} while (0)

#define TEST_ASSERT_VEC3_NEAR(actual, expected, tolerance) do { \
    const imu_vec3f_t test_vector_actual_ = (actual); \
    const imu_vec3f_t test_vector_expected_ = (expected); \
    const float test_vector_tolerance_ = (tolerance); \
    TEST_ASSERT_NEAR(test_vector_actual_.x, test_vector_expected_.x, test_vector_tolerance_); \
    TEST_ASSERT_NEAR(test_vector_actual_.y, test_vector_expected_.y, test_vector_tolerance_); \
    TEST_ASSERT_NEAR(test_vector_actual_.z, test_vector_expected_.z, test_vector_tolerance_); \
} while (0)

static int test_summary(void)
{
    printf("%u assertions, %u failures\n", test_assertions, test_failures);
    return test_failures == 0U ? 0 : 1;
}

#endif
