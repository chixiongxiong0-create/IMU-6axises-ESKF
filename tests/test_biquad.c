#include "test_framework.h"
#include "imu/imu_biquad.h"

#include <float.h>
#include <stdint.h>
#include <string.h>

#define TEST_SAMPLE_RATE_HZ 485.0f
#define TEST_PI 3.14159265358979323846

static uint32_t float_bits(float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static bool vec3_bits_equal(imu_vec3f_t lhs, imu_vec3f_t rhs)
{
    return float_bits(lhs.x) == float_bits(rhs.x) &&
           float_bits(lhs.y) == float_bits(rhs.y) &&
           float_bits(lhs.z) == float_bits(rhs.z);
}

/* Catches bypass paths that perform arithmetic, canonicalize NaNs, or alter
 * signed zero instead of forwarding all input bits unchanged. */
static void test_bypass_is_bitwise_transparent(void)
{
    imu_biquad_t filter = {0};
    imu_vec3f_t input;
    imu_vec3f_t output = {1.0f, 2.0f, 3.0f};
    const uint32_t patterns[3] = {
        UINT32_C(0x80000000), UINT32_C(0x7FC12345), UINT32_C(0x3F800001)
    };

    memcpy(&input, patterns, sizeof(input));
    TEST_ASSERT(imu_biquad_process(&filter, input, &output));
    TEST_ASSERT(memcmp(&output, &input, sizeof(output)) == 0);
    TEST_ASSERT(memcmp(&filter.z1, &(imu_vec3f_t){0}, sizeof(filter.z1)) == 0);
    TEST_ASSERT(memcmp(&filter.z2, &(imu_vec3f_t){0}, sizeof(filter.z2)) == 0);
}

/* Catches reset implementations that leave either delay state behind. */
static void test_reset_replays_deterministically(void)
{
    imu_biquad_t filter = {0};
    imu_vec3f_t first[8];
    imu_vec3f_t second[8];
    unsigned i;

    TEST_ASSERT(imu_biquad_configure_notch(&filter, TEST_SAMPLE_RATE_HZ,
                                            100.0f, 8.0f));
    for (i = 0U; i < 8U; ++i) {
        const float value = i == 0U ? 1.0f : 0.0f;
        TEST_ASSERT(imu_biquad_process(&filter,
                                       (imu_vec3f_t){value, -value, 0.5f * value},
                                       &first[i]));
    }
    imu_biquad_reset(&filter);
    for (i = 0U; i < 8U; ++i) {
        const float value = i == 0U ? 1.0f : 0.0f;
        TEST_ASSERT(imu_biquad_process(&filter,
                                       (imu_vec3f_t){value, -value, 0.5f * value},
                                       &second[i]));
        TEST_ASSERT(vec3_bits_equal(first[i], second[i]));
    }
}

/* Catches unnormalized, singular, or nonfinite coefficient generation. */
static void test_configuration_normalizes_finite_coefficients(void)
{
    imu_biquad_t filter;

    memset(&filter, 0xA5, sizeof(filter));
    TEST_ASSERT(imu_biquad_configure_notch(&filter, TEST_SAMPLE_RATE_HZ,
                                            100.0f, 8.0f));
    TEST_ASSERT(filter.enabled);
    TEST_ASSERT(isfinite(filter.b0));
    TEST_ASSERT(isfinite(filter.b1));
    TEST_ASSERT(isfinite(filter.b2));
    TEST_ASSERT(isfinite(filter.a1));
    TEST_ASSERT(isfinite(filter.a2));
    TEST_ASSERT_NEAR(filter.b0, 0.9432658f, 2.0e-6f);
    TEST_ASSERT_NEAR(filter.b1, -0.5128160f, 2.0e-6f);
    TEST_ASSERT_NEAR(filter.b2, 0.9432658f, 2.0e-6f);
    TEST_ASSERT_NEAR(filter.a1, -0.5128160f, 2.0e-6f);
    TEST_ASSERT_NEAR(filter.a2, 0.8865316f, 2.0e-6f);
}

static double filtered_rms(float frequency_hz)
{
    imu_biquad_t filter = {0};
    double energy = 0.0;
    unsigned i;

    TEST_ASSERT(imu_biquad_configure_notch(&filter, TEST_SAMPLE_RATE_HZ,
                                            100.0f, 8.0f));
    for (i = 0U; i < 4096U; ++i) {
        const float sample = (float)sin(2.0 * TEST_PI * (double)frequency_hz *
                                        (double)i / TEST_SAMPLE_RATE_HZ);
        imu_vec3f_t output = {0};

        TEST_ASSERT(imu_biquad_process(&filter,
                                       (imu_vec3f_t){sample, sample, sample},
                                       &output));
        if (i >= 2048U) {
            energy += (double)output.x * (double)output.x;
        }
    }
    return sqrt(energy / 2048.0);
}

/* Catches wrong-sign/state equations and incorrect angular-frequency use. */
static void test_notch_attenuates_100_hz_but_preserves_10_hz(void)
{
    const double rms_100_hz = filtered_rms(100.0f);
    const double rms_10_hz = filtered_rms(10.0f);

    TEST_ASSERT(rms_100_hz < 0.01);
    TEST_ASSERT(rms_10_hz > 0.68);
    TEST_ASSERT(rms_10_hz > 50.0 * rms_100_hz);
}

static void assert_config_rejected(float sample_rate_hz, float frequency_hz,
                                   float q)
{
    imu_biquad_t filter = {0};
    imu_biquad_t before;

    TEST_ASSERT(imu_biquad_configure_notch(&filter, TEST_SAMPLE_RATE_HZ,
                                            100.0f, 8.0f));
    before = filter;
    TEST_ASSERT(!imu_biquad_configure_notch(&filter, sample_rate_hz,
                                             frequency_hz, q));
    TEST_ASSERT(memcmp(&filter, &before, sizeof(filter)) == 0);
}

/* Catches inclusive endpoints and comparison-only validation that admits NaN. */
static void test_configuration_rejects_invalid_parameters_transactionally(void)
{
    assert_config_rejected(TEST_SAMPLE_RATE_HZ, 0.0f, 1.0f);
    assert_config_rejected(TEST_SAMPLE_RATE_HZ,
                           0.49f * TEST_SAMPLE_RATE_HZ, 1.0f);
    assert_config_rejected(TEST_SAMPLE_RATE_HZ, 10.0f, 0.0f);
    assert_config_rejected(TEST_SAMPLE_RATE_HZ, 10.0f, -1.0f);
    assert_config_rejected(0.0f, 10.0f, 1.0f);
    assert_config_rejected(NAN, 10.0f, 1.0f);
    assert_config_rejected(TEST_SAMPLE_RATE_HZ, NAN, 1.0f);
    assert_config_rejected(TEST_SAMPLE_RATE_HZ, 10.0f, INFINITY);
    TEST_ASSERT(!imu_biquad_configure_notch(NULL, TEST_SAMPLE_RATE_HZ,
                                             10.0f, 1.0f));
}

/* Catches reconfiguration that carries stale delay state into new settings. */
static void test_reconfiguration_resets_state(void)
{
    imu_biquad_t used = {0};
    imu_biquad_t fresh = {0};
    imu_vec3f_t used_output;
    imu_vec3f_t fresh_output;
    unsigned i;

    TEST_ASSERT(imu_biquad_configure_notch(&used, TEST_SAMPLE_RATE_HZ,
                                            100.0f, 8.0f));
    for (i = 0U; i < 20U; ++i) {
        TEST_ASSERT(imu_biquad_process(&used, (imu_vec3f_t){1.0f, -2.0f, 3.0f},
                                       &used_output));
    }
    TEST_ASSERT(imu_biquad_configure_notch(&used, TEST_SAMPLE_RATE_HZ,
                                            80.0f, 4.0f));
    TEST_ASSERT(imu_biquad_configure_notch(&fresh, TEST_SAMPLE_RATE_HZ,
                                            80.0f, 4.0f));
    TEST_ASSERT(imu_biquad_process(&used, (imu_vec3f_t){0.25f, -0.5f, 1.0f},
                                   &used_output));
    TEST_ASSERT(imu_biquad_process(&fresh, (imu_vec3f_t){0.25f, -0.5f, 1.0f},
                                   &fresh_output));
    TEST_ASSERT(vec3_bits_equal(used_output, fresh_output));
}

/* Catches enabled processing that accepts nonfinite input or partially mutates
 * state/output before reporting failure. */
static void test_nonfinite_input_is_rejected_without_mutation(void)
{
    imu_biquad_t filter = {0};
    imu_biquad_t before_filter;
    imu_vec3f_t output;
    imu_vec3f_t before_output;

    TEST_ASSERT(imu_biquad_configure_notch(&filter, TEST_SAMPLE_RATE_HZ,
                                            100.0f, 8.0f));
    TEST_ASSERT(imu_biquad_process(&filter, (imu_vec3f_t){1.0f, 2.0f, 3.0f},
                                   &output));
    before_filter = filter;
    memset(&output, 0xA5, sizeof(output));
    before_output = output;
    TEST_ASSERT(!imu_biquad_process(&filter, (imu_vec3f_t){1.0f, NAN, 3.0f},
                                    &output));
    TEST_ASSERT(memcmp(&filter, &before_filter, sizeof(filter)) == 0);
    TEST_ASSERT(memcmp(&output, &before_output, sizeof(output)) == 0);
    TEST_ASSERT(!imu_biquad_process(NULL, (imu_vec3f_t){0}, &output));
    TEST_ASSERT(!imu_biquad_process(&filter, (imu_vec3f_t){0}, NULL));
}

int main(void)
{
    test_bypass_is_bitwise_transparent();
    test_reset_replays_deterministically();
    test_configuration_normalizes_finite_coefficients();
    test_notch_attenuates_100_hz_but_preserves_10_hz();
    test_configuration_rejects_invalid_parameters_transactionally();
    test_reconfiguration_resets_state();
    test_nonfinite_input_is_rejected_without_mutation();
    return test_summary();
}
