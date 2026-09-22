#include "test_framework.h"
#include "imu/imu_calibration.h"

#include <float.h>
#include <string.h>

static imu_calibration_t valid_calibration(void)
{
    imu_calibration_t calibration = {0};

    calibration.version = IMU_CALIBRATION_VERSION;
    calibration.valid = true;
    calibration.checksum = UINT32_C(0x12345678);
    calibration.checksum_valid = true;
    calibration.temperature_min_c = 0.0f;
    calibration.temperature_max_c = 50.0f;
    calibration.temperature_reference_c = 10.0f;
    calibration.gyro_scale_alignment[0][0] = 2.0f;
    calibration.gyro_scale_alignment[0][1] = 1.0f;
    calibration.gyro_scale_alignment[1][1] = 3.0f;
    calibration.gyro_scale_alignment[2][2] = -1.0f;
    calibration.accel_scale_alignment[0][0] = 1.0f;
    calibration.accel_scale_alignment[0][1] = 0.5f;
    calibration.accel_scale_alignment[1][1] = 2.0f;
    calibration.accel_scale_alignment[2][2] = 0.5f;
    calibration.body_rotation[0][1] = -1.0f;
    calibration.body_rotation[1][0] = 1.0f;
    calibration.body_rotation[2][2] = 1.0f;
    calibration.gyro_bias_poly[0][0] = 1.0f;
    calibration.gyro_bias_poly[0][1] = 2.0f;
    calibration.gyro_bias_poly[0][2] = 3.0f;
    calibration.gyro_bias_poly[1][0] = -1.0f;
    calibration.gyro_bias_poly[1][1] = 0.5f;
    calibration.gyro_bias_poly[1][2] = 0.25f;
    calibration.gyro_bias_poly[2][0] = 0.5f;
    calibration.gyro_bias_poly[2][1] = -1.0f;
    calibration.gyro_bias_poly[2][2] = 0.125f;
    calibration.accel_bias_poly[0][0] = 0.1f;
    calibration.accel_bias_poly[0][1] = 0.2f;
    calibration.accel_bias_poly[0][2] = 0.03f;
    calibration.accel_bias_poly[1][0] = -2.0f;
    calibration.accel_bias_poly[1][1] = 0.1f;
    calibration.accel_bias_poly[2][0] = 1.0f;
    calibration.accel_bias_poly[2][2] = -0.02f;
    return calibration;
}

static imu_sample_t valid_sample(void)
{
    imu_sample_t input = {0};

    input.timestamp_ns = UINT64_C(987654321);
    input.gyro_rad_s[0] = 1.0f;
    input.gyro_rad_s[1] = 2.0f;
    input.gyro_rad_s[2] = 3.0f;
    input.gyro_temperature_c[0] = 12.0f;
    input.gyro_temperature_c[1] = 13.0f;
    input.gyro_temperature_c[2] = 14.0f;
    input.accel_m_s2[0] = 4.0f;
    input.accel_m_s2[1] = 6.0f;
    input.accel_m_s2[2] = 8.0f;
    input.accel_temperature_c = 15.0f;
    return input;
}

static void assert_rejected_without_mutation(const imu_calibration_t *calibration,
                                             const imu_sample_t *input)
{
    imu_calibrated_sample_t output;
    imu_calibrated_sample_t before;

    memset(&output, 0xA5, sizeof(output));
    before = output;
    TEST_ASSERT(!imu_calibrate_sample(calibration, input, &output));
    TEST_ASSERT(memcmp(&output, &before, sizeof(output)) == 0);
}

/* Catches reversed bias/matrix order, linear-only polynomials, and one shared
 * gyro temperature being used for all three independently heated gyros. */
static void test_calibration_order_and_horner_polynomials(void)
{
    const imu_calibration_t calibration = valid_calibration();
    const imu_sample_t input = valid_sample();
    imu_calibrated_sample_t output = {0};

    TEST_ASSERT(imu_calibrate_sample(&calibration, &input, &output));
    TEST_ASSERT(output.timestamp_ns == input.timestamp_ns);
    TEST_ASSERT_VEC3_NEAR(output.gyro_rad_s,
                          ((imu_vec3f_t){-3.25f, -13.0f, -1.5f}), 1.0e-6f);
    TEST_ASSERT_VEC3_NEAR(output.accel_m_s2,
                          ((imu_vec3f_t){-13.5f, 5.15f, 3.5f}), 1.0e-5f);
    TEST_ASSERT_NEAR(output.gyro_temperature_c[0], 12.0f, 0.0f);
    TEST_ASSERT_NEAR(output.gyro_temperature_c[1], 13.0f, 0.0f);
    TEST_ASSERT_NEAR(output.gyro_temperature_c[2], 14.0f, 0.0f);
    TEST_ASSERT_NEAR(output.accel_temperature_c, 15.0f, 0.0f);
}

/* Catches accepting stale/wrong calibration records or malformed ranges. */
static void test_metadata_contract(void)
{
    imu_calibration_t calibration = valid_calibration();
    const imu_sample_t input = valid_sample();

    calibration.valid = false;
    assert_rejected_without_mutation(&calibration, &input);
    calibration = valid_calibration();
    calibration.checksum_valid = false;
    assert_rejected_without_mutation(&calibration, &input);
    calibration = valid_calibration();
    calibration.version = IMU_CALIBRATION_VERSION + 1U;
    assert_rejected_without_mutation(&calibration, &input);
    calibration = valid_calibration();
    calibration.temperature_min_c = 20.0f;
    calibration.temperature_max_c = 10.0f;
    assert_rejected_without_mutation(&calibration, &input);
    calibration = valid_calibration();
    calibration.temperature_reference_c = 60.0f;
    assert_rejected_without_mutation(&calibration, &input);
}

/* Catches accepting scaled/skewed/reflected board transforms as rotations. */
static void test_board_rotation_validation(void)
{
    imu_calibration_t calibration = valid_calibration();
    const imu_sample_t input = valid_sample();

    calibration.body_rotation[0][1] = -2.0f;
    assert_rejected_without_mutation(&calibration, &input);
    calibration = valid_calibration();
    calibration.body_rotation[2][2] = -1.0f;
    assert_rejected_without_mutation(&calibration, &input);
}

/* Catches unchecked coefficients, raw samples, temperatures, and overflow. */
static void test_nonfinite_rejection(void)
{
    imu_calibration_t calibration = valid_calibration();
    imu_sample_t input = valid_sample();

    calibration.gyro_bias_poly[2][1] = NAN;
    assert_rejected_without_mutation(&calibration, &input);
    calibration = valid_calibration();
    calibration.accel_scale_alignment[1][2] = INFINITY;
    assert_rejected_without_mutation(&calibration, &input);
    calibration = valid_calibration();
    input.gyro_rad_s[1] = NAN;
    assert_rejected_without_mutation(&calibration, &input);
    input = valid_sample();
    input.accel_temperature_c = INFINITY;
    assert_rejected_without_mutation(&calibration, &input);
    input = valid_sample();
    input.accel_m_s2[0] = FLT_MAX;
    calibration.accel_scale_alignment[0][0] = FLT_MAX;
    assert_rejected_without_mutation(&calibration, &input);
}

/* Catches exclusive endpoints and silently extrapolating without permission. */
static void test_temperature_range_and_extrapolation(void)
{
    imu_calibration_t calibration = valid_calibration();
    imu_sample_t input = valid_sample();
    imu_calibrated_sample_t output = {0};

    input.gyro_temperature_c[0] = calibration.temperature_min_c;
    input.gyro_temperature_c[1] = calibration.temperature_max_c;
    input.accel_temperature_c = calibration.temperature_max_c;
    TEST_ASSERT(imu_calibrate_sample(&calibration, &input, &output));
    input.gyro_temperature_c[2] = calibration.temperature_max_c + 0.5f;
    assert_rejected_without_mutation(&calibration, &input);
    calibration.allow_temperature_extrapolation = true;
    TEST_ASSERT(imu_calibrate_sample(&calibration, &input, &output));
    TEST_ASSERT_NEAR(output.gyro_temperature_c[2], 50.5f, 0.0f);
}

static void test_pointer_validation(void)
{
    const imu_calibration_t calibration = valid_calibration();
    const imu_sample_t input = valid_sample();
    imu_calibrated_sample_t output = {0};

    TEST_ASSERT(!imu_calibrate_sample(NULL, &input, &output));
    TEST_ASSERT(!imu_calibrate_sample(&calibration, NULL, &output));
    TEST_ASSERT(!imu_calibrate_sample(&calibration, &input, NULL));
}

int main(void)
{
    test_calibration_order_and_horner_polynomials();
    test_metadata_contract();
    test_board_rotation_validation();
    test_nonfinite_rejection();
    test_temperature_range_and_extrapolation();
    test_pointer_validation();
    return test_summary();
}
