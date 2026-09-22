#include "test_framework.h"
#include "imu/imu_motion_quality.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#define TEST_DT_NS UINT64_C(10000000)

static imu_config_t quality_config(void)
{
    imu_config_t config = {0};

    config.gravity_m_s2 = 9.80665f;
    config.nominal_dt_s = 0.01f;
    config.quality_window_samples = 8U;
    config.accel_norm_soft_m_s2 = 0.2f;
    config.accel_norm_hard_m_s2 = 2.0f;
    config.vibration_soft_m_s2 = 0.1f;
    config.vibration_hard_m_s2 = 0.8f;
    config.jerk_soft_m_s3 = 5.0f;
    config.jerk_hard_m_s3 = 80.0f;
    config.innovation_soft_rad = 0.08f;
    config.innovation_hard_rad = 0.30f;
    config.innovation_persistence_s = 0.05f;
    config.accel_weight_min = 0.01f;
    config.weight_rise_per_s = 100.0f;
    config.weight_fall_per_s = 100.0f;
    config.static_gyro_max_rad_s = 0.02f;
    config.static_gyro_variance_max = 1.0e-4f;
    config.static_accel_norm_max_m_s2 = 0.10f;
    config.static_accel_variance_max = 0.01f;
    config.static_jerk_max_m_s3 = 2.0f;
    config.static_vibration_max_m_s2 = 0.05f;
    config.static_enter_samples = 4U;
    config.static_exit_samples = 3U;
    return config;
}

static imu_sample_t sample_at(unsigned index)
{
    imu_sample_t sample = {0};
    unsigned axis;

    sample.timestamp_ns = ((uint64_t)index + UINT64_C(1)) * TEST_DT_NS;
    sample.accel_m_s2[2] = -9.80665f;
    sample.accel_status = IMU_SENSOR_DATA_VALID;
    sample.accel_sample_fresh = true;
    for (axis = 0U; axis < IMU_AXIS_COUNT; ++axis) {
        sample.gyro_status[axis] = IMU_SENSOR_DATA_VALID;
        sample.gyro_sample_fresh[axis] = true;
    }
    return sample;
}

static void warm_stable(imu_motion_quality_t *quality,
                        const imu_config_t *config,
                        unsigned count)
{
    unsigned i;

    for (i = 0U; i < count; ++i) {
        const imu_sample_t sample = sample_at(i);
        TEST_ASSERT(imu_motion_quality_update(quality, config, &sample, 0.0f));
    }
}

/* Catches treating a stationary one-g input as vibration and entry-dwell
 * comparisons that switch static one sample too early or late. */
static void test_stable_one_g_reaches_full_quality_and_static_after_dwell(void)
{
    const imu_config_t config = quality_config();
    imu_motion_quality_t quality;
    unsigned i;

    TEST_ASSERT(imu_motion_quality_init(&quality, &config));
    for (i = 0U; i < config.static_enter_samples - 1U; ++i) {
        const imu_sample_t sample = sample_at(i);
        TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.0f));
    }
    TEST_ASSERT(!quality.static_detected);
    {
        const imu_sample_t sample = sample_at(config.static_enter_samples - 1U);
        TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.0f));
    }
    TEST_ASSERT(quality.static_detected);
    TEST_ASSERT_NEAR(quality.accel_weight, 1.0f, 1.0e-6f);
    TEST_ASSERT_NEAR(quality.vibration_rms, 0.0f, 1.0e-6f);
    TEST_ASSERT_NEAR(quality.jerk, 0.0f, 1.0e-6f);
    TEST_ASSERT(quality.mode_reasons == IMU_MOTION_REASON_NONE);
}

/* Catches missing magnitude and jerk gates on an isolated severe shock. */
static void test_four_g_impulse_reduces_quality(void)
{
    imu_config_t config = quality_config();
    imu_motion_quality_t quality;
    imu_sample_t impulse;
    float before;

    config.weight_fall_per_s = 20.0f;
    TEST_ASSERT(imu_motion_quality_init(&quality, &config));
    warm_stable(&quality, &config, 10U);
    before = quality.accel_weight;
    impulse = sample_at(10U);
    impulse.accel_m_s2[2] = -4.0f * config.gravity_m_s2;
    TEST_ASSERT(imu_motion_quality_update(&quality, &config, &impulse, 0.0f));
    TEST_ASSERT(quality.accel_weight < before);
    TEST_ASSERT(quality.jerk > config.jerk_hard_m_s3);
    TEST_ASSERT((quality.mode_reasons & IMU_MOTION_REASON_ACCEL_NORM) != 0U);
    TEST_ASSERT((quality.mode_reasons & IMU_MOTION_REASON_JERK) != 0U);
}

/* Catches implementations that inspect magnitude only and miss alternating
 * directional vibration whose total magnitude remains close to one g. */
static void test_alternating_vibration_builds_high_pass_rms(void)
{
    imu_config_t config = quality_config();
    imu_motion_quality_t quality;
    unsigned i;

    config.weight_fall_per_s = 50.0f;
    TEST_ASSERT(imu_motion_quality_init(&quality, &config));
    for (i = 0U; i < 16U; ++i) {
        imu_sample_t sample = sample_at(i);
        sample.accel_m_s2[0] = (i & 1U) == 0U ? 1.5f : -1.5f;
        TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.0f));
    }
    TEST_ASSERT(quality.vibration_rms > 1.0f);
    TEST_ASSERT(quality.accel_weight < 0.1f);
    TEST_ASSERT((quality.mode_reasons & IMU_MOTION_REASON_VIBRATION) != 0U);
}

/* Catches instantaneous-only innovation scoring: a constant tilted vector at
 * exactly one-g magnitude must lose confidence as disagreement persists. */
static void test_persistent_lateral_acceleration_decays_plausible_norm_weight(void)
{
    imu_config_t config = quality_config();
    imu_motion_quality_t quality;
    float early_weight = 0.0f;
    unsigned i;

    config.weight_fall_per_s = 1.0f;
    TEST_ASSERT(imu_motion_quality_init(&quality, &config));
    for (i = 0U; i < 20U; ++i) {
        imu_sample_t sample = sample_at(i);
        sample.accel_m_s2[0] = config.gravity_m_s2 * sinf(0.20f);
        sample.accel_m_s2[2] = -config.gravity_m_s2 * cosf(0.20f);
        TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.20f));
        if (i == 0U) {
            early_weight = quality.accel_weight;
        }
    }
    TEST_ASSERT(quality.accel_norm_residual < config.accel_norm_soft_m_s2);
    TEST_ASSERT(quality.accel_weight + 0.05f < early_weight);
    TEST_ASSERT((quality.mode_reasons & IMU_MOTION_REASON_ACCEL_NORM) == 0U);
    TEST_ASSERT((quality.mode_reasons & IMU_MOTION_REASON_INNOVATION) != 0U);
}

/* Catches unhealthy samples retaining observation authority and recovery that
 * jumps directly from zero to full confidence instead of respecting slew. */
static void test_status_hard_rejection_and_recovery_slew(void)
{
    imu_config_t config = quality_config();
    imu_motion_quality_t quality;
    imu_sample_t sample;

    config.weight_rise_per_s = 0.5f;
    TEST_ASSERT(imu_motion_quality_init(&quality, &config));
    warm_stable(&quality, &config, 10U);
    sample = sample_at(10U);
    sample.accel_status |= IMU_SENSOR_PARITY_ERROR;
    TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.0f));
    TEST_ASSERT_NEAR(quality.accel_weight, 0.0f, 0.0f);
    TEST_ASSERT((quality.mode_reasons & IMU_MOTION_REASON_ACCEL_STATUS) != 0U);
    sample = sample_at(11U);
    TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.0f));
    TEST_ASSERT(quality.accel_weight > 0.0f);
    TEST_ASSERT(quality.accel_weight <= 0.005001f);
}

/* Catches quantizing a valid low-confidence score to zero at the caller's
 * update-rejection threshold instead of reporting the continuous score. */
static void test_valid_weight_remains_continuous_below_rejection_threshold(void)
{
    imu_config_t config = quality_config();
    imu_motion_quality_t quality;
    imu_sample_t sample = sample_at(0U);

    config.accel_weight_min = 0.5f;
    sample.accel_m_s2[2] = -(config.gravity_m_s2 + 1.5f);
    TEST_ASSERT(imu_motion_quality_init(&quality, &config));
    TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.0f));
    TEST_ASSERT(quality.accel_weight > 0.0f);
    TEST_ASSERT(quality.accel_weight < config.accel_weight_min);
}

/* Catches a shared dwell counter or exit comparisons that clear static before
 * the configured number of consecutive dynamic samples. */
static void test_static_exit_uses_independent_dwell(void)
{
    const imu_config_t config = quality_config();
    imu_motion_quality_t quality;
    unsigned i;

    TEST_ASSERT(imu_motion_quality_init(&quality, &config));
    warm_stable(&quality, &config, config.static_enter_samples);
    TEST_ASSERT(quality.static_detected);
    for (i = 0U; i < config.static_exit_samples - 1U; ++i) {
        imu_sample_t sample = sample_at(config.static_enter_samples + i);
        sample.gyro_rad_s[0] = 1.0f;
        TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.0f));
        TEST_ASSERT(quality.static_detected);
    }
    {
        imu_sample_t sample = sample_at(config.static_enter_samples + i);
        sample.gyro_rad_s[0] = 1.0f;
        TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.0f));
    }
    TEST_ASSERT(!quality.static_detected);
    TEST_ASSERT(quality.static_enter_count == 0U);
}

/* Catches accepting an unusable zero-sized history and init paths that erase
 * caller state before discovering an invalid configuration. */
static void test_zero_window_is_rejected_transactionally(void)
{
    imu_config_t config = quality_config();
    imu_motion_quality_t quality;
    imu_motion_quality_t before;

    memset(&quality, 0xA5, sizeof(quality));
    before = quality;
    config.quality_window_samples = 0U;
    TEST_ASSERT(!imu_motion_quality_init(&quality, &config));
    TEST_ASSERT(memcmp(&quality, &before, sizeof(quality)) == 0);
}

/* Catches off-by-one array bounds or counters that wrap after filling the
 * largest statically allocated quality window. */
static void test_maximum_window_and_counter_saturation(void)
{
    imu_config_t config = quality_config();
    imu_motion_quality_t quality;
    imu_sample_t sample;
    unsigned i;

    config.quality_window_samples = IMU_QUALITY_WINDOW_MAX;
    config.static_enter_samples = UINT32_MAX;
    TEST_ASSERT(imu_motion_quality_init(&quality, &config));
    for (i = 0U; i < IMU_QUALITY_WINDOW_MAX + 3U; ++i) {
        sample = sample_at(i);
        TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.0f));
    }
    TEST_ASSERT(quality.window_count == IMU_QUALITY_WINDOW_MAX);
    quality.static_enter_count = UINT32_MAX;
    sample = sample_at(i++);
    TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.0f));
    TEST_ASSERT(quality.static_enter_count == UINT32_MAX);

    quality.static_detected = true;
    quality.static_exit_count = UINT32_MAX;
    sample = sample_at(i);
    sample.gyro_rad_s[0] = 1.0f;
    TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.0f));
    TEST_ASSERT(quality.static_exit_count == UINT32_MAX);
    TEST_ASSERT(!quality.static_detected);
}

/* Catches converting two large absolute timestamps to float before taking
 * their difference, which loses a 10 ms interval entirely. */
static void test_large_timestamps_preserve_dt_resolution(void)
{
    imu_config_t config = quality_config();
    imu_motion_quality_t quality;
    imu_sample_t sample = sample_at(0U);
    float first_weight;

    config.weight_rise_per_s = 0.5f;
    TEST_ASSERT(imu_motion_quality_init(&quality, &config));
    sample.timestamp_ns = UINT64_MAX - UINT64_C(20000000);
    TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.0f));
    first_weight = quality.accel_weight;
    sample.timestamp_ns += TEST_DT_NS;
    TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.0f));
    TEST_ASSERT_NEAR(quality.accel_weight - first_weight, 0.005f, 1.0e-6f);
}

static void assert_update_rejected_without_mutation(
    imu_motion_quality_t *quality, const imu_config_t *config,
    const imu_sample_t *sample, float innovation_angle_rad)
{
    const imu_motion_quality_t before = *quality;

    TEST_ASSERT(!imu_motion_quality_update(quality, config, sample,
                                           innovation_angle_rad));
    TEST_ASSERT(memcmp(quality, &before, sizeof(*quality)) == 0);
}

/* Catches partial ring/counter updates before all external numeric inputs and
 * derived arithmetic have been proven finite. */
static void test_nonfinite_and_time_errors_are_transactional(void)
{
    imu_config_t config = quality_config();
    imu_motion_quality_t quality;
    imu_sample_t sample;

    TEST_ASSERT(imu_motion_quality_init(&quality, &config));
    warm_stable(&quality, &config, 3U);
    sample = sample_at(3U);
    sample.accel_m_s2[1] = NAN;
    assert_update_rejected_without_mutation(&quality, &config, &sample, 0.0f);
    sample = sample_at(3U);
    sample.gyro_rad_s[2] = INFINITY;
    assert_update_rejected_without_mutation(&quality, &config, &sample, 0.0f);
    sample = sample_at(3U);
    assert_update_rejected_without_mutation(&quality, &config, &sample, NAN);
    config = quality_config();
    config.vibration_hard_m_s2 = INFINITY;
    assert_update_rejected_without_mutation(&quality, &config, &sample, 0.0f);
    config = quality_config();
    sample.timestamp_ns = quality.last_timestamp_ns;
    assert_update_rejected_without_mutation(&quality, &config, &sample, 0.0f);
}

/* Catches finite external values whose product overflows during slew-rate
 * calculation and is then silently hidden by clamping. */
static void test_derived_slew_overflow_is_transactional(void)
{
    imu_config_t config = quality_config();
    imu_motion_quality_t quality;
    imu_sample_t sample = sample_at(0U);

    config.weight_rise_per_s = FLT_MAX;
    TEST_ASSERT(imu_motion_quality_init(&quality, &config));
    sample.timestamp_ns = UINT64_C(1);
    sample.accel_status |= IMU_SENSOR_PARITY_ERROR;
    TEST_ASSERT(imu_motion_quality_update(&quality, &config, &sample, 0.0f));
    sample.timestamp_ns = UINT64_MAX;
    sample.accel_status = IMU_SENSOR_DATA_VALID;
    assert_update_rejected_without_mutation(&quality, &config, &sample, 0.0f);
}

int main(void)
{
    test_stable_one_g_reaches_full_quality_and_static_after_dwell();
    test_four_g_impulse_reduces_quality();
    test_alternating_vibration_builds_high_pass_rms();
    test_persistent_lateral_acceleration_decays_plausible_norm_weight();
    test_status_hard_rejection_and_recovery_slew();
    test_valid_weight_remains_continuous_below_rejection_threshold();
    test_static_exit_uses_independent_dwell();
    test_zero_window_is_rejected_transactionally();
    test_maximum_window_and_counter_saturation();
    test_large_timestamps_preserve_dt_resolution();
    test_nonfinite_and_time_errors_are_transactional();
    test_derived_slew_overflow_is_transactional();
    return test_summary();
}
