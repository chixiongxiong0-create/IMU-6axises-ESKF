#include "test_framework.h"
#include "imu/imu_types.h"
#include "imu/imu_sensor_port.h"

#include <limits.h>
#include <string.h>

/* These compile-time checks protect required wire-independent scalar widths. */
typedef char float_must_be_32_bits[(sizeof(float) * CHAR_BIT == 32U) ? 1 : -1];
typedef char timestamp_must_be_64_bits[
    (sizeof(((imu_sample_t *)0)->timestamp_ns) * CHAR_BIT == 64U) ? 1 : -1];

/* A minimal acquisition producer exercises the public callback boundary. */
static imu_status_t acquire_fixture(void *context, imu_acquisition_t *out)
{
    const uint64_t *timestamp = (const uint64_t *)context;
    imu_acquisition_t frame = {0};
    frame.sample.timestamp_ns = *timestamp;
    frame.sample.gyro_rad_s[0] = 0.5f;
    frame.sample.gyro_sample_fresh[0] = true;
    frame.sample.gyro_status[0] = IMU_SENSOR_DATA_VALID;
    frame.sample.gyro_status[1] = IMU_SENSOR_PARITY_ERROR;
    frame.gyro_latch_timestamp_ns[0] = *timestamp;
    frame.gyro_latch_timestamp_ns[1] = *timestamp + 200U;
    frame.gyro_latch_timestamp_ns[2] = *timestamp + 400U;
    frame.accel_latch_timestamp_ns = *timestamp - 100U;
    *out = frame;
    return IMU_STATUS_OK;
}

int main(int argc, char **argv)
{
    imu_sample_t s = {0};
    imu_acquisition_t acquired = {0};
    uint64_t timestamp = UINT64_C(4297029152);
    imu_sensor_port_t port = {acquire_fixture, &timestamp};
    const uint32_t faults[] = {
        IMU_SENSOR_PARITY_ERROR, IMU_SENSOR_SEQUENCE_ERROR,
        IMU_SENSOR_PLL_ERROR, IMU_SENSOR_POWER_ERROR,
        IMU_SENSOR_SELF_TEST_ERROR, IMU_SENSOR_FIFO_ERROR,
        IMU_SENSOR_OVERRANGE
    };
    uint32_t combined = 0U;
    unsigned i;

    /* Intentional negative run verifies the harness reports failures. */
    if (argc == 2 && strcmp(argv[1], "--check-failure-reporting") == 0) {
        TEST_ASSERT(false);
        TEST_ASSERT_NEAR(NAN, 0.0f, 0.1f);
        return test_summary();
    }

    s.timestamp_ns = UINT64_C(2061856);
    s.gyro_sample_fresh[0] = true;
    TEST_ASSERT(sizeof(float) == 4U);
    TEST_ASSERT(s.timestamp_ns == UINT64_C(2061856));
    TEST_ASSERT(s.gyro_sample_fresh[0]);
    TEST_ASSERT(!s.gyro_sample_fresh[1]);
    TEST_ASSERT(IMU_FRAME_FRD_NED == 1);

    for (i = 0U; i < sizeof(faults) / sizeof(faults[0]); ++i) {
        TEST_ASSERT(faults[i] != 0U && (faults[i] & (faults[i] - 1U)) == 0U);
        TEST_ASSERT((combined & faults[i]) == 0U);
        TEST_ASSERT((IMU_SENSOR_DATA_VALID & faults[i]) == 0U);
        combined |= faults[i];
    }
    TEST_ASSERT(combined == IMU_SENSOR_FAULT_MASK);
    TEST_ASSERT(port.acquire(port.context, &acquired) == IMU_STATUS_OK);
    TEST_ASSERT(acquired.sample.timestamp_ns == UINT64_C(4297029152));
    TEST_ASSERT(acquired.sample.gyro_sample_fresh[0]);
    TEST_ASSERT(!acquired.sample.gyro_sample_fresh[1]);
    TEST_ASSERT_NEAR(acquired.sample.gyro_rad_s[0], 0.5f, 0.0f);
    TEST_ASSERT((acquired.sample.gyro_status[0] & IMU_SENSOR_FAULT_MASK) == 0U);
    TEST_ASSERT((acquired.sample.gyro_status[1] & IMU_SENSOR_FAULT_MASK) != 0U);
    TEST_ASSERT(acquired.gyro_latch_timestamp_ns[2] -
                acquired.gyro_latch_timestamp_ns[0] == 400U);
    TEST_ASSERT(acquired.accel_latch_timestamp_ns == UINT64_C(4297029052));
    return test_summary();
}
