#ifndef IMU_SENSOR_PORT_H
#define IMU_SENSOR_PORT_H

#include "imu_types.h"

/* Acquisition metadata uses the SAME monotonic nanosecond epoch as sample.
 * sample.timestamp_ns is the common frame/reference latch time, not a delay-
 * corrected physical measurement time. Keep the per-device latch times for
 * sequential reads; a driver must not misrepresent them as simultaneous.
 * Internal MEMS sample phases are not synchronized by simultaneous chip select.
 * Signal-chain group delays belong in imu_config_t and must not be subtracted
 * here as well. The adapter must account for sequential-read skew before
 * passing the common-frame sample to the estimator; this core sample ABI
 * alone cannot convey per-axis latch times. */
typedef struct {
    imu_sample_t sample;
    uint64_t gyro_latch_timestamp_ns[3];
    uint64_t accel_latch_timestamp_ns;
} imu_acquisition_t;

/* The application owns context and output storage; no allocation or ownership
 * transfer occurs. The driver performs SPI decoding, SI-unit conversion, and
 * status/freshness classification. Freshness means a newly acquired sensor
 * measurement, not merely a new SPI command/sequence counter.
 *
 * On OK, populate every field. SENSOR_DEGRADED also supplies a complete frame
 * with individual status/freshness fields identifying unusable data. On all
 * other returns, leave *out unchanged. Never silently repeat a stale value
 * with its fresh flag set. Nonfinite decoded numbers must be marked invalid.
 * Caller serialization, trigger scheduling, and context lifetime are owned by
 * the application; this interface does not implement drivers or interrupts. */
typedef imu_status_t (*imu_sensor_acquire_fn)(void *context,
                                             imu_acquisition_t *out);

typedef struct {
    imu_sensor_acquire_fn acquire;
    void *context;
} imu_sensor_port_t;

#endif
