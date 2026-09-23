#ifndef IMU_TIMING_INTERNAL_H
#define IMU_TIMING_INTERNAL_H

#include "imu/imu_types.h"
#include <math.h>

/* Convert a common float delay to nearest integer ns without an out-of-range
 * float-to-integer cast or integer multiplication/addition overflow. Splitting
 * whole seconds retains fractional precision without production double math. */
static inline bool imu_common_gyro_delay_ns(const imu_config_t *c, uint64_t *ns)
{
    float whole;
    uint64_t seconds, fraction, base;
    const float delay=c->gyro_delay_s[0];
    if(!isfinite(delay) || delay<0 || c->gyro_delay_s[1]!=delay ||
       c->gyro_delay_s[2]!=delay) return false;
    whole=floorf(delay);
    if(whole>=0x1p64f) return false;
    seconds=(uint64_t)whole;
    if(seconds>UINT64_MAX/UINT64_C(1000000000)) return false;
    fraction=(uint64_t)floorf((delay-whole)*1e9f+0.5f);
    base=seconds*UINT64_C(1000000000);
    if(fraction>UINT64_MAX-base) return false;
    *ns=base+fraction;
    return true;
}

static inline void imu_output_epoch(imu_output_t *out, uint64_t delay_ns)
{
    if(out->timestamp_ns>=delay_ns) out->attitude_epoch_ns=out->timestamp_ns-delay_ns;
    else {
        out->attitude_epoch_ns=0;
        out->health_flags|=IMU_HEALTH_TIMING_DEGRADED;
        if(out->timing_status==IMU_STATUS_OK) out->timing_status=IMU_STATUS_BAD_ARGUMENT;
        if(out->status==IMU_STATUS_OK) out->status=IMU_STATUS_SENSOR_DEGRADED;
    }
}

#endif
