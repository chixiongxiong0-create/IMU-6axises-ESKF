#include "imu/imu_biquad.h"

#include <math.h>
#include <string.h>

#define IMU_TWO_PI_F 6.28318530717958647692f

static bool vector_is_finite(imu_vec3f_t value)
{
    return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

static bool coefficients_are_finite(const imu_biquad_t *filter)
{
    return isfinite(filter->b0) && isfinite(filter->b1) &&
           isfinite(filter->b2) && isfinite(filter->a1) &&
           isfinite(filter->a2);
}

bool imu_biquad_configure_notch(imu_biquad_t *filter, float sample_rate_hz,
                                float frequency_hz, float q)
{
    imu_biquad_t configured = {0};
    float omega;
    float alpha;
    float inverse_a0;

    if (filter == NULL || !isfinite(sample_rate_hz) ||
        !isfinite(frequency_hz) || !isfinite(q) || sample_rate_hz <= 0.0f ||
        frequency_hz <= 0.0f || frequency_hz >= 0.49f * sample_rate_hz ||
        q <= 0.0f) {
        return false;
    }

    omega = IMU_TWO_PI_F * frequency_hz / sample_rate_hz;
    alpha = sinf(omega) / (2.0f * q);
    inverse_a0 = 1.0f / (1.0f + alpha);
    configured.enabled = true;
    configured.b0 = inverse_a0;
    configured.b1 = -2.0f * cosf(omega) * inverse_a0;
    configured.b2 = inverse_a0;
    configured.a1 = configured.b1;
    configured.a2 = (1.0f - alpha) * inverse_a0;
    if (!coefficients_are_finite(&configured)) {
        return false;
    }

    *filter = configured;
    return true;
}

void imu_biquad_reset(imu_biquad_t *filter)
{
    if (filter != NULL) {
        memset(&filter->z1, 0, sizeof(filter->z1));
        memset(&filter->z2, 0, sizeof(filter->z2));
    }
}

static void process_axis(float input, float z1, float z2,
                         const imu_biquad_t *filter, float *output,
                         float *next_z1, float *next_z2)
{
    *output = filter->b0 * input + z1;
    *next_z1 = filter->b1 * input - filter->a1 * *output + z2;
    *next_z2 = filter->b2 * input - filter->a2 * *output;
}

bool imu_biquad_process(imu_biquad_t *filter, imu_vec3f_t input,
                        imu_vec3f_t *out)
{
    imu_vec3f_t output;
    imu_vec3f_t next_z1;
    imu_vec3f_t next_z2;

    if (filter == NULL || out == NULL) {
        return false;
    }
    if (!filter->enabled) {
        memcpy(out, &input, sizeof(*out));
        return true;
    }
    if (!vector_is_finite(input) || !vector_is_finite(filter->z1) ||
        !vector_is_finite(filter->z2) || !coefficients_are_finite(filter)) {
        return false;
    }

    process_axis(input.x, filter->z1.x, filter->z2.x, filter, &output.x,
                 &next_z1.x, &next_z2.x);
    process_axis(input.y, filter->z1.y, filter->z2.y, filter, &output.y,
                 &next_z1.y, &next_z2.y);
    process_axis(input.z, filter->z1.z, filter->z2.z, filter, &output.z,
                 &next_z1.z, &next_z2.z);
    if (!vector_is_finite(output) || !vector_is_finite(next_z1) ||
        !vector_is_finite(next_z2)) {
        return false;
    }

    filter->z1 = next_z1;
    filter->z2 = next_z2;
    *out = output;
    return true;
}
