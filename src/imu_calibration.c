#include "imu/imu_calibration.h"

#include <math.h>

#define IMU_ROTATION_TOLERANCE 1.0e-3f

static bool matrix_is_finite(const float matrix[3][3])
{
    unsigned row;
    unsigned column;

    for (row = 0U; row < 3U; ++row) {
        for (column = 0U; column < 3U; ++column) {
            if (!isfinite(matrix[row][column])) {
                return false;
            }
        }
    }
    return true;
}

static bool polynomial_table_is_finite(
    const float coefficients[3][IMU_TEMP_POLY_COEFFICIENTS])
{
    unsigned axis;
    unsigned coefficient;

    for (axis = 0U; axis < 3U; ++axis) {
        for (coefficient = 0U;
             coefficient < IMU_TEMP_POLY_COEFFICIENTS;
             ++coefficient) {
            if (!isfinite(coefficients[axis][coefficient])) {
                return false;
            }
        }
    }
    return true;
}

static bool rotation_is_valid(const float rotation[3][3])
{
    float determinant;
    unsigned row;
    unsigned other_row;
    unsigned column;

    if (!matrix_is_finite(rotation)) {
        return false;
    }
    for (row = 0U; row < 3U; ++row) {
        for (other_row = row; other_row < 3U; ++other_row) {
            float dot = 0.0f;
            const float expected = row == other_row ? 1.0f : 0.0f;

            for (column = 0U; column < 3U; ++column) {
                dot += rotation[row][column] * rotation[other_row][column];
            }
            if (!isfinite(dot) || fabsf(dot - expected) > IMU_ROTATION_TOLERANCE) {
                return false;
            }
        }
    }
    determinant =
        rotation[0][0] * (rotation[1][1] * rotation[2][2] -
                          rotation[1][2] * rotation[2][1]) -
        rotation[0][1] * (rotation[1][0] * rotation[2][2] -
                          rotation[1][2] * rotation[2][0]) +
        rotation[0][2] * (rotation[1][0] * rotation[2][1] -
                          rotation[1][1] * rotation[2][0]);
    return isfinite(determinant) &&
           fabsf(determinant - 1.0f) <= IMU_ROTATION_TOLERANCE;
}

static bool calibration_is_valid(const imu_calibration_t *calibration)
{
    if (calibration == NULL ||
        calibration->version != IMU_CALIBRATION_VERSION ||
        !calibration->valid || !calibration->checksum_valid ||
        !isfinite(calibration->temperature_min_c) ||
        !isfinite(calibration->temperature_max_c) ||
        !isfinite(calibration->temperature_reference_c) ||
        calibration->temperature_min_c > calibration->temperature_max_c ||
        calibration->temperature_reference_c < calibration->temperature_min_c ||
        calibration->temperature_reference_c > calibration->temperature_max_c ||
        !matrix_is_finite(calibration->gyro_scale_alignment) ||
        !matrix_is_finite(calibration->accel_scale_alignment) ||
        !rotation_is_valid(calibration->body_rotation) ||
        !polynomial_table_is_finite(calibration->gyro_bias_poly) ||
        !polynomial_table_is_finite(calibration->accel_bias_poly)) {
        return false;
    }
    return true;
}

static bool temperature_is_allowed(const imu_calibration_t *calibration,
                                   float temperature_c)
{
    return isfinite(temperature_c) &&
           (calibration->allow_temperature_extrapolation ||
            (temperature_c >= calibration->temperature_min_c &&
             temperature_c <= calibration->temperature_max_c));
}

static bool vector_is_finite(const float vector[3])
{
    return isfinite(vector[0]) && isfinite(vector[1]) && isfinite(vector[2]);
}

static bool multiply_matrix_vector(const float matrix[3][3],
                                   const float input[3], float output[3])
{
    float result[3];
    unsigned row;

    for (row = 0U; row < 3U; ++row) {
        result[row] = matrix[row][0] * input[0] +
                      matrix[row][1] * input[1] +
                      matrix[row][2] * input[2];
        if (!isfinite(result[row])) {
            return false;
        }
    }
    output[0] = result[0];
    output[1] = result[1];
    output[2] = result[2];
    return true;
}

static bool evaluate_bias(
    const float coefficients[IMU_TEMP_POLY_COEFFICIENTS],
    float temperature_delta_c, float *bias)
{
    float result;

    result = coefficients[2] * temperature_delta_c + coefficients[1];
    if (!isfinite(result)) {
        return false;
    }
    result = result * temperature_delta_c + coefficients[0];
    if (!isfinite(result)) {
        return false;
    }
    *bias = result;
    return true;
}

bool imu_calibrate_sample(const imu_calibration_t *calibration,
                          const imu_sample_t *input,
                          imu_calibrated_sample_t *out)
{
    imu_calibrated_sample_t result;
    float gyro_aligned[3];
    float accel_aligned[3];
    float gyro_corrected[3];
    float accel_corrected[3];
    float gyro_body[3];
    float accel_body[3];
    unsigned axis;

    if (input == NULL || out == NULL || !calibration_is_valid(calibration) ||
        !vector_is_finite(input->gyro_rad_s) ||
        !vector_is_finite(input->accel_m_s2) ||
        !temperature_is_allowed(calibration, input->gyro_temperature_c[0]) ||
        !temperature_is_allowed(calibration, input->gyro_temperature_c[1]) ||
        !temperature_is_allowed(calibration, input->gyro_temperature_c[2]) ||
        !temperature_is_allowed(calibration, input->accel_temperature_c) ||
        !multiply_matrix_vector(calibration->gyro_scale_alignment,
                                input->gyro_rad_s, gyro_aligned) ||
        !multiply_matrix_vector(calibration->accel_scale_alignment,
                                input->accel_m_s2, accel_aligned)) {
        return false;
    }
    for (axis = 0U; axis < 3U; ++axis) {
        float bias;
        const float temperature_delta_c =
            input->gyro_temperature_c[axis] -
            calibration->temperature_reference_c;

        if (!isfinite(temperature_delta_c) ||
            !evaluate_bias(calibration->gyro_bias_poly[axis],
                           temperature_delta_c, &bias)) {
            return false;
        }
        gyro_corrected[axis] = gyro_aligned[axis] - bias;
        if (!isfinite(gyro_corrected[axis])) {
            return false;
        }
    }
    for (axis = 0U; axis < 3U; ++axis) {
        float bias;
        const float temperature_delta_c = input->accel_temperature_c -
                                          calibration->temperature_reference_c;

        if (!isfinite(temperature_delta_c) ||
            !evaluate_bias(calibration->accel_bias_poly[axis],
                           temperature_delta_c, &bias)) {
            return false;
        }
        accel_corrected[axis] = accel_aligned[axis] - bias;
        if (!isfinite(accel_corrected[axis])) {
            return false;
        }
    }
    if (!multiply_matrix_vector(calibration->body_rotation,
                                gyro_corrected, gyro_body) ||
        !multiply_matrix_vector(calibration->body_rotation,
                                accel_corrected, accel_body)) {
        return false;
    }

    result.timestamp_ns = input->timestamp_ns;
    result.gyro_rad_s = (imu_vec3f_t){gyro_body[0], gyro_body[1], gyro_body[2]};
    result.accel_m_s2 = (imu_vec3f_t){accel_body[0], accel_body[1], accel_body[2]};
    for (axis = 0U; axis < 3U; ++axis) {
        result.gyro_temperature_c[axis] = input->gyro_temperature_c[axis];
    }
    result.accel_temperature_c = input->accel_temperature_c;
    *out = result;
    return true;
}
