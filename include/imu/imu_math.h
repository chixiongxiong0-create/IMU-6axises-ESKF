#ifndef IMU_MATH_H
#define IMU_MATH_H

#include "imu_types.h"

float imu_vec3_dot(imu_vec3f_t a, imu_vec3f_t b);
imu_vec3f_t imu_vec3_cross(imu_vec3f_t a, imu_vec3f_t b);
float imu_vec3_norm(imu_vec3f_t v);
bool imu_mat3_multiply(float a[3][3], float b[3][3],
                       float out[3][3]);
void imu_mat6_zero(float out[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM]);
void imu_mat6_identity(float out[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM]);
bool imu_mat6_multiply(float a[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM],
                       float b[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM],
                       float out[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM]);
void imu_mat6_transpose(float in[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM],
                        float out[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM]);
void imu_mat6_symmetrize(float in_out[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM]);
imu_quatf_t imu_quat_multiply(imu_quatf_t a, imu_quatf_t b);
imu_quatf_t imu_quat_conjugate(imu_quatf_t q);
float imu_quat_norm(imu_quatf_t q);
bool imu_quat_normalize(imu_quatf_t *q);
bool imu_quat_rotate(imu_quatf_t q, imu_vec3f_t in, imu_vec3f_t *out);
bool imu_quat_integrate(imu_quatf_t *q_nb, imu_vec3f_t omega_body_rad_s,
                        float dt_s);
bool imu_quat_to_euler_ned(imu_quatf_t q_nb, imu_vec3f_t *roll_pitch_yaw_rad);
float imu_quat_yaw_ned(imu_quatf_t q_nb);

#endif
