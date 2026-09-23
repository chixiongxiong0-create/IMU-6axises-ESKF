#ifndef IMU_SIM_H
#define IMU_SIM_H
#include "imu/imu_types.h"
#include <stddef.h>
#include <stdio.h>
#define IMU_SIM_QUEUE 256
typedef struct {
    double omega[3], acceleration[3], bias[3], temperature;
    uint32_t gyro_status[3], accel_status;
    bool gyro_fresh[3], accel_fresh;
} imu_sim_input_t;
typedef void (*imu_sim_motion_t)(double, imu_sim_input_t *, void *);
typedef struct {
    uint32_t seed;
    double substep_s, gyro_delay_s, accel_delay_s;
    double gyro_sigma, accel_sigma, bias_rw;
    double gyro_matrix[3][3], accel_matrix[3][3];
    double gyro_bias_poly[3][3], accel_bias_poly[3][3];
    double reference_temperature;
    imu_quatf_t initial_q;
    imu_sim_motion_t motion;
    void *context;
} imu_sim_config_t;
typedef struct { double t, q[4]; imu_sim_input_t input; } imu_sim_point_t;
typedef struct {
    imu_sim_config_t config;
    uint32_t rng;
    uint64_t frame;
    unsigned head, count;
    double t, q[4], walk[3];
    imu_sim_point_t queue[IMU_SIM_QUEUE];
} imu_sim_t;
void imu_sim_defaults(imu_sim_config_t *c);
/* Returns false for invalid parameters or delay history exceeding the fixed
 * queue, including shortened integration steps at 485 Hz frame boundaries. */
bool imu_sim_init(imu_sim_t *s, const imu_sim_config_t *c);
bool imu_sim_next(imu_sim_t *s, imu_sample_t *sample, imu_quatf_t *truth,
                  imu_vec3f_t *bias);
typedef struct {
    const char *name;
    double roll_rms_limit, pitch_rms_limit, peak_limit, discontinuity_limit;
    double recovery_limit, yaw_limit, bias_limit, consistency_limit;
    double disturbance_end_s;
} imu_scenario_t;
typedef struct { double start_s,last_bad_s; } imu_recovery_t;
void imu_recovery_init(imu_recovery_t *r,double disturbance_end_s);
void imu_recovery_observe(imu_recovery_t *r,double time_s,double tilt_error_rad);
double imu_recovery_result(const imu_recovery_t *r,double final_time_s);
typedef struct {
    double roll_rms, pitch_rms, yaw_rms, peak, recovery, bias_error;
    double nis_mean, nis_max, consistency_max, cpu_s;
    unsigned failures, discontinuities, unreported_degraded, samples, nis_samples;
    size_t memory_bytes;
} imu_metrics_t;
extern const imu_scenario_t imu_scenarios[];
extern const unsigned imu_scenario_count;
bool imu_run_scenario(unsigned scenario,uint32_t seed,imu_metrics_t metrics[2]);
bool imu_run_scenario_trace(unsigned scenario,uint32_t seed,imu_metrics_t metrics[2],FILE *trace);
bool imu_compare_accept(const imu_metrics_t *e,const imu_metrics_t *c);
#endif
