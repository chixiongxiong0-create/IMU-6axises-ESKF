#include "test_framework.h"
#include "imu/imu.h"
#include "imu/imu_math.h"
#include <string.h>

static imu_calibration_t calibration(void)
{
    imu_calibration_t c = {0}; unsigned i;
    c.version = IMU_CALIBRATION_VERSION;
    c.valid = c.checksum_valid = true;
    c.temperature_min_c = -40; c.temperature_max_c = 85;
    for (i=0; i<3; ++i) {
        c.gyro_scale_alignment[i][i] = 1;
        c.accel_scale_alignment[i][i] = 1;
        c.body_rotation[i][i] = 1;
    }
    return c;
}

static imu_sample_t sample_at(unsigned i)
{
    imu_sample_t s = {0}; unsigned j;
    s.timestamp_ns = UINT64_C(1000000000) + (uint64_t)i*UINT64_C(1000000000)/485;
    s.accel_m_s2[2] = -9.80665f;
    s.accel_status = IMU_SENSOR_DATA_VALID; s.accel_sample_fresh = true;
    for (j=0; j<3; ++j) {
        s.gyro_status[j] = IMU_SENSOR_DATA_VALID; s.gyro_sample_fresh[j] = true;
    }
    return s;
}

/* Catches premature startup, omitted averaging, wrong specific-force sign. */
static void initialization(void)
{
    imu_t imu = {0}, before; imu_output_t out = {0};
    imu_config_t c = {0}; imu_calibration_t cal = calibration();
    imu_sample_t s; unsigned i;
    imu_config_defaults(&c); c.alignment_samples = 4; c.static_enter_samples = 2;
    c.initial_yaw_rad=0.4f;
    TEST_ASSERT(imu_init(&imu, &c, &cal));
    TEST_ASSERT(imu_get_output(&imu, &out) == IMU_STATUS_NOT_INITIALIZED);
    TEST_ASSERT(!out.initialized);
    for (i=0; i<12; ++i) {
        s = sample_at(i); s.accel_status |= IMU_SENSOR_SELF_TEST_ERROR;
        (void)imu_process(&imu, &s, &out);
        TEST_ASSERT(!out.initialized);
    }
    for (i=12; i<24; ++i) {
        s = sample_at(i); s.gyro_rad_s[0] = 0.01f;
        s.accel_m_s2[1] = -4.903325f; s.accel_m_s2[2] = -8.492808f;
        (void)imu_process(&imu, &s, &out);
        if (i==12) TEST_ASSERT(!out.initialized);
    }
    TEST_ASSERT(out.initialized);
    TEST_ASSERT_NEAR(out.roll_rad, 0.5235988f, 0.001f);
    TEST_ASSERT_NEAR(out.yaw_rad,0.4f,0.001f);
    TEST_ASSERT_NEAR(out.gyro_bias_rad_s.x, 0.01f, 0.0001f);
    TEST_ASSERT(imu_reset(&imu));
    TEST_ASSERT(imu_get_output(&imu, &out) == IMU_STATUS_NOT_INITIALIZED);
    before=imu; cal.valid=false;
    TEST_ASSERT(!imu_init(&imu,&c,&cal));
    TEST_ASSERT(memcmp(&imu,&before,sizeof imu)==0);
    cal=calibration(); c.init_mode=IMU_INIT_QUATERNION;
    c.initial_q_nb=(imu_quatf_t){2,0,0,0};
    TEST_ASSERT(imu_init(&imu,&c,&cal));
    TEST_ASSERT(imu_get_output(&imu,&out)==IMU_STATUS_NOT_INITIALIZED);
    s=sample_at(0); TEST_ASSERT(imu_process(&imu,&s,&out)==IMU_STATUS_OK);
    TEST_ASSERT(out.initialized); TEST_ASSERT_NEAR(out.q_nb.w,1,1e-6f);
    before=imu; c.initial_yaw_rad=NAN;
    TEST_ASSERT(!imu_init(&imu,&c,&cal));
    TEST_ASSERT(memcmp(&imu,&before,sizeof imu)==0);
}

static void start(imu_t *imu, imu_config_t *c, imu_output_t *out)
{
    imu_calibration_t cal=calibration(); imu_sample_t s=sample_at(0);
    imu_config_defaults(c); c->init_mode=IMU_INIT_QUATERNION;
    TEST_ASSERT(imu_init(imu,c,&cal));
    TEST_ASSERT(imu_process(imu,&s,out)==IMU_STATUS_OK);
}

/* Catches nominal-dt substitution, stale-axis integration, and double-counted
 * recovery intervals. A missing interval is reported and covariance grows. */
static void timing_and_freshness(void)
{
    imu_t imu={0},before; imu_config_t c; imu_output_t out={0},sentinel;
    imu_sample_t s; unsigned i; float variance,yaw;
    start(&imu,&c,&out);
    for(i=1;i<=485;++i) {
        s=sample_at(i); s.gyro_rad_s[2]=1; s.accel_sample_fresh=false;
        TEST_ASSERT(imu_process(&imu,&s,&out)==IMU_STATUS_SENSOR_DEGRADED);
    }
    TEST_ASSERT_NEAR(out.yaw_rad,1,0.00002f);
    TEST_ASSERT(out.health_flags&IMU_HEALTH_ACCEL_DEGRADED);
    before=imu; sentinel=out;
    s.timestamp_ns--; TEST_ASSERT(imu_process(&imu,&s,&out)==IMU_STATUS_TIME_REVERSED);
    TEST_ASSERT(memcmp(&before,&imu,sizeof imu)==0);
    TEST_ASSERT(memcmp(&sentinel,&out,sizeof out)==0);
    s.timestamp_ns=imu.last_timestamp_ns+100;
    TEST_ASSERT(imu_process(&imu,&s,&out)==IMU_STATUS_BAD_ARGUMENT);
    TEST_ASSERT(memcmp(&before,&imu,sizeof imu)==0);
    s.timestamp_ns=imu.last_timestamp_ns+UINT64_C(3000000);
    (void)imu_process(&imu,&s,&out); TEST_ASSERT_NEAR(out.yaw_rad,1.003f,0.00002f);
    for(i=0;i<3;++i) {
        variance=imu.esekf.P[2][2]; yaw=out.yaw_rad;
        s.timestamp_ns+=UINT64_C(2000000); s.gyro_sample_fresh[i]=false;
        TEST_ASSERT(imu_process(&imu,&s,&out)==IMU_STATUS_SENSOR_DEGRADED);
        TEST_ASSERT_NEAR(out.yaw_rad,yaw,1e-6f);
        TEST_ASSERT(imu.esekf.P[2][2]>variance);
        TEST_ASSERT(out.health_flags&IMU_HEALTH_GYRO_DEGRADED);
        s.gyro_sample_fresh[i]=true; s.timestamp_ns+=UINT64_C(3000000);
        (void)imu_process(&imu,&s,&out); TEST_ASSERT_NEAR(out.yaw_rad,yaw+0.003f,0.00002f);
    }
    yaw=out.yaw_rad; s.timestamp_ns+=UINT64_C(100000000);
    TEST_ASSERT(imu_process(&imu,&s,&out)==IMU_STATUS_TIME_GAP);
    TEST_ASSERT_NEAR(out.yaw_rad,yaw,1e-6f);
    TEST_ASSERT(out.timing_status==IMU_STATUS_TIME_GAP);
    s.timestamp_ns+=UINT64_C(2000000); (void)imu_process(&imu,&s,&out);
    TEST_ASSERT_NEAR(out.yaw_rad,yaw+0.002f,0.00002f);
    before=imu; sentinel=out; s.timestamp_ns+=UINT64_C(2000000); s.accel_m_s2[0]=NAN;
    TEST_ASSERT(imu_process(&imu,&s,&out)==IMU_STATUS_NONFINITE);
    TEST_ASSERT(memcmp(&before,&imu,sizeof imu)==0);
    TEST_ASSERT(memcmp(&sentinel,&out,sizeof out)==0);
}

/* Catches calibration coupling that discards a valid gyro propagation when
 * accelerometer temperature/norm is rejected, and continuing a corrupt ESKF. */
static void observation_failure_and_fault(void)
{
    imu_t imu={0}; imu_config_t c; imu_output_t out={0}; imu_sample_t s;
    start(&imu,&c,&out); s=sample_at(1); s.gyro_rad_s[2]=1;
    s.accel_temperature_c=100;
    TEST_ASSERT(imu_process(&imu,&s,&out)==IMU_STATUS_SENSOR_DEGRADED);
    TEST_ASSERT_NEAR(out.yaw_rad,0.002061856f,1e-6f);
    s=sample_at(2); s.gyro_rad_s[2]=1; s.accel_m_s2[2]=0;
    (void)imu_process(&imu,&s,&out);
    TEST_ASSERT_NEAR(out.yaw_rad,0.004123711f,1e-6f);
    imu.esekf.P[0][0]=-1; s=sample_at(3);
    TEST_ASSERT(imu_process(&imu,&s,&out)==IMU_STATUS_NUMERICAL_FAULT);
    TEST_ASSERT(imu.state==IMU_STATE_FAULT); TEST_ASSERT(!out.initialized);
    TEST_ASSERT(out.health_flags&IMU_HEALTH_NUMERICAL_FAULT);
    s=sample_at(4); TEST_ASSERT(imu_process(&imu,&s,&out)==IMU_STATUS_NUMERICAL_FAULT);
    TEST_ASSERT(imu_reset(&imu));
    TEST_ASSERT(imu_process(&imu,&s,&out)==IMU_STATUS_OK);
}

/* Catches missing gravity fusion, fictitious heading information, missing
 * bias correction, and hysteresis permitting zero-rate updates during motion. */
static void output_and_yaw(void)
{
    imu_t imu={0}; imu_config_t c; imu_output_t out={0};
    imu_calibration_t cal=calibration(); imu_sample_t s; unsigned i;
    float initial_yaw_variance, bias;
    imu_config_defaults(&c); c.init_mode=IMU_INIT_QUATERNION;
    c.initial_q_nb=(imu_quatf_t){0.92387953f,0,0,0.38268343f};
    c.static_enter_samples=3; c.weight_rise_per_s=100;
    TEST_ASSERT(imu_init(&imu,&c,&cal));
    s=sample_at(0); (void)imu_process(&imu,&s,&out);
    initial_yaw_variance=imu.esekf.P[2][2];
    for(i=1;i<=485;++i) {
        s=sample_at(i); s.gyro_rad_s[2]=0.2f;
        TEST_ASSERT(imu_process(&imu,&s,&out)==IMU_STATUS_OK);
    }
    TEST_ASSERT(out.fusion_mode==IMU_FUSION_GRAVITY);
    TEST_ASSERT_NEAR(out.yaw_rad,0.98539816f,0.0001f);
    TEST_ASSERT(imu.esekf.P[2][2]>initial_yaw_variance);
    TEST_ASSERT(out.health_flags&IMU_HEALTH_HEADING_UNOBSERVABLE);
    TEST_ASSERT_NEAR(imu_quat_norm(out.q_nb),1,1e-6f);
    TEST_ASSERT(isfinite(out.attitude_std_rad.x) && out.attitude_std_rad.x>=0);
    TEST_ASSERT(isfinite(out.attitude_std_rad.y) && out.attitude_std_rad.y>=0);
    TEST_ASSERT(isfinite(out.attitude_std_rad.z) && out.attitude_std_rad.z>=0);
    TEST_ASSERT(out.accel_weight>=0 && out.accel_weight<=1);
    TEST_ASSERT_NEAR(out.angular_rate_rad_s.z,0.2f,1e-6f);
    for(i=486;i<600;++i) {
        s=sample_at(i); s.gyro_rad_s[0]=0.01f;
        (void)imu_process(&imu,&s,&out);
    }
    TEST_ASSERT(out.fusion_mode==IMU_FUSION_STATIC);
    TEST_ASSERT(out.gyro_bias_rad_s.x>0.005f);
    TEST_ASSERT_NEAR(out.angular_rate_rad_s.x,0.01f-out.gyro_bias_rad_s.x,1e-7f);
    bias=out.gyro_bias_rad_s.x;
    s=sample_at(600); s.gyro_rad_s[0]=1; s.accel_sample_fresh=false;
    (void)imu_process(&imu,&s,&out);
    TEST_ASSERT(out.fusion_mode==IMU_FUSION_GYRO_ONLY);
    TEST_ASSERT_NEAR(out.gyro_bias_rad_s.x,bias,1e-7f);

    /* Positive roll lowers the right wing in FRD/NED. Known quaternion
     * (roll=30 deg, pitch=-20 deg, yaw=40 deg), normalized independently. */
    c.initial_q_nb=(imu_quatf_t){0.8785122f,0.2968829f,-0.07043934f,0.3675801f};
    TEST_ASSERT(imu_init(&imu,&c,&cal)); s=sample_at(0);
    (void)imu_process(&imu,&s,&out);
    TEST_ASSERT_NEAR(out.roll_rad,0.5235988f,1e-5f);
    TEST_ASSERT_NEAR(out.pitch_rad,-0.34906585f,1e-5f);
    TEST_ASSERT_NEAR(out.yaw_rad,0.6981317f,1e-5f);
    c.initial_q_nb=(imu_quatf_t){0.99875026f,0.04997917f,0,0};
    TEST_ASSERT(imu_init(&imu,&c,&cal));
    for(i=0;i<100;++i) { s=sample_at(i); (void)imu_process(&imu,&s,&out); }
    TEST_ASSERT(fabsf(out.roll_rad)<0.01f);
}

static void magnetometer_noop(void)
{
    imu_t imu={0},before; imu_config_t c; imu_output_t out={0};
    imu_mag_sample_t mag={0}; unsigned i;
    start(&imu,&c,&out); mag.field_ut=(imu_vec3f_t){20,1,40};
    mag.status=IMU_SENSOR_DATA_VALID; mag.sample_fresh=true;
    before=imu;
    for(i=0;i<3;++i) {
        mag.timestamp_ns=i;
        TEST_ASSERT(imu_update_magnetometer(&imu,&mag)==IMU_STATUS_AIDING_UNAVAILABLE);
        TEST_ASSERT(memcmp(&before,&imu,sizeof imu)==0);
    }
    mag.field_ut.x=NAN;
    TEST_ASSERT(imu_update_magnetometer(&imu,&mag)==IMU_STATUS_NONFINITE);
    TEST_ASSERT(memcmp(&before,&imu,sizeof imu)==0);
    TEST_ASSERT(imu_update_magnetometer(&imu,NULL)==IMU_STATUS_BAD_ARGUMENT);
    TEST_ASSERT(imu_update_magnetometer(NULL,&mag)==IMU_STATUS_BAD_ARGUMENT);
}

/* Catches accepting invalid live configuration, treating nonfinite calibration
 * as a diagnostic fallback, and failing to report temperature extrapolation. */
static void validation_edges(void)
{
    imu_t imu={0},before; imu_config_t c; imu_output_t out={0};
    imu_calibration_t cal=calibration(); imu_sample_t s;
    start(&imu,&c,&out); before=imu;
    c.max_bias_step_rad_s=0; TEST_ASSERT(!imu_init(&imu,&c,&cal));
    TEST_ASSERT(memcmp(&before,&imu,sizeof imu)==0);
    imu_config_defaults(&c); c.gravity_nis_limit=0;
    TEST_ASSERT(!imu_init(&imu,&c,&cal));
    imu_config_defaults(&c); c.allow_uncalibrated=true; cal.gyro_bias_poly[1][2]=NAN;
    TEST_ASSERT(!imu_init(&imu,&c,&cal));
    cal=calibration(); cal.valid=false; c.init_mode=IMU_INIT_QUATERNION;
    TEST_ASSERT(imu_init(&imu,&c,&cal)); s=sample_at(0);
    (void)imu_process(&imu,&s,&out);
    TEST_ASSERT(out.health_flags&IMU_HEALTH_UNCALIBRATED);
    TEST_ASSERT(!out.calibration_valid);
    cal=calibration(); cal.allow_temperature_extrapolation=true;
    TEST_ASSERT(imu_init(&imu,&c,&cal)); s.gyro_status[1]|=IMU_SENSOR_PARITY_ERROR;
    (void)imu_process(&imu,&s,&out); TEST_ASSERT(!out.initialized);
    s=sample_at(1); s.accel_temperature_c=100;
    (void)imu_process(&imu,&s,&out);
    TEST_ASSERT(out.health_flags&IMU_HEALTH_TEMPERATURE_EXTRAPOLATED);
}

/* Exact constant roll rate: accel is measured 10 ms before the gyro body
 * epoch, so rotating its body vector by -omega*delay must remove the lag. */
static void delay_compensation(void)
{
    imu_t imu={0},equal={0},before; imu_config_t c,z;
    imu_calibration_t cal=calibration(); imu_output_t out={0},same={0};
    imu_sample_t s; unsigned i; float angle;
    imu_config_defaults(&c); c.init_mode=IMU_INIT_QUATERNION;
    c.accel_delay_s=0.01f; c.gyro_noise_density=0.0001f;
    c.initial_covariance_diag[0]=0.1f; c.weight_rise_per_s=1000;
    TEST_ASSERT(imu_init(&imu,&c,&cal));
    for(i=0;i<=100;++i) {
        s=sample_at(i); angle=(float)i/485.0f-0.01f;
        s.gyro_rad_s[0]=1;
        s.accel_m_s2[1]=-9.80665f*sinf(angle);
        s.accel_m_s2[2]=-9.80665f*cosf(angle);
        (void)imu_process(&imu,&s,&out);
    }
    TEST_ASSERT_NEAR(out.roll_rad,0.206185567f,0.0002f);
    before=imu; c.accel_delay_s=0.02f;
    TEST_ASSERT(!imu_init(&imu,&c,&cal));
    TEST_ASSERT(memcmp(&before,&imu,sizeof imu)==0);
    imu_config_defaults(&z); z.init_mode=IMU_INIT_QUATERNION; c=z;
    c.accel_delay_s=0.005f;
    for(i=0;i<3;++i) c.gyro_delay_s[i]=0.005f;
    TEST_ASSERT(imu_init(&imu,&z,&cal)); TEST_ASSERT(imu_init(&equal,&c,&cal));
    for(i=0;i<20;++i) {
        s=sample_at(i); s.gyro_rad_s[2]=0.2f;
        (void)imu_process(&imu,&s,&out); (void)imu_process(&equal,&s,&same);
    }
    TEST_ASSERT(memcmp(&out,&same,sizeof out)==0);
}

int main(void)
{
    initialization();
    timing_and_freshness();
    observation_failure_and_fault();
    output_and_yaw();
    magnetometer_noop();
    validation_edges();
    delay_compensation();
    return test_summary();
}
