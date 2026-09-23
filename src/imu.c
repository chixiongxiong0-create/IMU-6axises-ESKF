#include "imu/imu.h"
#include "imu/imu_math.h"
#include "imu_timing.h"
#include <math.h>
#include <string.h>

void imu_config_defaults(imu_config_t *c)
{
    unsigned i;
    if (!c) return;
    memset(c,0,sizeof *c);
    c->gravity_m_s2=9.80665f; c->nominal_dt_s=1.0f/485.0f;
    c->min_dt_s=0.0005f; c->max_dt_s=0.02f;
    c->max_delay_compensation_s=0.01f;
    c->gyro_noise_density=0.0003f; c->gyro_bias_rw=0.00001f;
    c->accel_direction_noise_variance=0.001f; c->zero_rate_noise_variance=0.00001f;
    c->gravity_nis_limit=11.345f; c->max_bias_step_rad_s=0.005f;
    c->quality_window_samples=8;
    c->accel_norm_soft_m_s2=0.2f; c->accel_norm_hard_m_s2=2;
    c->vibration_soft_m_s2=0.1f; c->vibration_hard_m_s2=0.8f;
    c->jerk_soft_m_s3=5; c->jerk_hard_m_s3=80;
    c->innovation_soft_rad=0.08f; c->innovation_hard_rad=0.3f;
    c->innovation_persistence_s=0.05f; c->accel_weight_min=0.01f;
    c->weight_rise_per_s=10; c->weight_fall_per_s=100;
    c->static_gyro_max_rad_s=0.02f; c->static_gyro_variance_max=0.0001f;
    c->static_accel_norm_max_m_s2=0.1f; c->static_accel_variance_max=0.01f;
    c->static_jerk_max_m_s3=2; c->static_vibration_max_m_s2=0.05f;
    c->static_enter_samples=32; c->static_exit_samples=3;
    c->zero_rate_update_interval_s=0.05f;
    for(i=0;i<6;++i) c->initial_covariance_diag[i]=i<3?0.01f:0.0001f;
    c->initial_q_nb.w=1; c->alignment_samples=128;
    c->complementary_kp=1; c->complementary_ki=0.01f;
}

static bool config_valid(const imu_config_t *c)
{
    unsigned i;
    uint64_t delay_ns;
    if(!imu_common_gyro_delay_ns(c,&delay_ns)) return false;
#define NONNEG(field) if (!isfinite(c->field) || c->field<0) return false
    NONNEG(min_dt_s); NONNEG(max_dt_s); NONNEG(max_delay_compensation_s);
    NONNEG(gyro_noise_density); NONNEG(gyro_bias_rw);
    NONNEG(accel_direction_noise_variance); NONNEG(zero_rate_noise_variance);
    NONNEG(gravity_nis_limit); NONNEG(max_bias_step_rad_s);
    NONNEG(zero_rate_update_interval_s); NONNEG(accel_delay_s);
    NONNEG(complementary_kp); NONNEG(complementary_ki);
#undef NONNEG
    if(c->min_dt_s<=0 || c->min_dt_s>c->nominal_dt_s ||
       c->nominal_dt_s>c->max_dt_s || c->accel_direction_noise_variance<=0 ||
       c->max_bias_step_rad_s<=0 || c->gravity_nis_limit<=0 ||
       c->zero_rate_noise_variance<=0 || c->zero_rate_update_interval_s<=0 ||
       c->alignment_samples<2 || !isfinite(c->initial_yaw_rad) ||
       !isfinite(imu_quat_norm(c->initial_q_nb)) ||
       c->gyro_notch_count>IMU_NOTCH_MAX || c->accel_notch_count>IMU_NOTCH_MAX ||
       (c->init_mode!=IMU_INIT_STATIONARY && c->init_mode!=IMU_INIT_QUATERNION)) return false;
    for(i=0;i<3;++i)
        if(!isfinite(c->gyro_delay_s[i]) || c->gyro_delay_s[i]<0 ||
           fabsf(c->gyro_delay_s[i]-c->accel_delay_s)>c->max_delay_compensation_s) return false;
    for(i=0;i<IMU_NOTCH_MAX;++i)
        if(!isfinite(c->gyro_notches[i].frequency_hz) || !isfinite(c->gyro_notches[i].q) ||
           !isfinite(c->accel_notches[i].frequency_hz) || !isfinite(c->accel_notches[i].q)) return false;
    return true;
}

static bool calibration_finite(const imu_calibration_t *c)
{
    unsigned i,j;
    if(!isfinite(c->temperature_min_c) || !isfinite(c->temperature_max_c) ||
       !isfinite(c->temperature_reference_c)) return false;
    for(i=0;i<3;++i) for(j=0;j<3;++j)
        if(!isfinite(c->gyro_scale_alignment[i][j]) ||
           !isfinite(c->accel_scale_alignment[i][j]) ||
           !isfinite(c->body_rotation[i][j]) || !isfinite(c->gyro_bias_poly[i][j]) ||
           !isfinite(c->accel_bias_poly[i][j])) return false;
    return true;
}

bool imu_init(imu_t *imu, const imu_config_t *c, const imu_calibration_t *cal)
{
    imu_t n; imu_sample_t probe={0}; imu_calibrated_sample_t checked; unsigned i;
    if(!imu || !c || !config_valid(c)) return false;
    memset(&n,0,sizeof n); n.config=*c;
    if(!imu_motion_quality_init(&n.quality,c)) return false;
    if(cal) {
        if(!calibration_finite(cal)) return false;
        probe.accel_temperature_c=cal->temperature_reference_c;
        for(i=0;i<3;++i) probe.gyro_temperature_c[i]=cal->temperature_reference_c;
        n.calibration_valid=imu_calibrate_sample(cal,&probe,&checked);
    }
    if(!n.calibration_valid && !c->allow_uncalibrated) return false;
    if(n.calibration_valid) n.calibration=*cal;
    else {
        n.calibration.version=IMU_CALIBRATION_VERSION;
        n.calibration.valid=n.calibration.checksum_valid=true;
        n.calibration.temperature_min_c=-273.15f; n.calibration.temperature_max_c=1000;
        n.calibration.allow_temperature_extrapolation=true;
        for(i=0;i<3;++i) {
            n.calibration.gyro_scale_alignment[i][i]=1;
            n.calibration.accel_scale_alignment[i][i]=1;
            n.calibration.body_rotation[i][i]=1;
        }
    }
    for(i=0;i<c->gyro_notch_count;++i)
        if(c->gyro_notches[i].enabled && !imu_biquad_configure_notch(&n.gyro_notches[i],
            1/c->nominal_dt_s,c->gyro_notches[i].frequency_hz,c->gyro_notches[i].q)) return false;
    for(i=0;i<c->accel_notch_count;++i)
        if(c->accel_notches[i].enabled && !imu_biquad_configure_notch(&n.accel_notches[i],
            1/c->nominal_dt_s,c->accel_notches[i].frequency_hz,c->accel_notches[i].q)) return false;
    if(imu_esekf_init(&n.esekf,c,c->initial_q_nb,(imu_vec3f_t){0,0,0})!=IMU_STATUS_OK) return false;
    n.configured=true; n.state=IMU_STATE_ALIGNING;
    n.output.status=IMU_STATUS_NOT_INITIALIZED; n.output.fusion_mode=IMU_FUSION_ALIGNING;
    n.output.calibration_valid=n.calibration_valid;
    n.output.health_flags=IMU_HEALTH_HEADING_UNOBSERVABLE |
        (n.calibration_valid?0:IMU_HEALTH_UNCALIBRATED);
    *imu=n; return true;
}

static bool healthy(uint32_t status, bool fresh)
{ return fresh && (status&IMU_SENSOR_DATA_VALID)!=0 && (status&IMU_SENSOR_FAULT_MASK)==0; }

static imu_quatf_t from_euler(float roll,float pitch,float yaw)
{
    const float cr=cosf(roll/2),sr=sinf(roll/2),cp=cosf(pitch/2),sp=sinf(pitch/2);
    const float cy=cosf(yaw/2),sy=sinf(yaw/2);
    return (imu_quatf_t){cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
                         cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy};
}

static bool sample_finite(const imu_sample_t *s)
{
    unsigned i;
    if(!isfinite(s->accel_temperature_c)) return false;
    for(i=0;i<3;++i) if(!isfinite(s->gyro_rad_s[i]) ||
        !isfinite(s->gyro_temperature_c[i]) || !isfinite(s->accel_m_s2[i])) return false;
    return true;
}

static imu_status_t fault(imu_t *imu, imu_output_t *out)
{
    imu->state=IMU_STATE_FAULT;
    imu->output.status=IMU_STATUS_NUMERICAL_FAULT;
    imu->output.initialized=false;
    imu->output.fusion_mode=IMU_FUSION_INVALID;
    imu->output.health_flags|=IMU_HEALTH_NUMERICAL_FAULT;
    *out=imu->output; return IMU_STATUS_NUMERICAL_FAULT;
}

static float seconds(uint64_t ns)
{
    return (float)(ns/UINT64_C(1000000000))+
           (float)(ns%UINT64_C(1000000000))*1e-9f;
}

static bool align(imu_t *n, const imu_calibrated_sample_t *v)
{
    if(n->config.init_mode==IMU_INIT_QUATERNION) n->state=IMU_STATE_RUNNING;
    else if(n->quality.static_detected && n->quality.static_enter_count>0 &&
            n->quality.mode_reasons==0) {
        const float count=(float)++n->alignment_count;
#define AVERAGE(field, value) n->field.x+=(value.x-n->field.x)/count; \
    n->field.y+=(value.y-n->field.y)/count; n->field.z+=(value.z-n->field.z)/count
        AVERAGE(alignment_accel,v->accel_m_s2);
        AVERAGE(alignment_gyro,v->gyro_rad_s);
#undef AVERAGE
        if(n->alignment_count>=n->config.alignment_samples) {
            const float roll=atan2f(-n->alignment_accel.y,-n->alignment_accel.z);
            const float pitch=atan2f(n->alignment_accel.x,
                hypotf(n->alignment_accel.y,n->alignment_accel.z));
            if(imu_esekf_init(&n->esekf,&n->config,
                from_euler(roll,pitch,n->config.initial_yaw_rad),n->alignment_gyro)!=IMU_STATUS_OK)
                return false;
            n->state=IMU_STATE_RUNNING;
        }
    } else n->alignment_count=0;
    return true;
}

imu_status_t imu_process(imu_t *imu, const imu_sample_t *s, imu_output_t *out)
{
    imu_t n; imu_calibrated_sample_t v={0},part; imu_sample_t calibrated,probe;
    bool gyro_good=true,accel_good,gap=false,was_running;
    imu_fusion_mode_t mode=IMU_FUSION_GYRO_ONLY;
    float dt=0; unsigned i; uint32_t flags=IMU_HEALTH_HEADING_UNOBSERVABLE;
    imu_status_t status=IMU_STATUS_OK;
    uint64_t delay_ns;
    if(!imu || !s || !out) return IMU_STATUS_BAD_ARGUMENT;
    if(!imu->configured) return IMU_STATUS_NOT_INITIALIZED;
    if(!imu_common_gyro_delay_ns(&imu->config,&delay_ns)) return IMU_STATUS_BAD_ARGUMENT;
    if(!sample_finite(s)) return IMU_STATUS_NONFINITE;
    if(imu->state==IMU_STATE_FAULT) { *out=imu->output; return IMU_STATUS_NUMERICAL_FAULT; }
    if(imu->has_timestamp) {
        if(s->timestamp_ns<=imu->last_timestamp_ns) return IMU_STATUS_TIME_REVERSED;
        dt=seconds(s->timestamp_ns-imu->last_timestamp_ns);
        if(dt<imu->config.min_dt_s) return IMU_STATUS_BAD_ARGUMENT;
        gap=dt>imu->config.max_dt_s;
    }
    n=*imu; was_running=n.state==IMU_STATE_RUNNING || n.state==IMU_STATE_DEGRADED;
    n.last_timestamp_ns=s->timestamp_ns; n.has_timestamp=true;
    for(i=0;i<3;++i) gyro_good=gyro_good && healthy(s->gyro_status[i],s->gyro_sample_fresh[i]);
    accel_good=healthy(s->accel_status,s->accel_sample_fresh);
    /* Calibrate independently: an invalid accelerometer observation must not
     * suppress valid gyro propagation (including temperature-range failures). */
    probe=*s; probe.accel_temperature_c=n.calibration.temperature_reference_c;
    for(i=0;i<3;++i) probe.accel_m_s2[i]=0;
    if(gyro_good && imu_calibrate_sample(&n.calibration,&probe,&part)) v.gyro_rad_s=part.gyro_rad_s;
    else gyro_good=false;
    probe=*s;
    for(i=0;i<3;++i) {
        probe.gyro_rad_s[i]=0; probe.gyro_temperature_c[i]=n.calibration.temperature_reference_c;
    }
    if(accel_good && imu_calibrate_sample(&n.calibration,&probe,&part)) v.accel_m_s2=part.accel_m_s2;
    else accel_good=false;
    for(i=0;i<IMU_NOTCH_MAX;++i) {
        if(!gyro_good || gap) imu_biquad_reset(&n.gyro_notches[i]);
        else if(!imu_biquad_process(&n.gyro_notches[i],v.gyro_rad_s,&v.gyro_rad_s)) return fault(imu,out);
        if(!accel_good || gap) imu_biquad_reset(&n.accel_notches[i]);
        else if(!imu_biquad_process(&n.accel_notches[i],v.accel_m_s2,&v.accel_m_s2)) accel_good=false;
    }
    if(gyro_good && accel_good && !gap) {
        /* Constant-rate bounded delay approximation at the common gyro age.
         * Unequal sensor ages are rejected before calibration mixes axes.
         * q_new=q_old*dq => fixed-vector body coordinates advance by conj(dq).
         * Equal signal-chain delays are an exact bypass. */
        imu_vec3f_t rotation={
            (v.gyro_rad_s.x-n.esekf.gyro_bias_rad_s.x)*(n.config.accel_delay_s-n.config.gyro_delay_s[0]),
            (v.gyro_rad_s.y-n.esekf.gyro_bias_rad_s.y)*(n.config.accel_delay_s-n.config.gyro_delay_s[1]),
            (v.gyro_rad_s.z-n.esekf.gyro_bias_rad_s.z)*(n.config.accel_delay_s-n.config.gyro_delay_s[2])};
        if(rotation.x!=0 || rotation.y!=0 || rotation.z!=0) {
            imu_quatf_t dq={1,0,0,0};
            if(!imu_quat_integrate(&dq,rotation,1) ||
               !imu_quat_rotate(imu_quat_conjugate(dq),v.accel_m_s2,&v.accel_m_s2))
                accel_good=false;
        }
    }
    if(was_running && dt>0) {
        /* Missing intervals are consumed once. A zero corrected rate freezes
         * attitude while propagating bias/noise covariance, never reusing data. */
        imu_vec3f_t rate=gyro_good && !gap?v.gyro_rad_s:n.esekf.gyro_bias_rad_s;
        if(imu_esekf_predict(&n.esekf,&n.config,rate,dt)!=IMU_STATUS_OK) return fault(imu,out);
    }
    calibrated=*s;
    calibrated.gyro_rad_s[0]=v.gyro_rad_s.x; calibrated.gyro_rad_s[1]=v.gyro_rad_s.y;
    calibrated.gyro_rad_s[2]=v.gyro_rad_s.z;
    calibrated.accel_m_s2[0]=v.accel_m_s2.x; calibrated.accel_m_s2[1]=v.accel_m_s2.y;
    calibrated.accel_m_s2[2]=v.accel_m_s2.z;
    if(gyro_good && accel_good && !gap) {
        float innovation=0;
        if(was_running) {
            imu_vec3f_t predicted;
            const float norm=imu_vec3_norm(v.accel_m_s2);
            if(!imu_quat_rotate(imu_quat_conjugate(n.esekf.q_nb),
                (imu_vec3f_t){0,0,-1},&predicted)) return fault(imu,out);
            if(isfinite(norm) && norm>1e-12f) {
                float dot=imu_vec3_dot(predicted,v.accel_m_s2)/norm;
                if(dot>1) dot=1;
                if(dot< -1) dot=-1;
                innovation=acosf(dot);
            } else accel_good=false;
        }
        if(accel_good && !imu_motion_quality_update(&n.quality,&n.config,&calibrated,innovation))
            accel_good=false;
    }
    if(!gyro_good || !accel_good || gap) {
        (void)imu_motion_quality_init(&n.quality,&n.config);
        n.alignment_count=0;
    }
    if(!was_running && gyro_good && accel_good && !gap && !align(&n,&v)) return fault(imu,out);
    if(was_running && gyro_good && accel_good && !gap) {
        if(n.quality.accel_weight>=n.config.accel_weight_min && n.quality.accel_weight>0) {
            imu_status_t observed=imu_esekf_update_gravity(&n.esekf,&n.config,
                v.accel_m_s2,n.quality.accel_weight);
            if(observed==IMU_STATUS_OK) mode=IMU_FUSION_GRAVITY;
            else {
                /* Measurement APIs are transactional. Preserve the successful
                 * prediction when an observation is ill-conditioned/rejected. */
                accel_good=false; n.quality.accel_weight=0;
            }
        }
        if(accel_good && n.quality.static_detected && n.quality.static_enter_count>0) {
            mode=IMU_FUSION_STATIC;
            if(seconds(s->timestamp_ns-n.last_zero_rate_ns)>=n.config.zero_rate_update_interval_s) {
                if(imu_esekf_update_zero_rate(&n.esekf,&n.config,v.gyro_rad_s,s->gyro_status)!=IMU_STATUS_OK)
                    return fault(imu,out);
                n.last_zero_rate_ns=s->timestamp_ns;
            }
        }
    }
    if(gyro_good && !gap) {
        n.esekf.angular_rate_rad_s=(imu_vec3f_t){
            v.gyro_rad_s.x-n.esekf.gyro_bias_rad_s.x,
            v.gyro_rad_s.y-n.esekf.gyro_bias_rad_s.y,
            v.gyro_rad_s.z-n.esekf.gyro_bias_rad_s.z};
    }
    if(!gyro_good) flags|=IMU_HEALTH_GYRO_DEGRADED;
    if(!accel_good) flags|=IMU_HEALTH_ACCEL_DEGRADED;
    if(!n.calibration_valid) flags|=IMU_HEALTH_UNCALIBRATED;
    if(n.calibration_valid && n.calibration.allow_temperature_extrapolation) {
        if(s->accel_temperature_c<n.calibration.temperature_min_c ||
           s->accel_temperature_c>n.calibration.temperature_max_c)
            flags|=IMU_HEALTH_TEMPERATURE_EXTRAPOLATED;
        for(i=0;i<3;++i)
            if(s->gyro_temperature_c[i]<n.calibration.temperature_min_c ||
               s->gyro_temperature_c[i]>n.calibration.temperature_max_c)
                flags|=IMU_HEALTH_TEMPERATURE_EXTRAPOLATED;
    }
    if(gap) flags|=IMU_HEALTH_TIMING_DEGRADED;
    if(!gyro_good || !accel_good) status=IMU_STATUS_SENSOR_DEGRADED;
    if(gap) status=IMU_STATUS_TIME_GAP;
    if(n.state==IMU_STATE_RUNNING || n.state==IMU_STATE_DEGRADED) {
        if(imu_esekf_get_output(&n.esekf,&n.output)!=IMU_STATUS_OK) return fault(imu,out);
        flags|=n.output.health_flags;
        n.state=status==IMU_STATUS_OK?IMU_STATE_RUNNING:IMU_STATE_DEGRADED;
        n.output.initialized=true;
        n.output.fusion_mode=mode;
    } else if(status==IMU_STATUS_OK) status=IMU_STATUS_NOT_INITIALIZED;
    n.output.status=status; n.output.health_flags=flags;
    n.output.timing_status=gap?IMU_STATUS_TIME_GAP:IMU_STATUS_OK;
    for(i=0;i<3;++i) n.output.gyro_status[i]=s->gyro_status[i];
    n.output.accel_status=s->accel_status;
    n.output.accel_weight=n.quality.accel_weight;
    n.output.calibration_valid=n.calibration_valid;
    n.output.timestamp_ns=s->timestamp_ns;
    imu_output_epoch(&n.output,delay_ns);
    if(n.state==IMU_STATE_RUNNING && n.output.status!=IMU_STATUS_OK) n.state=IMU_STATE_DEGRADED;
    *imu=n; *out=n.output; return out->status;
}

bool imu_reset(imu_t *imu)
{
    imu_t n;
    if(!imu || !imu->configured) return false;
    if(!imu_init(&n,&imu->config,imu->calibration_valid?&imu->calibration:NULL)) return false;
    *imu=n; return true;
}
imu_status_t imu_get_output(const imu_t *imu, imu_output_t *out)
{
    if(!imu || !out) return IMU_STATUS_BAD_ARGUMENT;
    if(!imu->configured) return IMU_STATUS_NOT_INITIALIZED;
    *out=imu->output; return out->status;
}
imu_status_t imu_update_magnetometer(imu_t *imu, const imu_mag_sample_t *sample)
{
    if(!imu || !sample) return IMU_STATUS_BAD_ARGUMENT;
    if(!isfinite(sample->field_ut.x) || !isfinite(sample->field_ut.y) ||
       !isfinite(sample->field_ut.z)) return IMU_STATUS_NONFINITE;
    return IMU_STATUS_AIDING_UNAVAILABLE;
}
