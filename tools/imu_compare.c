#include "sim/imu_sim.h"
#include <string.h>
#include "imu/imu.h"
#include "imu/imu_complementary.h"
#include "imu/imu_math.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
/* Frozen before the first estimator scenario run. Radians and seconds. */
#define LIMITS 0.05236,0.05236,0.174533,0.034907,4.0,0.349066,0.08,100.0
const imu_scenario_t imu_scenarios[]={
    {"static_arbitrary",LIMITS},{"compound_rotation",LIMITS},
    {"boundary_300_dps",LIMITS},{"linear_acceleration",LIMITS},
    {"sustained_turn",LIMITS},{"broadband_vibration",LIMITS},
    {"shock",LIMITS},{"bias_step",LIMITS},
    {"accel_dropout",LIMITS},{"recovery",LIMITS},
    {"combined_vibration_bias",LIMITS}
};
const unsigned imu_scenario_count=sizeof imu_scenarios/sizeof imu_scenarios[0];
static double wrap(double a) { return atan2(sin(a),cos(a)); }
static double angle(imu_quatf_t a,imu_quatf_t b)
{
    imu_quatf_t d=imu_quat_multiply(imu_quat_conjugate(a),b);
    return 2*atan2(sqrt((double)d.x*d.x+(double)d.y*d.y+(double)d.z*d.z),fabs(d.w));
}
static void trajectory(double t,imu_sim_input_t *u,void *ctx)
{
    unsigned id=*(unsigned *)ctx; const double pi=3.141592653589793;
    if(t<0) t=0;
    if(id==1 || id==10) {
        u->omega[0]=0.3*sin(2*pi*0.4*t); u->omega[1]=0.25*sin(2*pi*0.3*t);
        u->omega[2]=0.15*cos(2*pi*0.2*t);
    }
    if(id==2) u->omega[0]=(300*pi/180)*sin(2*pi*t);
    if(id==3 && t>=2 && t<4) u->acceleration[0]=4;
    if(id==4 && t>=2 && t<8) { u->omega[2]=0.5; u->acceleration[1]=3; }
    if((id==5 || id==9 || id==10) && t>=2 && t<8) {
        unsigned j;
        /* Multiple incommensurate bands and a chirp; sample white noise adds
         * broadband content. Amplitudes are sensor-output accelerations. */
        for(j=0;j<3;++j) u->acceleration[j]=0.5*sin(2*pi*(43+17*j)*t)+
            0.3*sin(2*pi*(91+11*j)*t)+0.2*sin(2*pi*(15*t+4*t*t)+j);
    }
    if(id==6 && t>=3 && t<3.02) { u->acceleration[2]=60; u->accel_status|=IMU_SENSOR_OVERRANGE; }
    if((id==7 || id==9 || id==10) && t>=4) {
        u->bias[0]=0.01; u->bias[1]=-0.007; u->bias[2]=0.003;
        u->temperature=35;
    }
    if((id==8 || id==9) && t>=3 && t<8) u->accel_fresh=false;
}
/* Pre-update gravity NIS: solve S*x=r in double precision; same model as
 * ESKF H=I-hh', S=H*P*H'+R. Not a post-update residual proxy. */
static double gravity_nis(const imu_esekf_t *e,const imu_config_t *c,
                          const imu_sample_t *s,float weight)
{
    imu_vec3f_t hv,z={s->accel_m_s2[0],s->accel_m_s2[1],s->accel_m_s2[2]},r;
    double h[3],H[3][3],a[3][4],res[3],norm=imu_vec3_norm(z),nis=0;
    unsigned i,j,k,l;
    if(weight<=0 || norm<=0) return 0;
    (void)imu_quat_rotate(imu_quat_conjugate(e->q_nb),(imu_vec3f_t){0,0,-1},&hv);
    z.x/=(float)norm; z.y/=(float)norm; z.z/=(float)norm; r=imu_vec3_cross(z,hv);
    h[0]=hv.x;h[1]=hv.y;h[2]=hv.z;res[0]=r.x;res[1]=r.y;res[2]=r.z;
    for(i=0;i<3;++i) for(j=0;j<3;++j) H[i][j]=(i==j?1:0)-h[i]*h[j];
    for(i=0;i<3;++i) {
        for(j=0;j<3;++j) {
            a[i][j]=i==j?c->accel_direction_noise_variance/weight:0;
            for(k=0;k<3;++k) for(l=0;l<3;++l) a[i][j]+=H[i][k]*e->P[k][l]*H[j][l];
        }
        a[i][3]=res[i];
    }
    for(i=0;i<3;++i) {
        double pivot=a[i][i];
        for(j=i;j<4;++j) a[i][j]/=pivot;
        for(k=0;k<3;++k) if(k!=i) { double f=a[k][i]; for(j=i;j<4;++j) a[k][j]-=f*a[i][j]; }
    }
    for(i=0;i<3;++i) nis+=res[i]*a[i][3];
    return nis;
}
bool imu_run_scenario(unsigned scenario,uint32_t seed,imu_metrics_t m[2])
{
    imu_sim_t sim; imu_sim_config_t sc; imu_config_t c;
    imu_t es; imu_complementary_t cf; imu_output_t out[2];
    imu_quatf_t truth,previous_truth={1,0,0,0},previous[2];
    imu_vec3f_t bias; double last_bad[2]={8,8}; unsigned i,k,j;
    imu_calibration_t cal; clock_t ticks[2]={0,0};
    if(scenario>=imu_scenario_count || !m) return false;
    memset(m,0,2*sizeof *m); memset(&cal,0,sizeof cal);
    cal.version=IMU_CALIBRATION_VERSION; cal.valid=cal.checksum_valid=true;
    cal.temperature_min_c=-40;cal.temperature_max_c=85;
    for(i=0;i<3;++i) cal.gyro_scale_alignment[i][i]=cal.accel_scale_alignment[i][i]=cal.body_rotation[i][i]=1;
    imu_sim_defaults(&sc); sc.seed=seed;sc.motion=trajectory;sc.context=&scenario;
    /* ADXRS453 0.015 deg/s/sqrt(Hz), ADXL355 25 ug/sqrt(Hz).
     * Ideal rectangular bandwidth approximation: 77.5 Hz and 125 Hz.
     * This models output noise, not the complete analog/decimation transfer. */
    sc.gyro_sigma=0.015*3.141592653589793/180*sqrt(77.5);
    sc.accel_sigma=25e-6*9.80665*sqrt(125);sc.bias_rw=1e-5;
    if(scenario==0) sc.initial_q=(imu_quatf_t){0.8785122f,0.2968829f,-0.07043934f,0.3675801f};
    imu_config_defaults(&c); c.init_mode=IMU_INIT_QUATERNION;c.initial_q_nb=sc.initial_q;
    c.accel_delay_s=(float)sc.accel_delay_s;
    for(i=0;i<3;++i)c.gyro_delay_s[i]=(float)sc.gyro_delay_s;
    if(!imu_sim_init(&sim,&sc) || !imu_init(&es,&c,&cal) ||
       imu_complementary_init(&cf,&c,c.initial_q_nb,(imu_vec3f_t){0,0,0})!=IMU_STATUS_OK) return false;
    previous[0]=previous[1]=sc.initial_q;
    for(i=0;i<=5820;++i) {
        imu_sample_t raw,s; imu_vec3f_t euler,accel,rotation;
        imu_quatf_t dq={1,0,0,0}; imu_motion_quality_t quality;
        imu_esekf_t predicted=es.esekf; imu_status_t status[2]; clock_t begin;
        double t=(double)i/485,nis;
        if(!imu_sim_next(&sim,&raw,&truth,&bias)) return false;
        s=raw;
        rotation=(imu_vec3f_t){(raw.gyro_rad_s[0]-es.esekf.gyro_bias_rad_s.x)*0.00127f,
            (raw.gyro_rad_s[1]-es.esekf.gyro_bias_rad_s.y)*0.00127f,
            (raw.gyro_rad_s[2]-es.esekf.gyro_bias_rad_s.z)*0.00127f};
        (void)imu_quat_integrate(&dq,rotation,1);
        (void)imu_quat_rotate(imu_quat_conjugate(dq),(imu_vec3f_t){raw.accel_m_s2[0],raw.accel_m_s2[1],raw.accel_m_s2[2]},&accel);
        s.accel_m_s2[0]=accel.x;s.accel_m_s2[1]=accel.y;s.accel_m_s2[2]=accel.z;
        begin=clock(); status[0]=imu_process(&es,&raw,&out[0]); ticks[0]+=clock()-begin;
        quality=es.quality;
        /* Share NIS/hemisphere rejection as well as the quality decision. */
        if(out[0].fusion_mode==IMU_FUSION_GYRO_ONLY) quality.accel_weight=0;
        begin=clock();status[1]=imu_complementary_process(&cf,&c,&s,&quality,&out[1]);ticks[1]+=clock()-begin;
        if(i) (void)imu_esekf_predict(&predicted,&c,(imu_vec3f_t){raw.gyro_rad_s[0],raw.gyro_rad_s[1],raw.gyro_rad_s[2]},c.nominal_dt_s);
        nis=gravity_nis(&predicted,&c,&s,raw.accel_sample_fresh?es.quality.accel_weight:0);
        (void)imu_quat_to_euler_ned(truth,&euler);
        for(k=0;k<2;++k) {
            double er=wrap(out[k].roll_rad-euler.x),ep=wrap(out[k].pitch_rad-euler.y);
            double ey=wrap(out[k].yaw_rad-euler.z),qe=angle(out[k].q_nb,truth);
            imu_quatf_t di=imu_quat_multiply(imu_quat_conjugate(previous[k]),out[k].q_nb);
            imu_quatf_t dt=imu_quat_multiply(imu_quat_conjugate(previous_truth),truth);
            if(!isfinite(qe) || !isfinite(er) || !isfinite(ep) || !isfinite(ey) ||
               fabs(imu_quat_norm(out[k].q_nb)-1)>1e-5 ||
               status[k]==IMU_STATUS_NUMERICAL_FAULT || !out[k].initialized) ++m[k].failures;
            if(i && angle(di,dt)>imu_scenarios[scenario].discontinuity_limit) ++m[k].discontinuities;
            if(!raw.accel_sample_fresh || (raw.accel_status&IMU_SENSOR_FAULT_MASK))
                if(!(out[k].health_flags&IMU_HEALTH_ACCEL_DEGRADED) || out[k].fusion_mode!=IMU_FUSION_GYRO_ONLY) ++m[k].unreported_degraded;
            m[k].roll_rms+=er*er;m[k].pitch_rms+=ep*ep;m[k].yaw_rms+=ey*ey;
            m[k].peak=fmax(m[k].peak,qe);
            if(t>=8 && hypot(er,ep)>0.034907) last_bad[k]=t;
            if(k==0 && raw.accel_sample_fresh && es.quality.accel_weight>=c.accel_weight_min) {
                m[k].nis_mean+=nis;m[k].nis_max=fmax(m[k].nis_max,nis);++m[k].nis_samples;
            }
            if(k==0) {
                double trace=0; for(j=0;j<3;++j)trace+=es.esekf.P[j][j];
                m[k].consistency_max=fmax(m[k].consistency_max,qe*qe/trace);
                if(!imu_esekf_covariance_valid(&es.esekf)) ++m[k].failures;
            }
            previous[k]=out[k].q_nb;++m[k].samples;
        }
        previous_truth=truth;
    }
    for(k=0;k<2;++k) {
        m[k].roll_rms=sqrt(m[k].roll_rms/m[k].samples);m[k].pitch_rms=sqrt(m[k].pitch_rms/m[k].samples);
        m[k].yaw_rms=sqrt(m[k].yaw_rms/m[k].samples);m[k].recovery=last_bad[k]>11.5?INFINITY:last_bad[k]-8;
        m[k].bias_error=sqrt(pow(out[k].gyro_bias_rad_s.x-bias.x,2)+pow(out[k].gyro_bias_rad_s.y-bias.y,2)+pow(out[k].gyro_bias_rad_s.z-bias.z,2));
        if(m[k].nis_samples)m[k].nis_mean/=m[k].nis_samples;
        m[k].cpu_s=(double)ticks[k]/CLOCKS_PER_SEC;
    }
    m[0].memory_bytes=sizeof es; m[1].memory_bytes=sizeof cf+sizeof(imu_motion_quality_t);
    return true;
}
bool imu_compare_accept(const imu_metrics_t *e,const imu_metrics_t *c)
{
    return hypot(e->roll_rms,e->pitch_rms)<hypot(c->roll_rms,c->pitch_rms) &&
        e->failures<=c->failures && e->discontinuities<=c->discontinuities;
}
#ifndef IMU_COMPARE_NO_MAIN
int main(int argc,char **argv)
{
    unsigned seeds=30,seed,k; const char *path=NULL; FILE *f;
    imu_metrics_t m[2],total[2]={{0}};
    for(k=1;k<(unsigned)argc;++k) {
        if(strcmp(argv[k],"--seeds")==0 && k+1<(unsigned)argc) {
            char *end; unsigned long n=strtoul(argv[++k],&end,10);
            if(*end || n<30 || n>10000) { fprintf(stderr,"--seeds must be 30..10000\n");return 2; }seeds=(unsigned)n;
        } else if(strcmp(argv[k],"--output")==0 && k+1<(unsigned)argc) path=argv[++k];
        else { fprintf(stderr,"usage: imu_compare --seeds 30 --output comparison.csv\n");return 2; }
    }
    f=path?fopen(path,"w"):stdout;if(!f){perror(path);return 2;}
    fprintf(f,"seed,estimator,roll_rms_rad,pitch_rms_rad,peak_error_rad,recovery_s,bias_error_rad_s,nis_mean,nis_max,numerical_failures,cpu_s,static_memory_bytes,discontinuities,yaw_relative_rms_rad,consistency_max\n");
    for(seed=1;seed<=seeds;++seed) {
        if(!imu_run_scenario(10,seed,m))return 2;
        for(k=0;k<2;++k) {
            fprintf(f,"%u,%s,%.9g,%.9g,%.9g,%.9g,%.9g,",seed,k?"complementary":"esekf",m[k].roll_rms,m[k].pitch_rms,m[k].peak,m[k].recovery,m[k].bias_error);
            if(k)fprintf(f,"NA,NA,");else fprintf(f,"%.9g,%.9g,",m[k].nis_mean,m[k].nis_max);
            fprintf(f,"%u,%.9g,%lu,%u,%.9g,",m[k].failures,m[k].cpu_s,(unsigned long)m[k].memory_bytes,m[k].discontinuities,m[k].yaw_rms);
            if(k)fprintf(f,"NA\n");else fprintf(f,"%.9g\n",m[k].consistency_max);
            total[k].roll_rms+=m[k].roll_rms*m[k].roll_rms/seeds;
            total[k].pitch_rms+=m[k].pitch_rms*m[k].pitch_rms/seeds;
            total[k].failures+=m[k].failures;total[k].discontinuities+=m[k].discontinuities;
        }
    }
    if(ferror(f) || (path && fclose(f)!=0)) return 2;
    for(k=0;k<2;++k){total[k].roll_rms=sqrt(total[k].roll_rms);total[k].pitch_rms=sqrt(total[k].pitch_rms);}
    fprintf(stderr,"%u fixed seeds: ESKF tilt RMS %.9g, complementary %.9g rad; %s\n",seeds,
        hypot(total[0].roll_rms,total[0].pitch_rms),hypot(total[1].roll_rms,total[1].pitch_rms),
        imu_compare_accept(&total[0],&total[1])?"ESKF accepted":"ESKF criterion FAILED; complementary supported default candidate");
    return imu_compare_accept(&total[0],&total[1])?0:1;
}
#endif
