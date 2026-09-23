#include "imu_sim.h"
#include <string.h>
#include <math.h>
static double uniform(imu_sim_t *s)
{
    uint32_t x=s->rng; x^=x<<13; x^=x>>17; x^=x<<5; s->rng=x;
    return ((double)x+0.5)/4294967296.0;
}
static double normal(imu_sim_t *s)
{ double u=uniform(s); return sqrt(-2*log(u))*cos(6.283185307179586*uniform(s)); }
static imu_sim_input_t input(const imu_sim_t *s,double t)
{
    imu_sim_input_t u; unsigned i; memset(&u,0,sizeof u); u.temperature=25;
    u.accel_status=IMU_SENSOR_DATA_VALID; u.accel_fresh=true;
    for(i=0;i<3;++i) { u.gyro_status[i]=IMU_SENSOR_DATA_VALID; u.gyro_fresh[i]=true; }
    if(s->config.motion) s->config.motion(t,&u,s->config.context);
    return u;
}
static void push(imu_sim_t *s)
{
    imu_sim_point_t *p;
    s->head=(s->head+1)%IMU_SIM_QUEUE;
    if(s->count<IMU_SIM_QUEUE) ++s->count;
    p=&s->queue[s->head]; p->t=s->t; memcpy(p->q,s->q,sizeof p->q);
    p->input=input(s,s->t);
}
/* Truth integration is independent double-precision quaternion arithmetic. */
static void advance(imu_sim_t *s,double dt)
{
    imu_sim_input_t u=input(s,s->t+0.5*dt);
    double w=u.omega[0],x=u.omega[1],y=u.omega[2];
    double n=sqrt(w*w+x*x+y*y), k=n>0?sin(0.5*n*dt)/n:0.5*dt;
    double d[4]={cos(0.5*n*dt),w*k,x*k,y*k},q[4];
    q[0]=s->q[0]*d[0]-s->q[1]*d[1]-s->q[2]*d[2]-s->q[3]*d[3];
    q[1]=s->q[0]*d[1]+s->q[1]*d[0]+s->q[2]*d[3]-s->q[3]*d[2];
    q[2]=s->q[0]*d[2]-s->q[1]*d[3]+s->q[2]*d[0]+s->q[3]*d[1];
    q[3]=s->q[0]*d[3]+s->q[1]*d[2]-s->q[2]*d[1]+s->q[3]*d[0];
    memcpy(s->q,q,sizeof q); s->t+=dt; push(s);
}
void imu_sim_defaults(imu_sim_config_t *c)
{
    unsigned i; memset(c,0,sizeof *c); c->seed=1; c->substep_s=0.0001;
    c->gyro_delay_s=0.004; c->accel_delay_s=0.00527;
    c->reference_temperature=25; c->initial_q.w=1;
    for(i=0;i<3;++i) c->gyro_matrix[i][i]=c->accel_matrix[i][i]=1;
}
bool imu_sim_init(imu_sim_t *s,const imu_sim_config_t *c)
{
    unsigned i; double norm;
    if(!s || !c || !isfinite(c->substep_s) || c->substep_s<=0 ||
       !isfinite(c->gyro_delay_s) || !isfinite(c->accel_delay_s) ||
       c->gyro_delay_s<0 || c->accel_delay_s<0 ||
       /* Every frame boundary may add a shortened integration step. Count
        * those points as well as full substeps, plus bracketing endpoints. */
       ceil(fmax(c->gyro_delay_s,c->accel_delay_s)/c->substep_s)+
       ceil(fmax(c->gyro_delay_s,c->accel_delay_s)*485)+3>IMU_SIM_QUEUE ||
       !isfinite(c->gyro_sigma) || !isfinite(c->accel_sigma) || !isfinite(c->bias_rw) ||
       c->gyro_sigma<0 || c->accel_sigma<0 || c->bias_rw<0) return false;
    memset(s,0,sizeof *s); s->config=*c; s->rng=c->seed?c->seed:1;
    s->q[0]=c->initial_q.w; s->q[1]=c->initial_q.x;
    s->q[2]=c->initial_q.y; s->q[3]=c->initial_q.z;
    norm=sqrt(s->q[0]*s->q[0]+s->q[1]*s->q[1]+s->q[2]*s->q[2]+s->q[3]*s->q[3]);
    if(!isfinite(norm) || norm<1e-12) return false;
    for(i=0;i<4;++i) s->q[i]/=norm;
    /* Constant prehistory until t=0; emitted frames have valid filled queues. */
    s->t=-fmax(c->gyro_delay_s,c->accel_delay_s)-2*c->substep_s;
    while(s->t<0) { push(s); s->t=fmin(0,s->t+c->substep_s); }
    push(s); return true;
}
static bool delayed(const imu_sim_t *s,double t,imu_sim_point_t *result)
{
    unsigned j; imu_sim_point_t hi=s->queue[s->head],lo=hi,r=hi;
    for(j=1;j<s->count;++j) {
        lo=s->queue[(s->head+IMU_SIM_QUEUE-j)%IMU_SIM_QUEUE];
        if(lo.t<=t) {
            double f=(t-lo.t)/(hi.t-lo.t); unsigned k;
            r=lo; r.t=t;
            for(k=0;k<4;++k) r.q[k]=lo.q[k]+f*(hi.q[k]-lo.q[k]);
            for(k=0;k<3;++k) {
                r.input.omega[k]=lo.input.omega[k]+f*(hi.input.omega[k]-lo.input.omega[k]);
                r.input.acceleration[k]=lo.input.acceleration[k]+f*(hi.input.acceleration[k]-lo.input.acceleration[k]);
                r.input.bias[k]=lo.input.bias[k]+f*(hi.input.bias[k]-lo.input.bias[k]);
            }
            r.input.temperature=lo.input.temperature+f*(hi.input.temperature-lo.input.temperature);
            /* Interpolation uses both measurements: keep any fault and require
             * both contributors valid/fresh. At an endpoint use only that
             * endpoint's diagnostics. Never label delayed data with now. */
            if(f>=1) {
                for(k=0;k<3;++k) {
                    r.input.gyro_status[k]=hi.input.gyro_status[k];
                    r.input.gyro_fresh[k]=hi.input.gyro_fresh[k];
                }
                r.input.accel_status=hi.input.accel_status;r.input.accel_fresh=hi.input.accel_fresh;
            } else if(f>0) {
                for(k=0;k<3;++k) {
                    uint32_t a=lo.input.gyro_status[k],b=hi.input.gyro_status[k];
                    r.input.gyro_status[k]=((a|b)&~IMU_SENSOR_DATA_VALID)|(a&b&IMU_SENSOR_DATA_VALID);
                    r.input.gyro_fresh[k]=lo.input.gyro_fresh[k] && hi.input.gyro_fresh[k];
                }
                r.input.accel_status=((lo.input.accel_status|hi.input.accel_status)&~IMU_SENSOR_DATA_VALID)|
                    (lo.input.accel_status&hi.input.accel_status&IMU_SENSOR_DATA_VALID);
                r.input.accel_fresh=lo.input.accel_fresh && hi.input.accel_fresh;
            }
            *result=r;return true;
        }
        hi=lo;
    }
    /* A caller-mutated configuration must never silently substitute the
     * oldest available point for the requested physical measurement time. */
    return false;
}
bool imu_sim_next(imu_sim_t *s,imu_sample_t *x,imu_quatf_t *q,imu_vec3f_t *b)
{
    double target,force[3],bias[3]; unsigned i,j;
    imu_sim_point_t g,a;
    if(!s || !x || !q || !b) return false;
    target=(double)s->frame/485;
    while(s->t+1e-12<target) advance(s,fmin(s->config.substep_s,target-s->t));
    if(!delayed(s,target-s->config.gyro_delay_s,&g) ||
       !delayed(s,target-s->config.accel_delay_s,&a))return false;
    force[0]=-9.80665*2*(a.q[1]*a.q[3]-a.q[0]*a.q[2])+a.input.acceleration[0];
    force[1]=-9.80665*2*(a.q[2]*a.q[3]+a.q[0]*a.q[1])+a.input.acceleration[1];
    force[2]=-9.80665*(1-2*(a.q[1]*a.q[1]+a.q[2]*a.q[2]))+a.input.acceleration[2];
    memset(x,0,sizeof *x); x->timestamp_ns=UINT64_C(1000000000)+s->frame*UINT64_C(1000000000)/485;
    for(i=0;i<3;++i) {
        double tg=g.input.temperature-s->config.reference_temperature;
        double ta=a.input.temperature-s->config.reference_temperature;
        double gv=0,av=0;
        for(j=0;j<3;++j) { gv+=s->config.gyro_matrix[i][j]*g.input.omega[j]; av+=s->config.accel_matrix[i][j]*force[j]; }
        if(s->frame) s->walk[i]+=s->config.bias_rw*sqrt(1.0/485)*normal(s);
        bias[i]=g.input.bias[i]+s->walk[i]+s->config.gyro_bias_poly[i][0]+tg*(s->config.gyro_bias_poly[i][1]+tg*s->config.gyro_bias_poly[i][2]);
        x->gyro_rad_s[i]=(float)(gv+bias[i]+s->config.gyro_sigma*normal(s));
        x->accel_m_s2[i]=(float)(av+s->config.accel_bias_poly[i][0]+ta*(s->config.accel_bias_poly[i][1]+ta*s->config.accel_bias_poly[i][2])+s->config.accel_sigma*normal(s));
        x->gyro_temperature_c[i]=(float)g.input.temperature;
        x->gyro_status[i]=g.input.gyro_status[i]; x->gyro_sample_fresh[i]=g.input.gyro_fresh[i];
    }
    x->accel_temperature_c=(float)a.input.temperature;
    x->accel_status=a.input.accel_status; x->accel_sample_fresh=a.input.accel_fresh;
    *q=(imu_quatf_t){(float)s->q[0],(float)s->q[1],(float)s->q[2],(float)s->q[3]};
    *b=(imu_vec3f_t){(float)bias[0],(float)bias[1],(float)bias[2]};
    ++s->frame; return true;
}
