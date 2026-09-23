#include "test_framework.h"
#include "sim/imu_sim.h"
#include <string.h>
static void rate(double t,imu_sim_input_t *u,void *ctx)
{ (void)ctx; u->omega[0]=1; u->temperature=30; u->acceleration[0]=t; }
static void ramp(double t,imu_sim_input_t *u,void *ctx)
{ (void)ctx;u->omega[2]=t;u->accel_fresh=false;u->gyro_status[1]|=IMU_SENSOR_PARITY_ERROR; }
static void sensor_shock(double t,imu_sim_input_t *u,void *ctx)
{
    (void)ctx;
    if(t>=0.03 && t<0.05) {
        u->acceleration[2]=60;u->accel_status|=IMU_SENSOR_OVERRANGE;
        u->omega[0]=2;u->gyro_status[0]|=IMU_SENSOR_PARITY_ERROR;
    }
}
static void delayed_status_test(void)
{
    imu_sim_config_t c;imu_sim_t sim;imu_sample_t x;imu_quatf_t q;
    imu_vec3f_t bias;unsigned i,accel_faults=0,gyro_faults=0;
    imu_sim_defaults(&c);c.motion=sensor_shock;
    TEST_ASSERT(imu_sim_init(&sim,&c));
    for(i=0;i<40;++i) {
        TEST_ASSERT(imu_sim_next(&sim,&x,&q,&bias));
        if(x.accel_m_s2[2]>-9) {
            ++accel_faults;TEST_ASSERT(x.accel_status&IMU_SENSOR_OVERRANGE);
        }
        if(x.gyro_rad_s[0]>0.01f) {
            ++gyro_faults;TEST_ASSERT(x.gyro_status[0]&IMU_SENSOR_PARITY_ERROR);
        }
    }
    TEST_ASSERT(accel_faults>0 && gyro_faults>0);
}
static void simulator_tests(void)
{
    imu_sim_config_t c; imu_sim_t a,b; imu_sample_t x={0},y={0};
    imu_quatf_t q={0},r={0}; imu_vec3f_t bias; unsigned i;
    double sum=0,square=0;
    imu_sim_defaults(&c);
    TEST_ASSERT(imu_sim_init(&a,&c));
    TEST_ASSERT(imu_sim_next(&a,&x,&q,&bias));
    TEST_ASSERT_NEAR(x.accel_m_s2[2],-9.80665,1e-5);
    TEST_ASSERT_NEAR(x.gyro_rad_s[0],0,1e-8);
    TEST_ASSERT(x.accel_sample_fresh && x.gyro_status[0]==IMU_SENSOR_DATA_VALID);
    c.motion=rate; c.gyro_bias_poly[1][0]=0.1; c.gyro_bias_poly[1][1]=0.02;
    c.gyro_bias_poly[1][2]=0.003;
    TEST_ASSERT(imu_sim_init(&a,&c));
    for(i=0;i<=485;++i) (void)imu_sim_next(&a,&x,&q,&bias);
    TEST_ASSERT_NEAR(q.w,0.877582562,2e-6);
    TEST_ASSERT_NEAR(q.x,0.479425539,2e-6);
    TEST_ASSERT_NEAR(x.gyro_rad_s[1],0.275,1e-6);
    TEST_ASSERT_NEAR(x.accel_m_s2[0],0.99473,2e-5);
    TEST_ASSERT_NEAR(x.accel_m_s2[1],-9.80665*sin(0.99473),2e-5);
    /* Distinct physical measurement epochs, not rounded frame delays. */
    TEST_ASSERT_NEAR(a.queue[a.head].t,1,1e-9);
    imu_sim_defaults(&c); c.gyro_sigma=0.03; c.accel_sigma=0.2; c.seed=129;
    TEST_ASSERT(imu_sim_init(&a,&c)); TEST_ASSERT(imu_sim_init(&b,&c));
    for(i=0;i<20000;++i) {
        TEST_ASSERT(imu_sim_next(&a,&x,&q,&bias));
        TEST_ASSERT(imu_sim_next(&b,&y,&r,&bias));
        TEST_ASSERT(memcmp(&x,&y,sizeof x)==0 && memcmp(&q,&r,sizeof q)==0);
        sum+=x.gyro_rad_s[0]; square+=x.gyro_rad_s[0]*x.gyro_rad_s[0];
    }
    TEST_ASSERT_NEAR(sum/20000,0,0.001);
    TEST_ASSERT_NEAR(sqrt(square/20000-(sum/20000)*(sum/20000)),0.03,0.001);
    imu_sim_defaults(&c);c.motion=ramp;
    TEST_ASSERT(imu_sim_init(&a,&c));
    for(i=0;i<=485;++i)(void)imu_sim_next(&a,&x,&q,&bias);
    TEST_ASSERT_NEAR(x.gyro_rad_s[2],0.996,2e-6);
    TEST_ASSERT_NEAR(q.z,0.247403959,2e-6);
    TEST_ASSERT(!x.accel_sample_fresh && (x.gyro_status[1]&IMU_SENSOR_PARITY_ERROR));
    imu_sim_defaults(&c);c.gyro_sigma=NAN;
    TEST_ASSERT(!imu_sim_init(&a,&c));
    /* Frame boundaries split a nominal 10 ms substep into 485 history points
     * per second: a 256-entry queue cannot retain a one-second delay. */
    imu_sim_defaults(&c);c.substep_s=0.01;c.gyro_delay_s=c.accel_delay_s=1;
    TEST_ASSERT(!imu_sim_init(&a,&c));
    c.gyro_delay_s=c.accel_delay_s=0.4;c.motion=ramp;
    TEST_ASSERT(imu_sim_init(&a,&c));
    for(i=0;i<=970;++i)TEST_ASSERT(imu_sim_next(&a,&x,&q,&bias));
    TEST_ASSERT_NEAR(x.gyro_rad_s[2],1.6,2e-6);
    TEST_ASSERT_NEAR(q.z,0.841470985,2e-6);
    a.config.gyro_delay_s=1;
    TEST_ASSERT(!imu_sim_next(&a,&x,&q,&bias));
    delayed_status_test();
}
static void scenario_tests(void)
{
    unsigned i; imu_metrics_t m[2];
    for(i=0;i<imu_scenario_count;++i) {
        const imu_scenario_t *f=&imu_scenarios[i];
        TEST_ASSERT(imu_run_scenario(i,42,m));
        printf("%s roll=%.6g pitch=%.6g peak=%.6g yaw_rms=%.6g recovery=%.6g failures=%u\n",
               f->name,m[0].roll_rms,m[0].pitch_rms,m[0].peak,m[0].yaw_rms,m[0].recovery,m[0].failures);
        TEST_ASSERT(m[0].samples==5821);
        TEST_ASSERT(m[0].roll_rms<f->roll_rms_limit && m[0].pitch_rms<f->pitch_rms_limit);
        TEST_ASSERT(m[0].peak<f->peak_limit);
        TEST_ASSERT(m[0].yaw_rms<f->yaw_limit);
        TEST_ASSERT(m[0].recovery<=f->recovery_limit);
        TEST_ASSERT(m[0].failures==0 && m[0].discontinuities==0 && m[0].unreported_degraded==0);
        TEST_ASSERT(m[0].bias_error<f->bias_limit && m[0].consistency_max<f->consistency_limit);
    }
}
static void recovery_tests(void)
{
    const unsigned scenarios[3]={3,6,7};
    const double unsettled[3]={4.5,3.2,4.75},expected[3]={0.5,0.18,0.75};
    unsigned i;imu_recovery_t r;
    /* Pulse ends at 4 s, shock at 3.02 s, bias settles to its new value at
     * the 4 s step. Excursions before each endpoint do not count. */
    for(i=0;i<3;++i) {
        imu_recovery_init(&r,imu_scenarios[scenarios[i]].disturbance_end_s);
        imu_recovery_observe(&r,1,0.2);
        TEST_ASSERT_NEAR(imu_recovery_result(&r,12),0,1e-12);
        imu_recovery_observe(&r,unsettled[i],0.04);
        imu_recovery_observe(&r,unsettled[i]+0.1,0.01);
        TEST_ASSERT_NEAR(imu_recovery_result(&r,12),expected[i],1e-12);
        imu_recovery_observe(&r,11.75,0.04);
        TEST_ASSERT(isinf(imu_recovery_result(&r,12)));
    }
}
/* Locks the field mapping that the old positional initializer obscured.
 * In particular, 0.062285 yaw RMS passes while a 0.04 jump is counted. */
static void frozen_limit_mapping(void)
{
    unsigned i;
    for(i=0;i<imu_scenario_count;++i) {
        TEST_ASSERT(imu_scenarios[i].yaw_limit==0.349066);
        TEST_ASSERT(imu_scenarios[i].discontinuity_limit==0.034907);
        TEST_ASSERT(imu_scenarios[i].peak_limit==0.174533);
        TEST_ASSERT(0.062285<imu_scenarios[i].yaw_limit);
        TEST_ASSERT(0.04>imu_scenarios[i].discontinuity_limit);
    }
}
static void comparison_test(void)
{
    imu_metrics_t total[2]={{0}},m[2]; unsigned seed,k;
    /* Seeds 1..30, equal duration and sample count, pooled tilt mean square.
     * No change to seeds/thresholds/aggregation after inspecting results. */
    for(seed=1;seed<=30;++seed) {
        TEST_ASSERT(imu_run_scenario(10,seed,m));
        for(k=0;k<2;++k) {
            total[k].roll_rms+=m[k].roll_rms*m[k].roll_rms/30;
            total[k].pitch_rms+=m[k].pitch_rms*m[k].pitch_rms/30;
            total[k].failures+=m[k].failures;
            total[k].discontinuities+=m[k].discontinuities;
        }
    }
    for(k=0;k<2;++k) {
        total[k].roll_rms=sqrt(total[k].roll_rms);
        total[k].pitch_rms=sqrt(total[k].pitch_rms);
    }
    printf("30 seeds ESKF tilt RMS %.9g complementary %.9g\n",
       hypot(total[0].roll_rms,total[0].pitch_rms),hypot(total[1].roll_rms,total[1].pitch_rms));
    TEST_ASSERT(imu_compare_accept(&total[0],&total[1]));
}
int main(int argc,char **argv)
{
    if(argc==3 && strcmp(argv[1],"--diagnose")==0) {
        imu_metrics_t m[2];FILE *trace=fopen(argv[2],"w");
        TEST_ASSERT(trace!=NULL);
        if(trace) {TEST_ASSERT(imu_run_scenario_trace(10,42,m,trace));TEST_ASSERT(fclose(trace)==0);}
    }
    else if(argc==2 && strcmp(argv[1],"--simulator")==0) simulator_tests();
    else if(argc==2 && strcmp(argv[1],"--comparison")==0) comparison_test();
    else if(argc==2 && strcmp(argv[1],"--recovery")==0) recovery_tests();
    else if(argc==2 && strcmp(argv[1],"--limits")==0) frozen_limit_mapping();
    else scenario_tests();
    return test_summary();
}
