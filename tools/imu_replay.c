#include "imu/imu.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <float.h>
#include <ctype.h>
/* SI rates and force, degrees C, decimal status masks, freshness 0 or 1.
 * Exact ordered schema; no quoting, extra columns, or locale separators. */
static const char schema[]="timestamp_ns,gx,gy,gz,tgx,tgy,tgz,gstatus_x,gstatus_y,gstatus_z,gfresh_x,gfresh_y,gfresh_z,ax,ay,az,ta,astatus,afresh";
static int fail(FILE *error,unsigned line,const char *message)
{ fprintf(error,"line %u: %s\n",line,message);return 2; }
static bool unsigned_number(const char *s,uint64_t limit,uint64_t *out)
{
    uint64_t value=0;
    if(!*s)return false;
    while(*s) {
        unsigned digit=(unsigned)(*s-'0');
        if(digit>9 || digit>limit || value>(limit-digit)/10)return false;
        value=value*10+digit;++s;
    }
    *out=value;return true;
}
static bool number(const char *s,float *out)
{
    char *end;double value;
    if(!*s || isspace((unsigned char)*s))return false;
    errno=0;value=strtod(s,&end);
    if(errno || *end || !isfinite(value) || fabs(value)>FLT_MAX)return false;
    *out=(float)value;return true;
}
static int replay(FILE *input,FILE *output,FILE *error)
{
    char line[2048];unsigned n=1,rows=0;uint64_t last=0;
    imu_t imu;imu_config_t c;
    if(!fgets(line,sizeof line,input))return fail(error,1,"missing CSV schema");
    line[strcspn(line,"\r\n")]=0;
    if(strcmp(line,schema)!=0)return fail(error,1,"expected exact 19-column CSV schema");
    /* Replay has no supplied calibration or truth attitude. Identity start is
     * an explicit diagnostic assumption, marked UNCALIBRATED on every row. */
    imu_config_defaults(&c);c.allow_uncalibrated=true;c.init_mode=IMU_INIT_QUATERNION;
    if(!imu_init(&imu,&c,NULL))return fail(error,1,"estimator initialization failed");
    fprintf(output,"timestamp_ns,qw,qx,qy,qz,accel_weight,fusion_mode,health_flags,status\n");
    while(fgets(line,sizeof line,input)) {
        char *fields[19],*cursor=line;unsigned j;imu_sample_t s={0};imu_output_t out;
        uint64_t integers[19]={0};float values[19]={0};imu_status_t status;
        ++n;
        if(!strchr(line,'\n') && !feof(input))return fail(error,n,"row exceeds 2047 bytes");
        line[strcspn(line,"\r\n")]=0;
        for(j=0;j<19;++j) {
            char *comma;fields[j]=cursor;comma=strchr(cursor,',');
            if(j<18 && !comma)return fail(error,n,"missing column");
            if(j==18 && comma)return fail(error,n,"extra column");
            if(comma){*comma=0;cursor=comma+1;}
        }
        for(j=0;j<19;++j) {
            bool integer=j==0 || (j>=7 && j<=12) || j>=17;
            if(integer) {
                uint64_t limit=j==0?UINT64_MAX:((j>=10 && j<=12)||j==18?1:UINT32_MAX);
                if(!unsigned_number(fields[j],limit,&integers[j]))return fail(error,n,"invalid integer/status/freshness");
            } else if(!number(fields[j],&values[j]))return fail(error,n,"malformed or nonfinite numeric input");
        }
        s.timestamp_ns=integers[0];
        if(rows && s.timestamp_ns<=last)return fail(error,n,"nonmonotonic timestamp");
        for(j=0;j<3;++j) {
            s.gyro_rad_s[j]=values[1+j];s.gyro_temperature_c[j]=values[4+j];
            s.gyro_status[j]=(uint32_t)integers[7+j];s.gyro_sample_fresh[j]=integers[10+j]!=0;
            s.accel_m_s2[j]=values[13+j];
        }
        s.accel_temperature_c=values[16];s.accel_status=(uint32_t)integers[17];s.accel_sample_fresh=integers[18]!=0;
        status=imu_process(&imu,&s,&out);
        if(status==IMU_STATUS_BAD_ARGUMENT || status==IMU_STATUS_NONFINITE || status==IMU_STATUS_TIME_REVERSED || status==IMU_STATUS_NUMERICAL_FAULT)
            return fail(error,n,"estimator rejected input (timing or numerical failure)");
        fprintf(output,"%llu,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%lu,%u\n",
            (unsigned long long)out.timestamp_ns,out.q_nb.w,out.q_nb.x,out.q_nb.y,out.q_nb.z,
            out.accel_weight,(unsigned)out.fusion_mode,(unsigned long)out.health_flags,(unsigned)out.status);
        last=s.timestamp_ns;++rows;
    }
    if(ferror(input) || ferror(output))return fail(error,n,"I/O error");
    if(!rows)return fail(error,2,"missing data rows");
    return 0;
}
#ifdef IMU_REPLAY_TEST
#include "test_framework.h"
static void fixture(const char *header,const char *rows,int expected,const char *diagnostic)
{
    FILE *in=tmpfile(),*out=tmpfile(),*err=tmpfile(); char text[2048]={0};
    TEST_ASSERT(in && out && err); if(!in || !out || !err)return;
    fprintf(in,"%s\n%s",header,rows);rewind(in);
    TEST_ASSERT(replay(in,out,err)==expected); rewind(err);
    (void)fread(text,1,sizeof text-1,err);
    if(diagnostic)TEST_ASSERT(strstr(text,diagnostic)!=NULL);
    else {
        rewind(out); memset(text,0,sizeof text);(void)fread(text,1,sizeof text-1,out);
        TEST_ASSERT(strstr(text,"timestamp_ns,qw,qx,qy,qz,accel_weight,fusion_mode,health_flags,status")!=NULL);
        TEST_ASSERT(strstr(text,"1000000000,1,0,0,0,")!=NULL);
        TEST_ASSERT(strstr(text,"1002061855,")!=NULL);
    }
    fclose(in);fclose(out);fclose(err);
}
int main(void)
{
    const char *valid="1000000000,0,0,0,25,25,25,1,1,1,1,1,1,0,0,-9.80665,25,1,1\n1002061855,0,0,0,25,25,25,1,1,1,1,1,1,0,0,-9.80665,25,1,1\n";
    fixture(schema,valid,0,NULL);
    fixture("timestamp_ns,gx",valid,2,"line 1:");
    fixture(schema,"1000000000,no,0,0,25,25,25,1,1,1,1,1,1,0,0,-9.80665,25,1,1\n",2,"line 2:");
    fixture(schema,"1000000000,nan,0,0,25,25,25,1,1,1,1,1,1,0,0,-9.80665,25,1,1\n",2,"line 2:");
    fixture(schema,"1000000000,inf,0,0,25,25,25,1,1,1,1,1,1,0,0,-9.80665,25,1,1\n",2,"line 2:");
    fixture(schema,"1000000000,1x,0,0,25,25,25,1,1,1,1,1,1,0,0,-9.80665,25,1,1\n",2,"line 2:");
    fixture(schema,"1000000000,0,0\n",2,"line 2:");
    fixture(schema,"1000000000,0,0,0,25,25,25,1,1,1,2,1,1,0,0,-9.80665,25,1,1\n",2,"line 2:");
    fixture(schema,"18446744073709551616,0,0,0,25,25,25,1,1,1,1,1,1,0,0,-9.80665,25,1,1\n",2,"line 2:");
    fixture(schema,"1000000000,0,0,0,25,25,25,1,1,1,1,1,1,0,0,-9.80665,25,1,1\n1000000000,0,0,0,25,25,25,1,1,1,1,1,1,0,0,-9.80665,25,1,1\n",2,"line 3:");
    fixture(schema,"1000000000,0,0,0,25,25,25,1,1,1,1,1,1,0,0,-9.80665,25,1,1,\n",2,"line 2:");
    fixture(schema,"",2,"line 2:");
    return test_summary();
}
#else
int main(int argc,char **argv)
{
    FILE *in,*out;int result;
    if(argc!=3){fprintf(stderr,"usage: imu_replay input.csv output.csv\n");return 2;}
    if(strcmp(argv[1],argv[2])==0){fprintf(stderr,"input and output must differ\n");return 2;}
    in=fopen(argv[1],"r");if(!in){perror(argv[1]);return 2;}
    out=fopen(argv[2],"w");if(!out){perror(argv[2]);fclose(in);return 2;}
    result=replay(in,out,stderr);fclose(in);if(fclose(out)!=0)result=2;
    return result;
}
#endif
