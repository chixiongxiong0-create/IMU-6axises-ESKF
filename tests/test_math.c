#include "test_framework.h"
#include "imu/imu_math.h"

#include <string.h>
#include <float.h>

/* Catches left-multiplying body rates, invalid-input mutation, and alias bugs. */
static void test_guarded_math(void)
{
    imu_quatf_t q = {0.70710678f, 0.0f, 0.0f, 0.70710678f};
    imu_quatf_t before = q;
    imu_vec3f_t out = {3.0f, 4.0f, 5.0f};
    const imu_vec3f_t saved = out;
    float a[3][3] = {{1, 2, 3}, {0, 1, 4}, {5, 6, 0}};
    float b[3][3] = {{-2, 1, 2}, {3, 0, -1}, {4, 5, 0}};
    const float expected[3][3] = {{16, 16, 0}, {19, 20, -1}, {8, 5, 4}};
    float saved_matrix[3][3];
    float six[6][6];
    float rhs[6][6];
    float saved_six[6][6];
    unsigned i, j;

    TEST_ASSERT(imu_quat_integrate(&q, (imu_vec3f_t){1, 0, 0}, 1.57079633f));
    TEST_ASSERT_NEAR(q.w, 0.5f, 2e-6f);
    TEST_ASSERT_NEAR(q.x, 0.5f, 2e-6f);
    TEST_ASSERT_NEAR(q.y, 0.5f, 2e-6f);
    TEST_ASSERT_NEAR(q.z, 0.5f, 2e-6f);
    q = before;
    TEST_ASSERT(!imu_quat_integrate(&q, (imu_vec3f_t){NAN, 0, 0}, 1));
    TEST_ASSERT(!imu_quat_integrate(&q, (imu_vec3f_t){1, 0, 0}, INFINITY));
    TEST_ASSERT(!imu_quat_integrate(&q, (imu_vec3f_t){1, 0, 0}, 0));
    TEST_ASSERT(memcmp(&q, &before, sizeof(q)) == 0);
    TEST_ASSERT(!imu_quat_rotate((imu_quatf_t){0, 0, 0, 0}, out, &out));
    TEST_ASSERT(!imu_quat_rotate(q, (imu_vec3f_t){INFINITY, 0, 0}, &out));
    TEST_ASSERT(memcmp(&out, &saved, sizeof(out)) == 0);
    q = (imu_quatf_t){1e-13f, 0, 0, 0};
    before = q;
    TEST_ASSERT(!imu_quat_normalize(&q));
    TEST_ASSERT(memcmp(&q, &before, sizeof(q)) == 0);
    q = (imu_quatf_t){2, 0, 0, 0};
    TEST_ASSERT(imu_quat_normalize(&q));
    TEST_ASSERT_NEAR(q.w, 1, 0);
    q.x = NAN;
    TEST_ASSERT(!imu_quat_normalize(&q));
    TEST_ASSERT(isnan(q.x));
    TEST_ASSERT(!imu_quat_normalize(NULL));
    TEST_ASSERT(!imu_quat_integrate(NULL, out, 1));
    TEST_ASSERT(!imu_quat_rotate(q, out, NULL));
    TEST_ASSERT(isnan(imu_quat_yaw_ned((imu_quatf_t){0, 0, 0, 0})));

    TEST_ASSERT(imu_mat3_multiply(a, b, a));
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j)
            TEST_ASSERT_NEAR(a[i][j], expected[i][j], 0);
    memcpy(saved_matrix, a, sizeof(a));
    b[2][2] = NAN;
    TEST_ASSERT(!imu_mat3_multiply(a, b, a));
    TEST_ASSERT(memcmp(a, saved_matrix, sizeof(a)) == 0);
    b[2][2] = FLT_MAX;
    TEST_ASSERT(!imu_mat3_multiply(a, b, a));
    TEST_ASSERT(memcmp(a, saved_matrix, sizeof(a)) == 0);
    imu_mat6_identity(six);
    imu_mat6_zero(rhs);
    rhs[0][5] = 7;
    rhs[5][0] = 3;
    TEST_ASSERT(imu_mat6_multiply(six, rhs, six));
    TEST_ASSERT_NEAR(six[0][5], 7, 0);
    TEST_ASSERT_NEAR(six[5][0], 3, 0);
    imu_mat6_transpose(six, six);
    TEST_ASSERT_NEAR(six[0][5], 3, 0);
    TEST_ASSERT_NEAR(six[5][0], 7, 0);
    imu_mat6_symmetrize(six);
    TEST_ASSERT_NEAR(six[0][5], 5, 0);
    TEST_ASSERT_NEAR(six[5][0], 5, 0);
    memcpy(saved_six, six, sizeof(six));
    rhs[3][4] = NAN;
    TEST_ASSERT(!imu_mat6_multiply(six, rhs, six));
    TEST_ASSERT(memcmp(saved_six, six, sizeof(six)) == 0);
}

int main(void)
{
    imu_quatf_t q = {1.0f, 0.0f, 0.0f, 0.0f};
    const imu_vec3f_t forward = {1.0f, 0.0f, 0.0f};
    const imu_vec3f_t right = {0.0f, 1.0f, 0.0f};
    const imu_quatf_t yaw_90 = {0.70710678f, 0.0f, 0.0f, 0.70710678f};
    imu_vec3f_t rotated = {0};
    imu_vec3f_t inverse = {0};
    imu_vec3f_t euler = {0};
    float a[3][3] = {{1.0f, 2.0f, 3.0f}, {0.0f, 1.0f, 4.0f},
                     {5.0f, 6.0f, 0.0f}};
    float b[3][3] = {{-2.0f, 1.0f, 2.0f}, {3.0f, 0.0f, -1.0f},
                     {4.0f, 5.0f, 0.0f}};
    float product[3][3] = {{0}};
    float p[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM] = {{0}};
    float pt[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM] = {{0}};
    float identity[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM] = {{0}};
    const imu_vec3f_t rate = {0.0f, 0.0f, 5.23598776f};
    const imu_quatf_t invalid = {0.0f, 0.0f, 0.0f, 0.0f};
    imu_quatf_t unchanged = invalid;

    TEST_ASSERT_VEC3_NEAR(imu_vec3_cross(forward, right),
                          ((imu_vec3f_t){0.0f, 0.0f, 1.0f}), 0.0f);
    TEST_ASSERT_NEAR(imu_vec3_dot(forward, right), 0.0f, 0.0f);
    TEST_ASSERT_NEAR(imu_vec3_norm((imu_vec3f_t){3.0f, 4.0f, 0.0f}), 5.0f, 1e-6f);
    TEST_ASSERT(imu_mat3_multiply(a, b, product));
    TEST_ASSERT_NEAR(product[0][0], 16.0f, 1e-6f);
    TEST_ASSERT_NEAR(product[1][2], -1.0f, 1e-6f);
    imu_mat6_identity(identity);
    p[0][1] = 2.0f;
    imu_mat6_transpose(p, pt);
    TEST_ASSERT(imu_mat6_multiply(identity, pt, p));
    TEST_ASSERT_NEAR(p[1][0], 2.0f, 0.0f);
    imu_mat6_symmetrize(p);
    TEST_ASSERT_NEAR(p[0][1], p[1][0], 0.0f);

    TEST_ASSERT(imu_quat_rotate(q, forward, &rotated));
    TEST_ASSERT_VEC3_NEAR(rotated, forward, 0.0f);
    TEST_ASSERT(imu_quat_rotate(yaw_90, forward, &rotated));
    TEST_ASSERT_VEC3_NEAR(rotated, right, 2.0e-6f);
    TEST_ASSERT(imu_quat_rotate(imu_quat_conjugate(yaw_90), rotated, &inverse));
    TEST_ASSERT_VEC3_NEAR(inverse, forward, 2.0e-6f);
    TEST_ASSERT_NEAR(imu_quat_multiply(yaw_90,
                                      imu_quat_conjugate(yaw_90)).w,
                     1.0f, 2.0e-6f);
    TEST_ASSERT(imu_quat_to_euler_ned(yaw_90, &euler));
    TEST_ASSERT_VEC3_NEAR(euler,
                          ((imu_vec3f_t){0.0f, 0.0f, 1.57079633f}),
                          2.0e-6f);
    TEST_ASSERT(!imu_quat_to_euler_ned(invalid, &euler));

    TEST_ASSERT(imu_quat_integrate(&q, (imu_vec3f_t){1.0e-3f, 0.0f, 0.0f},
                                   1.0e-3f));
    TEST_ASSERT_NEAR(q.x, 5.0e-7f, 1.0e-9f);
    q = (imu_quatf_t){1.0f, 0.0f, 0.0f, 0.0f};
    TEST_ASSERT(imu_quat_integrate(&q, rate, 0.002061856f));
    TEST_ASSERT_NEAR(imu_quat_norm(q), 1.0f, 2.0e-6f);
    TEST_ASSERT_NEAR(imu_quat_yaw_ned(q), 0.0107958f, 2.0e-5f);
    TEST_ASSERT(!imu_quat_integrate(&unchanged, rate, -1.0f));
    TEST_ASSERT(memcmp(&unchanged, &invalid, sizeof(unchanged)) == 0);
    TEST_ASSERT(!imu_quat_normalize(&unchanged));
    TEST_ASSERT(memcmp(&unchanged, &invalid, sizeof(unchanged)) == 0);
    test_guarded_math();
    return test_summary();
}
