#include <math.h>

#include "unity.h"

#include "trajectory.h"

TEST_CASE("Quintic endpoints", "[traj]")
{
    traj_quintic_t tr;
    traj_init_quintic(&tr, 0.0f, 0, 0, 100.0f, 0, 0, 2.0f);

    float v;
    /* At t=0 */
    tr.t0_us = 0;
    float p0 = traj_sample(&tr, 1e-9f, &v);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, p0);

    /* Force completion */
    float p1 = traj_sample(&tr, 1.0f + 2.0f, &v);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 100.0f, p1);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, v);
    TEST_ASSERT_TRUE(traj_done(&tr, 1.0f + 2.0f));
}

TEST_CASE("S-curve reaches target", "[traj]")
{
    traj_scurve_t sc;
    TEST_ASSERT_TRUE(traj_init_scurve(&sc, 0.0f, 90.0f, 200.0f, 800.0f, 3200.0f));
    float v;
    float p_end = traj_scurve_sample(&sc, sc.duration_us * 1e-6f + 1.0f, &v);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 90.0f, p_end);
}
