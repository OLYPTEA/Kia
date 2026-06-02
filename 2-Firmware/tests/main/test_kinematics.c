#include <math.h>

#include "unity.h"

#include "dom_joint.h"
#include "kinematics.h"

static const float EPS_MM  = 0.5f;
static const float EPS_DEG = 0.5f;

TEST_CASE("FK home pose", "[kin]")
{
    dom_joint_init();
    kin_set_geometry(60, 105, 100, 85);
    kin_joints_t j = {0, 90, 0, 0};
    kin_pose_t   p;
    TEST_ASSERT_EQUAL(ESP_OK, kin_fk(&j, &p));
    /* For shoulder=90°, elbow=0, wrist=0: arm extended horizontally along +X. */
    TEST_ASSERT_FLOAT_WITHIN(EPS_MM, 105 + 100 + 85, p.x_mm);
    TEST_ASSERT_FLOAT_WITHIN(EPS_MM, 0,              p.y_mm);
    TEST_ASSERT_FLOAT_WITHIN(EPS_MM, 60,             p.z_mm);
}

TEST_CASE("FK -> IK roundtrip elbow up", "[kin]")
{
    dom_joint_init();
    kin_set_geometry(60, 105, 100, 85);
    kin_joints_t in = {15.0f, 60.0f, 45.0f, -20.0f};
    kin_pose_t   p;
    TEST_ASSERT_EQUAL(ESP_OK, kin_fk(&in, &p));

    kin_joints_t out;
    esp_err_t e = kin_ik(&p, KIN_ELBOW_DOWN, &out);
    if (e != ESP_OK) e = kin_ik(&p, KIN_ELBOW_UP, &out);
    TEST_ASSERT_EQUAL(ESP_OK, e);

    kin_pose_t p2;
    TEST_ASSERT_EQUAL(ESP_OK, kin_fk(&out, &p2));
    TEST_ASSERT_FLOAT_WITHIN(EPS_MM, p.x_mm, p2.x_mm);
    TEST_ASSERT_FLOAT_WITHIN(EPS_MM, p.y_mm, p2.y_mm);
    TEST_ASSERT_FLOAT_WITHIN(EPS_MM, p.z_mm, p2.z_mm);
}

TEST_CASE("IK unreachable far", "[kin]")
{
    dom_joint_init();
    kin_set_geometry(60, 105, 100, 85);
    kin_pose_t p = {500, 0, 60, 0};
    kin_joints_t out;
    esp_err_t e = kin_ik(&p, KIN_ELBOW_UP, &out);
    TEST_ASSERT_EQUAL(KIA_ERR_UNREACHABLE, e);
}

TEST_CASE("Joint clamp", "[kin]")
{
    dom_joint_init();
    TEST_ASSERT_TRUE(dom_joint_in_limits(0, 0));
    TEST_ASSERT_FALSE(dom_joint_in_limits(0, 200));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90, dom_joint_clamp(0, 200));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -90, dom_joint_clamp(0, -200));
}
