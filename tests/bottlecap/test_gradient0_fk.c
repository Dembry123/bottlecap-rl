/* Unit tests for Gradient0 FK, limits, gripper.
 *   cc -std=c11 -O2 -I ocean/bottlecap -o /tmp/test_g0_fk \
 *      tests/bottlecap/test_gradient0_fk.c -lm && /tmp/test_g0_fk
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "gradient0_model.h"

static int fails = 0;

static void expect_near(const char* name, float got, float exp, float tol) {
    if (fabsf(got - exp) > tol) {
        printf("FAIL %s: got %.8f expected %.8f (tol %.8f)\n", name, got, exp, tol);
        fails++;
    } else {
        printf("ok   %s: %.6f\n", name, got);
    }
}

static void expect_true(const char* name, int cond) {
    if (!cond) { printf("FAIL %s\n", name); fails++; }
    else printf("ok   %s\n", name);
}

int main(void) {
    float q[G0_NJOINTS];
    memset(q, 0, sizeof(q));
    G0Vec3 wrist, tool, tip;
    G0Frame wf;

    g0_fk(q, NULL, &wrist, &tool, &tip, &wf);
    expect_near("zero wrist.x", wrist.x, 0.309006f, 1e-5f);
    expect_near("zero wrist.y", wrist.y, 0.0f, 1e-5f);
    expect_near("zero wrist.z", wrist.z, 0.3701f, 1e-5f);
    expect_near("zero tool.x", tool.x, 0.489006f, 1e-5f);
    expect_near("zero tip.x", tip.x, 0.45592109f, 1e-5f);
    expect_near("zero tip.z", tip.z, 0.3701f, 1e-5f);

    q[0] = 0.5f * (float)M_PI;
    g0_fk(q, NULL, &wrist, &tool, &tip, &wf);
    expect_near("j1_90 wrist.x", wrist.x, 0.0f, 1e-4f);
    expect_near("j1_90 wrist.y", wrist.y, 0.309006f, 1e-4f);
    expect_near("j1_90 wrist.z", wrist.z, 0.3701f, 1e-5f);

    expect_near("clamp j1 +4", g0_clamp_joint(0, 4.0f), (float)M_PI, 1e-6f);
    expect_near("clamp j1 -4", g0_clamp_joint(0, -4.0f), -(float)M_PI, 1e-6f);
    expect_near("clamp j2 +2", g0_clamp_joint(1, 2.0f), 0.5f * (float)M_PI, 1e-6f);
    expect_near("clamp j5 hi", g0_clamp_joint(4, 3.0f), 2.0944f, 1e-6f);
    expect_near("clamp j5 lo", g0_clamp_joint(4, -3.0f), -1.8326f, 1e-6f);

    expect_near("grip closed", g0_gripper_width(0.0f), 0.0f, 1e-7f);
    float open = g0_gripper_width(G0_GRIPPER_OPEN_RAD);
    expect_near("grip open max", open, 0.080634211f, 1e-5f);
    float mid_w = 0.04f;
    float th = g0_gripper_from_width(mid_w);
    expect_near("grip roundtrip", g0_gripper_width(th), mid_w, 1e-6f);
    expect_true("grip clamp above open",
                g0_clamp_gripper(10.0f) == G0_GRIPPER_OPEN_RAD);

    memset(q, 0, sizeof(q));
    q[1] = 30.0f * (float)M_PI / 180.0f;
    g0_fk(q, NULL, &wrist, &tool, &tip, &wf);
    expect_near("j2_30 tip.y", tip.y, 0.0f, 1e-5f);
    expect_near("j2_30 tip.x", tip.x, 0.51616425f, 1e-4f);
    expect_near("j2_30 tip.z", tip.z, 0.10963052f, 1e-4f);

    if (fails) {
        printf("%d FAILURES\n", fails);
        return 1;
    }
    printf("ALL PASS\n");
    return 0;
}
