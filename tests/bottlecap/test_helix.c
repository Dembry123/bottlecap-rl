/* Helical cap constraint unit tests.
 *   cc -std=c11 -O2 -I ocean/bottlecap -o /tmp/test_helix \
 *      tests/bottlecap/test_helix.c -lm && /tmp/test_helix
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "helix.h"

static int fails = 0;

static void expect_near(const char* name, float got, float exp, float tol) {
    if (fabsf(got - exp) > tol) {
        printf("FAIL %s: got %.8f expected %.8f\n", name, got, exp);
        fails++;
    } else printf("ok   %s: %.6f\n", name, got);
}

static void expect_true(const char* name, int cond) {
    if (!cond) { printf("FAIL %s\n", name); fails++; }
    else printf("ok   %s\n", name);
}

int main(void) {
    HelixParams p;
    HelixState s;
    memset(&p, 0, sizeof(p));
    p.start_height = 0.20f;
    p.pitch = 0.004f;
    p.direction = 1;
    p.max_turns = 2.0f;
    p.breakaway_torque = 0.10f;
    p.running_torque = 0.04f;
    p.max_axial_force = 30.0f;
    p.radial_tol = 0.01f;
    p.bottle_x = 0.0f;
    p.bottle_y = 0.0f;
    p.bottle_radius = 0.015f;

    helix_reset(&p, &s);
    expect_near("reset axial", s.axial, 0.20f, 1e-6f);
    expect_true("reset attached", !s.detached);
    expect_true("reset not broken", !s.broken_away);

    expect_true("no move low torque",
                !helix_apply(&p, &s, 1.0f/60.0f, 0.05f, 0.0f, 0.0f, 1));
    expect_near("still seated", s.angle, 0.0f, 1e-6f);

    expect_true("breakaway",
                helix_apply(&p, &s, 1.0f/60.0f, 0.20f, 0.0f, 0.0f, 1));
    expect_true("now broken", s.broken_away);
    expect_true("a1>0", s.angle > 0.0f);
    float expected_axial = p.start_height + p.pitch * s.angle / (2.0f * (float)M_PI);
    expect_near("helix axial", s.axial, expected_axial, 1e-5f);

    float a_before = s.angle;
    expect_true("radial block",
                !helix_apply(&p, &s, 1.0f/60.0f, 0.20f, 0.0f, 0.05f, 1));
    expect_near("angle unchanged", s.angle, a_before, 1e-6f);

    for (int i = 0; i < 20000 && !s.detached; i++) {
        helix_apply(&p, &s, 1.0f/60.0f, 0.25f, 0.0f, 0.0f, 1);
    }
    expect_true("detached after turns", s.detached);
    expect_near("max turns", fabsf(s.angle) / (2.0f * (float)M_PI), p.max_turns, 1e-3f);
    expect_near("final axial", s.axial,
                p.start_height + p.pitch * p.max_turns, 1e-3f);

    helix_reset(&p, &s);
    expect_true("force detach",
                !helix_apply(&p, &s, 1.0f/60.0f, 0.20f, 50.0f, 0.0f, 1));
    expect_true("detached by force", s.detached);

    if (fails) { printf("%d FAILURES\n", fails); return 1; }
    printf("ALL PASS\n");
    return 0;
}
