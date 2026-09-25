// Helical bottle-cap constraint (curriculum stage 1).
//
// While attached:
//   axial = start_height + pitch * angle / (2*pi) * direction
// with thread direction, allowed turns, breakaway/running torque,
// axial-force limits, radial alignment tolerance, and a detached state.
// Bottle is fixed to the table. Not a meshed thread model.

#ifndef BOTTLECAP_HELIX_H
#define BOTTLECAP_HELIX_H

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    float start_height;     /* m, cap center Z when angle == 0 (seated) */
    float pitch;            /* m per full turn (positive magnitude) */
    int direction;          /* +1: +angle raises cap; -1 opposite */
    float max_turns;        /* turns until thread disengages */
    float breakaway_torque; /* N*m to start motion from rest */
    float running_torque;   /* N*m opposing motion once broken away */
    float max_axial_force;  /* N; |force| above this detaches early */
    float radial_tol;       /* m; grasp must stay within this of axis */
    float bottle_x;         /* fixed bottle axis (world Z-up) */
    float bottle_y;
    float bottle_radius;    /* m, nominal cap outer radius */
} HelixParams;

typedef struct {
    float angle;            /* rad, 0 = fully seated */
    float axial;            /* m, world Z of cap center */
    int broken_away;
    int detached;
    float last_torque;
    float last_axial_force;
} HelixState;

static inline void helix_axial_from_angle(const HelixParams* p, HelixState* s) {
    float turns = s->angle / (2.0f * (float)M_PI);
    s->axial = p->start_height + p->pitch * (float)p->direction * turns;
}

static inline void helix_reset(const HelixParams* p, HelixState* s) {
    s->angle = 0.0f;
    s->broken_away = 0;
    s->detached = 0;
    s->last_torque = 0.0f;
    s->last_axial_force = 0.0f;
    helix_axial_from_angle(p, s);
}

/*
 * Apply grasp wrench for one control step (dt seconds).
 * torque_nm: signed about bottle +Z. With direction=+1, positive unscrews.
 * axial_force_n: signed world +Z force from gripper on cap.
 * radial_error_m: horizontal distance gripper center -> bottle axis.
 * grasped: nonzero if gripper holds the cap.
 * Returns 1 if the cap advanced this step.
 */
static inline int helix_apply(const HelixParams* p, HelixState* s, float dt,
                              float torque_nm, float axial_force_n,
                              float radial_error_m, int grasped) {
    s->last_torque = torque_nm;
    s->last_axial_force = axial_force_n;
    if (s->detached || !grasped) return 0;
    if (radial_error_m > p->radial_tol) return 0;

    float unscrew = torque_nm * (float)p->direction;
    float threshold = s->broken_away ? p->running_torque : p->breakaway_torque;
    if (unscrew <= threshold) return 0;

    const float I_cap = 2.0e-5f;
    float excess = unscrew - threshold;
    float omega = excess / I_cap;
    float max_omega = (2.0f * (float)M_PI * p->max_turns) / 0.5f;
    if (omega > max_omega) omega = max_omega;

    float d_angle = (float)p->direction * omega * dt;
    float d_axial = p->pitch * (float)p->direction
                  * (d_angle / (2.0f * (float)M_PI));

    if (d_axial > 0.0f && axial_force_n < -p->max_axial_force) return 0;
    if (d_axial < 0.0f && axial_force_n > p->max_axial_force) return 0;
    if (fabsf(axial_force_n) > p->max_axial_force) {
        s->detached = 1;
        return 0;
    }

    s->angle += d_angle;
    s->broken_away = 1;
    helix_axial_from_angle(p, s);

    float turns = fabsf(s->angle) / (2.0f * (float)M_PI);
    if (turns >= p->max_turns - 1e-4f) {
        float sign = (s->angle >= 0.0f) ? 1.0f : -1.0f;
        s->angle = sign * p->max_turns * 2.0f * (float)M_PI;
        helix_axial_from_angle(p, s);
        s->detached = 1;
    }
    return 1;
}

#endif /* BOTTLECAP_HELIX_H */
