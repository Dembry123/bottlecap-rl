// Gradient0 / Cheap Robot Arm kinematic model (CPU).
//
// Frames follow resources/gradient0/mini-6dof-arm.urdf (frozen snapshot).
// Soft limits follow the Gradient0 controller (not URDF J1).
// See resources/gradient0/MODEL.md for provenance and open issues.
//
// Convention: URDF Z-up, meters, radians.
// Logical DOF: 6 revolute arm joints + 1 gripper drive.

#ifndef BOTTLECAP_GRADIENT0_MODEL_H
#define BOTTLECAP_GRADIENT0_MODEL_H

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <string.h>

#define G0_NJOINTS 6
#define G0_NACTIONS 7

#define G0_GRIPPER_OPEN_RAD (165.0f * (float)M_PI / 180.0f)
#define G0_RACK_M_PER_RAD 0.014f
#define G0_GRIPPER_TIP_X 0.146915092086197f
#define G0_TOOL_X 0.180f

typedef struct {
    float x, y, z;
} G0Vec3;

typedef struct {
    float m[3][3]; /* row-major R */
    G0Vec3 p;
} G0Frame;

static inline float g0_clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline G0Vec3 g0_v3(float x, float y, float z) {
    G0Vec3 v = {x, y, z};
    return v;
}

static inline G0Frame g0_identity(void) {
    G0Frame f;
    memset(&f, 0, sizeof(f));
    f.m[0][0] = f.m[1][1] = f.m[2][2] = 1.0f;
    return f;
}

static inline G0Vec3 g0_rmul(const G0Frame* f, G0Vec3 v) {
    G0Vec3 o;
    o.x = f->m[0][0] * v.x + f->m[0][1] * v.y + f->m[0][2] * v.z;
    o.y = f->m[1][0] * v.x + f->m[1][1] * v.y + f->m[1][2] * v.z;
    o.z = f->m[2][0] * v.x + f->m[2][1] * v.y + f->m[2][2] * v.z;
    return o;
}

static inline G0Vec3 g0_xform(const G0Frame* f, G0Vec3 v) {
    G0Vec3 r = g0_rmul(f, v);
    r.x += f->p.x;
    r.y += f->p.y;
    r.z += f->p.z;
    return r;
}

/* A * B: apply B first. */
static inline G0Frame g0_mul(const G0Frame* a, const G0Frame* b) {
    G0Frame c;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            c.m[i][j] = a->m[i][0] * b->m[0][j]
                      + a->m[i][1] * b->m[1][j]
                      + a->m[i][2] * b->m[2][j];
        }
    }
    c.p = g0_xform(a, b->p);
    return c;
}

static inline G0Frame g0_trans(float x, float y, float z) {
    G0Frame f = g0_identity();
    f.p.x = x;
    f.p.y = y;
    f.p.z = z;
    return f;
}

static inline G0Frame g0_rotx(float a) {
    float c = cosf(a), s = sinf(a);
    G0Frame f = g0_identity();
    f.m[1][1] = c; f.m[1][2] = -s;
    f.m[2][1] = s; f.m[2][2] = c;
    return f;
}

static inline G0Frame g0_roty(float a) {
    float c = cosf(a), s = sinf(a);
    G0Frame f = g0_identity();
    f.m[0][0] = c;  f.m[0][2] = s;
    f.m[2][0] = -s; f.m[2][2] = c;
    return f;
}

static inline G0Frame g0_rotz(float a) {
    float c = cosf(a), s = sinf(a);
    G0Frame f = g0_identity();
    f.m[0][0] = c; f.m[0][1] = -s;
    f.m[1][0] = s; f.m[1][1] = c;
    return f;
}

/* Controller soft limits. OPEN ISSUE: URDF J1 is ±pi/2. */
static inline void g0_joint_limits(int joint, float* lo, float* hi) {
    static const float L[G0_NJOINTS] = {
        -(float)M_PI, -0.5f * (float)M_PI, -0.5f * (float)M_PI,
        -(float)M_PI, -1.8326f, -(float)M_PI
    };
    static const float H[G0_NJOINTS] = {
        (float)M_PI, 0.5f * (float)M_PI, 0.5f * (float)M_PI,
        (float)M_PI, 2.0944f, (float)M_PI
    };
    *lo = L[joint];
    *hi = H[joint];
}

static inline float g0_clamp_joint(int joint, float q) {
    float lo, hi;
    g0_joint_limits(joint, &lo, &hi);
    return g0_clampf(q, lo, hi);
}

static inline float g0_clamp_gripper(float theta) {
    return g0_clampf(theta, 0.0f, G0_GRIPPER_OPEN_RAD);
}

/* Opening width between opposing jaw faces (meters). Closed at 0. */
static inline float g0_gripper_width(float drive_rad) {
    return 2.0f * G0_RACK_M_PER_RAD * g0_clamp_gripper(drive_rad);
}

static inline float g0_gripper_from_width(float width_m) {
    float max_w = g0_gripper_width(G0_GRIPPER_OPEN_RAD);
    width_m = g0_clampf(width_m, 0.0f, max_w);
    return width_m / (2.0f * G0_RACK_M_PER_RAD);
}

/*
 * Forward kinematics from URDF joint origins.
 * q[6] radians. Optionally fills links[6] (child frame after each joint),
 * wrist/tool/tip positions, and wrist orientation frame.
 */
static inline void g0_fk(const float q[G0_NJOINTS],
                         G0Frame* links,
                         G0Vec3* wrist,
                         G0Vec3* tool,
                         G0Vec3* tip,
                         G0Frame* wrist_frame) {
    static const float ox[G0_NJOINTS] = {
        0.0f, 0.0f, 0.0f, 0.176556f, 0.05515f, 0.0773f
    };
    static const float oy[G0_NJOINTS] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };
    static const float oz[G0_NJOINTS] = {
        0.0843f, 0.04315f, 0.19715f, 0.0455f, 0.0f, 0.0f
    };
    /* axis: 0=x 1=y 2=z */
    static const int axis[G0_NJOINTS] = {2, 1, 1, 0, 1, 0};

    G0Frame T = g0_identity();
    for (int i = 0; i < G0_NJOINTS; i++) {
        G0Frame tr = g0_trans(ox[i], oy[i], oz[i]);
        G0Frame R = (axis[i] == 0) ? g0_rotx(q[i])
                  : (axis[i] == 1) ? g0_roty(q[i])
                                   : g0_rotz(q[i]);
        G0Frame step = g0_mul(&tr, &R);
        T = g0_mul(&T, &step);
        if (links) links[i] = T;
    }
    if (wrist) *wrist = T.p;
    if (wrist_frame) *wrist_frame = T;
    if (tool) *tool = g0_xform(&T, g0_v3(G0_TOOL_X, 0.0f, 0.0f));
    if (tip) *tip = g0_xform(&T, g0_v3(G0_GRIPPER_TIP_X, 0.0f, 0.0f));
}

#endif /* BOTTLECAP_GRADIENT0_MODEL_H */
