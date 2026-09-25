// Bottle-cap unscrewing environment (CPU, state observations).
// Gradient0 6-DOF arm + rack gripper; helical cap constraint; curriculum stage 1.
//
// Control rate: 60 Hz. Actions: 6 arm joint velocities + 1 gripper velocity.
// Observations: state only (no vision). See TASK.md and resources/gradient0/MODEL.md.
//
// Observation layout (BC_OBS_SIZE = 32):
//   [0:6]   joint positions / pi
//   [6:12]  joint velocities / BC_MAX_JOINT_VEL
//   [12:15] gripper_tip xyz (m)
//   [15:21] wrist rotation columns 0 and 1 (6 floats)
//   [21]    gripper width / max_width
//   [22]    gripper velocity / BC_MAX_GRIP_VEL
//   [23]    cap angle / (2*pi*max_turns)
//   [24]    cap axial height (m)
//   [25]    cap detached (0/1)
//   [26]    holding (0/1)
//   [27:30] tip - (bottle_x, bottle_y, cap_axial)
//   [30:32] reserved zeros

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "raylib.h"
typedef float obs_t;
#include "pufferenv.h"
#include "gradient0_model.h"
#include "helix.h"

#define BC_OBS_SIZE 32
#define ACT_SIZES {1, 1, 1, 1, 1, 1, 1}
#define OBS_SIZE BC_OBS_SIZE
#define NUM_ATNS G0_NACTIONS

#define BC_DT (1.0f / 60.0f)
#define BC_MAX_JOINT_VEL 1.5f
#define BC_MAX_GRIP_VEL 2.0f
#define BC_TABLE_Z 0.0f
#define BC_STAGE_HOLDING 1

typedef struct Log Log;
struct Log {
    float perf;
    float score;
    float episode_return;
    float episode_length;
    float success;
    float detached;
    float cap_turns;
    float max_steps_termination;
    float drop_termination;
    float table_termination;
    float n;
};

typedef struct Client Client;
struct Client {};

struct Env {
    Log log;
    Agent agents[1];
    int tag;
    int boundary_reached;
    int num_agents;
    Client* client;
    unsigned int rng;

    float q[G0_NJOINTS];
    float qd[G0_NJOINTS];
    float gripper;
    float gripper_vel;
    float tip_x, tip_y, tip_z;
    float tool_x, tool_y, tool_z;
    G0Frame wrist_frame;

    HelixParams helix_p;
    HelixState helix_s;
    int holding;
    int tick;
    int max_steps;
    float episode_return;
    int success;
    int curriculum_stage;

    float max_joint_vel;
    float table_z;
    float gravity;
    float grasp_width_max;
};
typedef Env Bottlecap;

static void bc_default_helix(HelixParams* p) {
    memset(p, 0, sizeof(*p));
    p->start_height = 0.18f;
    p->pitch = 0.003f;
    p->direction = 1;
    p->max_turns = 2.5f;
    p->breakaway_torque = 0.15f;
    p->running_torque = 0.05f;
    p->max_axial_force = 40.0f;
    p->radial_tol = 0.015f;
    p->bottle_x = 0.28f;
    p->bottle_y = 0.0f;
    p->bottle_radius = 0.015f;
}

static void bc_stage1_joints(float q[G0_NJOINTS]) {
    q[0] = 0.0f;
    q[1] = -0.55f;
    q[2] = 1.05f;
    q[3] = 0.0f;
    q[4] = 0.55f;
    q[5] = 0.0f;
}

static void bc_update_fk(Bottlecap* env) {
    G0Vec3 wrist, tool, tip;
    g0_fk(env->q, NULL, &wrist, &tool, &tip, &env->wrist_frame);
    env->tool_x = tool.x; env->tool_y = tool.y; env->tool_z = tool.z;
    env->tip_x = tip.x; env->tip_y = tip.y; env->tip_z = tip.z;
}

static float bc_radial_error(const Bottlecap* env) {
    float dx = env->tip_x - env->helix_p.bottle_x;
    float dy = env->tip_y - env->helix_p.bottle_y;
    return sqrtf(dx * dx + dy * dy);
}

static int bc_grasp_ok(const Bottlecap* env) {
    float width = g0_gripper_width(env->gripper);
    if (width > env->grasp_width_max) return 0;
    if (bc_radial_error(env) > env->helix_p.radial_tol) return 0;
    float dz = fabsf(env->tip_z - env->helix_s.axial);
    if (dz > 0.025f) return 0;
    return 1;
}

static float bc_wrist_yaw(const Bottlecap* env) {
    return atan2f(env->wrist_frame.m[1][0], env->wrist_frame.m[0][0]);
}

static float bc_estimate_unscrew_torque(Bottlecap* env, float prev_yaw, float dt) {
    float yaw = bc_wrist_yaw(env);
    float dyaw = yaw - prev_yaw;
    while (dyaw > (float)M_PI) dyaw -= 2.0f * (float)M_PI;
    while (dyaw < -(float)M_PI) dyaw += 2.0f * (float)M_PI;
    float omega = dyaw / fmaxf(dt, 1e-6f);
    return 0.08f * omega;
}

static void compute_observations(Bottlecap* env) {
    float* o = env->agents[0].observations;
    int i = 0;
    for (int j = 0; j < G0_NJOINTS; j++) o[i++] = env->q[j] / (float)M_PI;
    for (int j = 0; j < G0_NJOINTS; j++) o[i++] = env->qd[j] / BC_MAX_JOINT_VEL;
    o[i++] = env->tip_x;
    o[i++] = env->tip_y;
    o[i++] = env->tip_z;
    o[i++] = env->wrist_frame.m[0][0];
    o[i++] = env->wrist_frame.m[1][0];
    o[i++] = env->wrist_frame.m[2][0];
    o[i++] = env->wrist_frame.m[0][1];
    o[i++] = env->wrist_frame.m[1][1];
    o[i++] = env->wrist_frame.m[2][1];
    o[i++] = g0_gripper_width(env->gripper) / g0_gripper_width(G0_GRIPPER_OPEN_RAD);
    o[i++] = env->gripper_vel / BC_MAX_GRIP_VEL;
    o[i++] = env->helix_s.angle / (2.0f * (float)M_PI * env->helix_p.max_turns);
    o[i++] = env->helix_s.axial;
    o[i++] = env->helix_s.detached ? 1.0f : 0.0f;
    o[i++] = env->holding ? 1.0f : 0.0f;
    o[i++] = env->tip_x - env->helix_p.bottle_x;
    o[i++] = env->tip_y - env->helix_p.bottle_y;
    o[i++] = env->tip_z - env->helix_s.axial;
    while (i < BC_OBS_SIZE) o[i++] = 0.0f;
}

static void add_log(Bottlecap* env, int success, int timeout, int drop, int table) {
    float turns = fabsf(env->helix_s.angle) / (2.0f * (float)M_PI);
    env->log.perf += success ? 1.0f : fminf(turns / env->helix_p.max_turns, 1.0f);
    env->log.score += env->episode_return;
    env->log.episode_return += env->episode_return;
    env->log.episode_length += (float)env->tick;
    env->log.success += success ? 1.0f : 0.0f;
    env->log.detached += env->helix_s.detached ? 1.0f : 0.0f;
    env->log.cap_turns += turns;
    env->log.max_steps_termination += timeout ? 1.0f : 0.0f;
    env->log.drop_termination += drop ? 1.0f : 0.0f;
    env->log.table_termination += table ? 1.0f : 0.0f;
    env->log.n += 1.0f;
}

void init(Bottlecap* env) {
    env->num_agents = 1;
    if (env->max_steps <= 0) env->max_steps = 600;
    if (env->max_joint_vel <= 0.0f) env->max_joint_vel = BC_MAX_JOINT_VEL;
    if (env->grasp_width_max <= 0.0f) env->grasp_width_max = 0.035f;
    if (env->curriculum_stage <= 0) env->curriculum_stage = BC_STAGE_HOLDING;
    bc_default_helix(&env->helix_p);
}

void puf_close(Bottlecap* env) {
    (void)env;
}

const Color PUFF_RED = (Color){187, 0, 0, 255};
const Color PUFF_CYAN = (Color){0, 187, 187, 255};
const Color PUFF_WHITE = (Color){241, 241, 241, 255};
const Color PUFF_BACKGROUND = (Color){6, 24, 24, 255};
const Color PUFF_YELLOW = (Color){245, 197, 66, 255};

Client* make_client(Bottlecap* env) {
    (void)env;
    Client* client = (Client*)calloc(1, sizeof(Client));
    InitWindow(800, 600, "puffer bottlecap");
    SetTargetFPS(60);
    return client;
}

void close_client(Client* client) {
    CloseWindow();
    free(client);
}

void puf_render(Bottlecap* env) {
    if (IsKeyDown(KEY_ESCAPE)) exit(0);
    if (env->client == NULL) env->client = make_client(env);
    BeginDrawing();
    ClearBackground(PUFF_BACKGROUND);
    DrawText(TextFormat("tick=%d grip=%.1fdeg hold=%d", env->tick,
                        env->gripper * 180.0f / (float)M_PI, env->holding),
             10, 10, 18, PUFF_WHITE);
    DrawText(TextFormat("tip=(%.3f,%.3f,%.3f)", env->tip_x, env->tip_y, env->tip_z),
             10, 36, 18, PUFF_CYAN);
    DrawText(TextFormat("cap turns=%.2f axial=%.3f detached=%d",
                        env->helix_s.angle / (2.0f * (float)M_PI),
                        env->helix_s.axial, env->helix_s.detached),
             10, 62, 18, PUFF_YELLOW);
    DrawText(TextFormat("return=%.2f success=%d", env->episode_return, env->success),
             10, 88, 18, PUFF_RED);
    float sx = 400.0f, sz = 500.0f, sc = 800.0f;
    DrawLine(0, (int)sz, 800, (int)sz, PUFF_CYAN);
    DrawCircle((int)(sx + env->helix_p.bottle_x * sc),
               (int)(sz - env->helix_p.start_height * sc), 8, PUFF_YELLOW);
    DrawCircle((int)(sx + env->tip_x * sc), (int)(sz - env->tip_z * sc), 6, PUFF_RED);
    EndDrawing();
    puf_web_vsync();
}

void puf_reset(Bottlecap* env) {
    env->episode_return = 0.0f;
    env->tick = 0;
    env->success = 0;
    memset(env->qd, 0, sizeof(env->qd));
    env->gripper_vel = 0.0f;

    if (env->curriculum_stage == BC_STAGE_HOLDING) {
        bc_stage1_joints(env->q);
        for (int j = 0; j < G0_NJOINTS; j++)
            env->q[j] = g0_clamp_joint(j, env->q[j]);
        env->gripper = g0_gripper_from_width(0.028f);
        bc_update_fk(env);
        env->helix_p.bottle_x = env->tip_x;
        env->helix_p.bottle_y = env->tip_y;
        env->helix_p.start_height = env->tip_z;
        helix_reset(&env->helix_p, &env->helix_s);
        env->holding = 1;
    } else {
        memset(env->q, 0, sizeof(env->q));
        env->gripper = G0_GRIPPER_OPEN_RAD * 0.5f;
        bc_default_helix(&env->helix_p);
        helix_reset(&env->helix_p, &env->helix_s);
        bc_update_fk(env);
        env->holding = 0;
    }
    compute_observations(env);
}

void puf_step(Bottlecap* env) {
    float* a = env->agents[0].actions;
    float prev_yaw = bc_wrist_yaw(env);

    for (int j = 0; j < G0_NJOINTS; j++) {
        float u = g0_clampf(a[j], -1.0f, 1.0f);
        env->qd[j] = u * env->max_joint_vel;
        env->q[j] = g0_clamp_joint(j, env->q[j] + env->qd[j] * BC_DT);
    }
    float ug = g0_clampf(a[6], -1.0f, 1.0f);
    env->gripper_vel = ug * BC_MAX_GRIP_VEL;
    env->gripper = g0_clamp_gripper(env->gripper + env->gripper_vel * BC_DT);

    bc_update_fk(env);

    int table_hit = (env->tip_z < env->table_z + 0.01f) ? 1 : 0;
    env->holding = bc_grasp_ok(env) ? 1 : 0;

    float torque = 0.0f;
    float axial_f = 0.0f;
    if (env->holding && !env->helix_s.detached) {
        torque = bc_estimate_unscrew_torque(env, prev_yaw, BC_DT);
        axial_f = 20.0f * (env->tip_z - env->helix_s.axial);
        helix_apply(&env->helix_p, &env->helix_s, BC_DT,
                    torque, axial_f, bc_radial_error(env), 1);
    }

    float reward = 0.0f;
    float turns = fabsf(env->helix_s.angle) / (2.0f * (float)M_PI);
    reward += 0.05f * (env->holding ? 1.0f : -0.5f);
    reward += 0.2f * fminf(turns / env->helix_p.max_turns, 1.0f);
    if (env->holding) {
        float z_err = fabsf(env->tip_z - env->helix_s.axial);
        reward += 0.05f * (1.0f - fminf(z_err / 0.05f, 1.0f));
        reward += 0.02f * fmaxf(torque * (float)env->helix_p.direction, 0.0f);
    }
    reward -= 0.001f;

    int drop = (!env->holding && !env->helix_s.detached && env->tick > 5) ? 1 : 0;
    int success = env->helix_s.detached ? 1 : 0;
    env->success = success;
    env->tick += 1;
    int timeout = env->tick >= env->max_steps;

    if (success) reward += 5.0f;
    if (drop) reward -= 2.0f;
    if (table_hit) reward -= 2.0f;

    env->agents[0].rewards[0] = reward;
    env->episode_return += reward;
    int terminated = success || drop || table_hit;
    env->agents[0].terminals[0] = terminated ? 1 : 0;

    if (terminated || timeout) {
        add_log(env, success, timeout, drop, table_hit);
        puf_reset(env);
    } else {
        compute_observations(env);
    }
}

void puf_log(Log* log, Dict* out) {
    dict_set(out, "score", log->score);
    dict_set(out, "perf", log->perf);
    dict_set(out, "episode_return", log->episode_return);
    dict_set(out, "episode_length", log->episode_length);
    dict_set(out, "success", log->success);
    dict_set(out, "detached", log->detached);
    dict_set(out, "cap_turns", log->cap_turns);
    dict_set(out, "max_steps_termination", log->max_steps_termination);
    dict_set(out, "drop_termination", log->drop_termination);
    dict_set(out, "table_termination", log->table_termination);
    dict_set(out, "n", log->n);
}

void puf_init(Env* env, Dict* kwargs) {
    env->num_agents = 1;
    env->max_steps = (int)dict_get(kwargs, "max_steps");
    if (env->max_steps <= 0) env->max_steps = 600;
    env->max_joint_vel = (float)dict_get(kwargs, "max_joint_vel");
    if (env->max_joint_vel <= 0.0f) env->max_joint_vel = BC_MAX_JOINT_VEL;
    env->table_z = (float)dict_get(kwargs, "table_z");
    env->gravity = (float)dict_get(kwargs, "gravity");
    if (env->gravity == 0.0f) env->gravity = 9.81f;
    env->grasp_width_max = (float)dict_get(kwargs, "grasp_width_max");
    if (env->grasp_width_max <= 0.0f) env->grasp_width_max = 0.035f;
    env->curriculum_stage = (int)dict_get(kwargs, "curriculum_stage");
    if (env->curriculum_stage <= 0) env->curriculum_stage = BC_STAGE_HOLDING;
    env->agents[0].action_mask = NULL;
    env->agents[0].policy = 0;
    init(env);
}
