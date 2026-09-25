# Bottle-cap unscrewing task

## Scope (this milestone)

CPU-first Gradient0 environment with **state observations only**.
Curriculum **stage 1**: gripper already closed on a seated cap; learn wrist
rotation + axial following along a helical constraint until the cap detaches.

Out of scope here: FoundationPose / vision, GradientOS runtime coupling, CUDA
batched physics, meshed threads.

## Control contract

| Item | Value |
| --- | --- |
| Rate | 60 Hz (`BC_DT = 1/60`) |
| Actions | 7 continuous in [-1, 1]: J1..J6 velocity cmds + gripper velocity |
| Arm velocity scale | `max_joint_vel` (default 1.5 rad/s) |
| Gripper velocity scale | `BC_MAX_GRIP_VEL` (2.0 rad/s drive) |
| Joint limits | Controller soft limits (see `resources/gradient0/MODEL.md`) |
| Gripper | Drive angle [0, 165°]; width = 2 × 0.014 × θ |

## Observation contract (`BC_OBS_SIZE = 32`)

| Index | Content |
| --- | --- |
| 0:6 | joint positions / π |
| 6:12 | joint velocities / `BC_MAX_JOINT_VEL` |
| 12:15 | gripper_tip xyz (m) |
| 15:21 | wrist rotation columns 0 and 1 (6 floats) |
| 21 | gripper width / max width |
| 22 | gripper velocity / `BC_MAX_GRIP_VEL` |
| 23 | cap angle / (2π · max_turns) |
| 24 | cap axial height (m) |
| 25 | cap detached (0/1) |
| 26 | holding (0/1) |
| 27:30 | tip − (bottle_x, bottle_y, cap_axial) |
| 30:32 | reserved zeros |

## Helical cap model

While attached:

```text
axial = start_height + pitch * angle / (2π) * direction
```

Parameters (defaults in `bc_default_helix`):

- `pitch` = 3 mm/turn
- `direction` = +1 (positive angle raises the cap)
- `max_turns` = 2.5
- `breakaway_torque` / `running_torque`
- `max_axial_force`, `radial_tol`
- Bottle fixed to table; axis placed under the stage-1 tip at reset

Detachment when |turns| ≥ `max_turns`, or early if |axial_force| exceeds the limit.

## Reward terms (dense; terminal success = physical detach)

| Term | Weight (default) | Meaning |
| --- | ---: | --- |
| hold | +0.05 / −0.025 | maintaining grasp vs lost grasp |
| turn progress | +0.2 · turns/max_turns | progress along helix |
| axial follow | +0.05 · (1 − |z_err|/0.05) | tip tracks helix height |
| unscrew torque | +0.02 · max(τ·direction, 0) | torque in unscrew sense |
| time | −0.001 | small step cost |
| success | +5 | cap detached |
| drop | −2 | lost grasp before detach |
| table | −2 | tip below table |

Episode ends on success, drop, table collision, or `max_steps` (default 600).

## Curriculum

1. **Stage 1 (implemented):** gripper already holding seated cap; bottle snapped
   under tip after FK of a scripted reach pose.
2. **Stage 2 (TODO):** start above the cap; learn approach + grasp + unscrew.
3. **Stage 3 (TODO):** randomized bottle pose, friction, torque, delay, noise.

## How to run tests

On a machine with a working C compiler (Mac currently blocked by unsigned
Xcode license — use Linux/box or accept the license):

```bash
cc -std=c11 -O2 -I ocean/bottlecap -o /tmp/test_g0_fk   tests/bottlecap/test_gradient0_fk.c -lm && /tmp/test_g0_fk

cc -std=c11 -O2 -I ocean/bottlecap -o /tmp/test_helix   tests/bottlecap/test_helix.c -lm && /tmp/test_helix
```

Or: `bash tests/bottlecap/run_unit_tests.sh`

## Smoke train (after `./build.sh bottlecap` works)

```bash
./build.sh bottlecap
./puffer train --config config/bottlecap.ini --train.total_timesteps=100000
```

Learning is **not** expected to converge yet; this only checks the env wires
into PPO.

## Known gaps before "trainable-complete"

- [ ] Accept/fix Xcode license (or build on Linux) and compile `./build.sh bottlecap`
- [ ] Validate stage-1 tip pose against a trusted GradientOS FK sample (beyond URDF zeros)
- [ ] Gravity / table contact are scaffolding only (kinematic joint integration)
- [ ] Unscrew torque is a wrist-yaw proxy, not contact wrench from finger pads
- [ ] No MuJoCo cross-check yet
- [ ] No CUDA port
- [ ] Resolve J1 URDF ±π/2 vs controller ±π on hardware
- [ ] Gripper inertias / mesh redistribution clarification
