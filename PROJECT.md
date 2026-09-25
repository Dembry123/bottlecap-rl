# Bottlecap RL

This repository is a standalone fork of PufferAI/PufferLib 5.0 for training a
policy that can eventually unscrew a bottle cap with the Gradient0 / Cheap
Robot Arm. It deliberately does not depend on GradientOS. GradientOS remains
the hardware-control and eventual policy-inference system; this repository owns
simulation, training, evaluation, and policy export.

## Fork point

- Upstream: `https://github.com/PufferAI/PufferLib.git`
- Fork: `https://github.com/Dembry123/bottlecap-rl.git`
- Upstream branch: `5.0`
- Initial upstream commit: `6ffa5b10dbbbe4d1e8288367c7d9d3acd3bad4a2`
- Local development branch: `bottlecap-rl`

The local `origin` remote is the user-owned fork. The `upstream` remote remains
PufferAI/PufferLib so upstream 5.0 changes can be fetched without confusing the
two repositories.

## Current status

Gradient0 model snapshot is frozen under `resources/gradient0/` (URDF, meshes,
gripper, controller limits, `MODEL.md`). A CPU bottle-cap ocean env lives in
`ocean/bottlecap/` with URDF FK, controller limits, gripper width conversion,
helical cap constraint, and curriculum stage-1 (holding) task wiring. Unit
tests for FK/limits/gripper and the helix pass on Linux (`tests/bottlecap/`).
Full `./build.sh bottlecap` / PPO smoke train is not yet verified on this Mac
(Xcode license). No vision, CUDA port, or GradientOS runtime coupling yet.

The Franka example is reference code, not a parameter file. Its seven-joint
forward kinematics, mass and inertia tables, collision bodies, gripper,
observation layout, task rewards, and renderer contract are hard-coded across
`ocean/robot_arm/robot_arm.h`, `robot_arm_cuda.cuh`, and `robot_arm.cu`.
Porting it to Gradient0 is a simulator implementation task.

## Target system

The target is the six-DOF Gradient0 / Cheap Robot Arm with Feetech STS3215
actuators and its rack-and-pinion parallel gripper. Training must use logical
joint coordinates, not the nine physical servo IDs. Twin shoulder and elbow
servos are one simulated joint each; physical actuator mapping belongs to the
later GradientOS deployment adapter.

The current GradientOS worktree contains relevant URDF, meshes, joint limits,
and gripper geometry, but those files include uncommitted gripper changes. They
must be reviewed and frozen into a versioned robot-model snapshot here before
they become simulator inputs. This repository must not read them from a
GradientOS checkout at build or runtime.

## First milestone

Implement and test a deterministic CPU model of Gradient0 before implementing
the final task or porting it to CUDA. The milestone is complete only when:

1. Six-joint forward kinematics matches trusted GradientOS poses.
2. Joint limits, reset state, action scaling, and gripper travel are explicit.
3. Link masses, centers of mass, inertias, collision approximations, motor
   limits, and control rate are versioned and provenance is recorded.
4. Free-space stepping, gravity behavior, table contact, and gripper closing
   have deterministic tests.
5. PufferLib can collect observations, actions, rewards, and terminal flags from
   the environment without any GradientOS dependency.

The detailed sequence is in
[`docs/bottlecap-rl/PORTING_PLAN.md`](docs/bottlecap-rl/PORTING_PLAN.md).

## Upstream workflow

```bash
git fetch upstream
git rebase upstream/5.0
```

Push project changes to the fork's project branch:

```bash
git push origin bottlecap-rl
```
