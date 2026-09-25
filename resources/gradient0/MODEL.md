# Gradient0 model snapshot

Frozen robot-model inputs for the bottlecap-rl simulator. This tree is the
authoritative geometry for training. Runtime code must not read GradientOS.

## Provenance

| Field | Value |
| --- | --- |
| Snapshot date | 2026-09-25 (America/Chicago) |
| Source tree | `/Users/dylanembry/Projects/robot_learning/GradientOS/mini-6dof-arm/` |
| Controller config | `GradientOS/src/gradient_os/arm_controller/robots/gradient0/config.py` |
| GradientOS license | MIT — see `LICENSE.GradientOS` |
| Primary URDF | `mini-6dof-arm.urdf` (includes rack-and-pinion gripper) |
| Alternate URDF | `opw-mini-arm.urdf` (reference only; not used by sim) |
| Checksums | `SHA256SUMS` |

### Files included

- `mini-6dof-arm.urdf`, `opw-mini-arm.urdf`
- `dh_params.csv`, `create_dh_params.py`
- `stl-files/{base,L1,L2,L3,L4,L5,wrist}.stl`
- `stl-files/gripper/*` including `source.json`
- `assembly/gripper.md`, `assembly/README.md`
- `controller/gradient0_config.py` (frozen copy of controller limits)

Not copied: `mini-6dof-arm.usd` (large, unused for this milestone).

## Authoritative choices

1. **Primary geometry:** `mini-6dof-arm.urdf` (not the OPW / spherical-wrist
   variant). Controller joint ordering and principal axes match this URDF.
2. **Logical DOF:** 6 arm joints + 1 gripper action. Twin shoulder/elbow servos
   are not extra simulated joints.
3. **Controller joint limits (simulation soft limits):**

   | Joint | Lower | Upper |
   | --- | ---: | ---: |
   | J1 base | -π | +π |
   | J2 shoulder | -π/2 | +π/2 |
   | J3 elbow | -π/2 | +π/2 |
   | J4 wrist roll | -π | +π |
   | J5 wrist pitch | -1.8326 | +2.0944 |
   | J6 wrist yaw | -π | +π |
   | Gripper drive | 0 | 165° (≈2.879793 rad) |

4. **Gripper kinematics** (from URDF / `assembly/gripper.md`):
   - Drive angle θ_g ∈ [0, 165°]; positive opens.
   - Each jaw travel = 0.014 m/rad (14-tooth module-2 pinion).
   - Opening width = 2 × 0.014 × θ_g → max ≈ 0.080634 m.
   - `gripper_tip` is 0.146915092086197 m from wrist along +X.
   - Controller TCP `tool_link` remains 0.180 m from wrist.

5. **Control rate:** 60 Hz policy / control step (`BC_DT = 1/60`).

## URDF joint origins used for FK (meters)

| Joint | Parent→child | Origin xyz | Axis |
| --- | --- | --- | --- |
| joint1 | base→L1 | 0, 0, 0.0843 | z |
| joint2 | L1→L2 | 0, 0, 0.04315 | y |
| joint3 | L2→L3 | 0, 0, 0.19715 | y |
| joint4 | L3→L4 | 0.176556, 0, 0.0455 | x |
| joint5 | L4→L5 | 0.05515, 0, 0 | y |
| joint6 | L5→wrist | 0.0773, 0, 0 | x |
| fixed_tool | wrist→tool_link | 0.180, 0, 0 | fixed |
| fixed_gripper_tip | gripper_base(=wrist)→gripper_tip | 0.146915092086197, 0, 0 | fixed |

At all-zero joints (identity rotations), expected positions:

| Frame | x | y | z |
| --- | ---: | ---: | ---: |
| wrist | 0.309006 | 0 | 0.3701 |
| tool_link | 0.489006 | 0 | 0.3701 |
| gripper_tip | 0.45592109 | 0 | 0.3701 |

## OPEN ISSUES (do not silently "fix")

1. **J1 limit discrepancy:** URDF `joint1` limits are ±π/2; Gradient0 controller
   soft limits are ±π. Simulation uses **controller** limits. Physical gearing /
   calibrated travel vs URDF must be verified on hardware before treating either
   as deployment truth.
2. **DH CSV vs URDF:** `dh_params.csv` was generated for an earlier kinematic
   convention and does not match the URDF origin chain 1:1. FK tests use the
   URDF chain; DH is retained as source material only.
3. **Gripper inertias:** Not characterized. Dynamics use simplified proxies.
4. **Mesh licensing:** Arm/gripper meshes come from the GradientOS MIT tree.
   Gripper CAD provenance is in `stl-files/gripper/source.json`. Confirm
   redistribution intent before any public binary release that embeds meshes.
5. **Uncommitted GradientOS gripper work:** Snapshot includes the gripper URDF
   branch and STLs present in the GradientOS worktree on the snapshot date.
   Re-sync if GradientOS gripper calibration changes.
6. **Home / grasp poses:** Controller master offsets are all zero; no calibrated
   home was found. Sim home = all zeros. Stage-1 grasp reset uses a scripted
   joint configuration documented in `ocean/bottlecap/TASK.md`.

## What this snapshot is not

- Not a MuJoCo/USD runtime dependency.
- Not a GradientOS Python import path.
- Not a validated rigid-body dynamics model.
