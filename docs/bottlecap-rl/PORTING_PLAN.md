# Gradient0 and bottle-cap porting plan

## What can be reused

PufferLib already supplies PPO training, rollout collection, batching, logging,
checkpointing, and the compiled environment interface. The Franka example also
supplies useful CUDA patterns for one-environment-per-thread simulation,
primitive collision detection, contact manifolds, friction impulses, and GPU
observation/reward kernels.

Those pieces reduce integration work. They do not make the Franka model generic.
The new environment should reuse proven generic routines where their behavior
is covered by tests, but it should not preserve `RA_*` Franka constants under
new names.

## Recorded target robot

The current Gradient0 sources describe six logical revolute joints plus a
gripper. The physical arm has nine Feetech STS3215 servos: one base servo, paired
shoulder servos, paired elbow servos, three wrist servos, and one gripper servo.
The simulator therefore exposes six arm actions plus one gripper action; it does
not model paired servos as extra kinematic degrees of freedom.

The current controller limits are:

| Joint | Function | Lower | Upper |
| --- | --- | ---: | ---: |
| J1 | base rotation | -pi | +pi |
| J2 | shoulder | -pi/2 | +pi/2 |
| J3 | elbow | -pi/2 | +pi/2 |
| J4 | wrist roll | -pi | +pi |
| J5 | wrist pitch | -1.8326 | +2.0944 rad |
| J6 | wrist yaw | -pi | +pi |
| Gripper servo | rack-and-pinion drive | 0 | 165 degrees |

The controller and URDF agree on the six-joint ordering and principal axes, but
the model must be frozen before implementation. The checked-out GradientOS
worktree currently contains uncommitted gripper geometry, and it contains both
the original URDF and a spherical-wrist variant. The first implementation task
is to select one authoritative geometry, copy a reviewed snapshot into this
repository with MIT attribution, and add model-consistency tests.

## Why this is not a constants-only edit

The PufferLib Franka environment assumes seven arm joints and one gripper action.
Its code directly encodes Franka transforms, joint axes, link masses, centers of
mass, inertia tensors, torque limits, collision boxes, finger pads, reset pose,
69-value observation, task state, reward terms, and eleven-mesh render order.
Its CUDA body indexes also assume a fixed set of cube, table, robot shells,
links, pads, rim, and backboard bodies.

Changing Gradient0 requires replacing each of those contracts and testing the
result. A correct port should separate reusable math/contact code from three
layers that are currently interleaved:

1. Robot model: topology, kinematics, inertial data, actuation, gripper, and
   robot collision proxies.
2. World physics: integration, rigid bodies, collision queries, contacts, and
   constraints.
3. RL task: reset distribution, observations, rewards, termination, logging,
   and curriculum.

## Implementation sequence

### 1. Freeze and validate the robot model

Create a versioned Gradient0 model snapshot in this repository. Add tests for
home pose, several known joint configurations, tool and fingertip positions,
joint limits, gripper width conversion, and self-collision exclusions. Resolve
the current J1 gearing and URDF/controller discrepancies against the physical
reference arm before treating them as simulation truth.

### 2. Build a small deterministic CPU environment

Start with free-space joint control, gravity, a table, and gripper open/close.
Use a small observation and action contract and run deterministic reset/step
tests. This stage is for finding model and task errors quickly; CUDA throughput
does not help when the equations or coordinate frames are wrong.

### 3. Add a constrained cap model

Do not begin by meshing literal bottle and cap threads. Represent the attached
cap as a one-degree-of-freedom helical constraint:

```text
cap_axial_position = starting_height + thread_pitch * cap_angle / (2 * pi)
```

The constraint should include thread direction, allowed turns, breakaway torque,
running torque, axial-force limits, radial alignment tolerance, and a detached
state. The bottle should initially be fixed to the table or held by a fixture.
This captures the action-relevant mechanics while remaining fast and stable.
Explicit thread geometry is a later validation experiment, not the first
training model.

### 4. Build the task in a curriculum

Use progressively harder initial states:

1. Gripper already centered on and holding the cap: learn wrist rotation and
   axial following.
2. Gripper above the cap: learn alignment, descent, grasp, and unscrewing.
3. Randomized bottle pose, cap friction, breakaway torque, grip friction, sensor
   noise, and actuator delay.

Track success using cap rotation, axial travel, maintained grasp, excessive
contact force, bottle motion, drops, and final detachment. Dense shaping can
reward improved alignment, stable grip, rotation in the correct direction, and
progress along the helix, but the terminal success condition must remain
physical and unambiguous.

### 5. Validate before CUDA

Compare deterministic trajectories and contact outcomes against an independent
reference such as MuJoCo for a small set of scripted motions. Validate joint
poses, gravity, gripper-cap forces, breakaway behavior, and cap angle/height
coupling. This is a correctness comparison, not a throughput benchmark.

### 6. Port the stable kernel to CUDA

Only after CPU tests and task semantics are stable should the environment be
converted to PufferLib's batched CUDA layout. Then measure environment steps per
second, learner utilization, numerical divergence across batch sizes, and PPO
learning curves.

### 7. Export rather than couple to GradientOS

Export the trained policy plus a machine-readable observation/action contract,
normalization constants, control frequency, joint ordering, gripper convention,
and safety bounds. Implement any GradientOS inference adapter later in the
GradientOS repository. Do not import controller or hardware code here.

## Immediate next coding task

The next code change is not the final unscrewing reward. It is a reviewed
Gradient0 model snapshot and a CPU forward-kinematics test harness. Once that
passes, add the helical cap state/constraint and test it with scripted actions
before PPO is involved.
