# Installed pointy-finger gripper

The main URDF and its web asset copy describe the rack-and-pinion gripper shown
in `IMG_1014.HEIC`, `IMG_1015.HEIC`, and `IMG_1016.HEIC`. The old soldering-tool
`wrist.stl` is retained as an unused legacy asset. `wrist` still names the J6
mounting frame; `gripper_mount` attaches the new assembly to it.

## CAD and regeneration

The source parts were supplied locally in `/Users/dylanembry/robot_cad/GRIPPER`.
`../stl-files/gripper/source.json` records source filenames and SHA-256 hashes.
Meshes retain their original millimeter coordinates, with `scale="0.001 0.001
0.001"` in the URDF. The base and pinion are original STL exports. Rack, finger,
and end-stop meshes are tessellated from STEP, preserving their separate colors.

With `web-ui` dependencies installed, regenerate both sets of meshes using:

```sh
node scripts/import_gripper_meshes.cjs /path/to/GRIPPER
```

After a URDF edit, copy `mini-6dof-arm/mini-6dof-arm.urdf` to
`web-ui/public/assets/mini-6dof-arm/mini-6dof-arm.urdf`. The regression tests check
that both URDFs and all referenced gripper meshes match.

## Assembly coordinates

The base's cylindrical mounting axis is centered at CAD X = 58.1168426857 mm,
Y = 0, with its rear mounting plane at Z = 56.3 mm. CAD +Z maps to wrist +X,
CAD +X to wrist -Y, and CAD +Y to wrist -Z. The secondary jaw is turned 180°
about CAD Y so both fingers point forward, as in the photos.

The secondary jaw's CAD Z offset is 29.5380613908 mm relative to the primary;
this aligns their finger attachment planes and tips. Their opposing contact
faces are centered on the wrist axis at the model's closed position. The jaw
gear axis is at primary CAD Z = 14.7689693046 mm, matching the bearing axis.

The pinion is placed at 45.115 mm forward of the wrist mounting plane. This is
the nominal assembled position from the cradle's 10 mm mounting base and the
STS3215 shaft location (45.23 / 2 + 12.5 mm from the rear of the servo case).
See the [Feetech mechanical drawing, page 6](https://files.seeedstudio.com/products/Feetech/108090023_STS3215-C001_Datasheet.pdf).
The motor case is represented by a simple box; the printed parts use the CAD.

## Motion and calibration scope

`gripper_drive` takes logical servo radians, matching telemetry `gripper`.
Positive angles open the jaws. Both prismatic joints mimic the drive with
0.014 meters of travel per radian, along opposite axes. This follows the
14-tooth module-2 pinion's 14 mm pitch radius. At the configured 165° limit,
each jaw travels 40.317 mm and the modeled opening is 80.634 mm. This is a
nominal rigid mechanism: it does not model printed-gear backlash or compliance.

The visualization clamps the drive before invoking URDFLoader, because that
loader updates mimic joints before applying the drive's own limits. Non-finite
telemetry is ignored, and telemetry received before mesh loading is replayed
when the model initializes.

`gripper_tip` marks the nominal centered fingertip plane, 146.915 mm from the
wrist. The existing `tool_link` remains at 180 mm, matching the controller and
IK backends. This change does **not** recalibrate the physical TCP, servo zero,
or J6 mounting roll. The visual assembly is based on CAD and the photos, not a
measured hardware calibration. Gripper inertias have not been characterized;
the geometry/collision model is not a validated rigid-body dynamics model.
Joint effort limits use the existing arm model's nominal servo effort and the
pinion ratio; velocity limits follow the 120°/s gripper jog cap.

## Verification

```sh
.venv/bin/python -m pytest tests/test_gripper_model.py -q
npm run build --prefix web-ui
npm run dev --prefix web-ui -- --host 127.0.0.1
```

Open `/tests/gripper-preview.html` on that dev server for the actual Three.js
URDFLoader motion checks and an isolated visual inspection. Its slider changes
only the rendered model; it has no API or hardware-control connection.
