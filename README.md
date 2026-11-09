https://github.com/user-attachments/assets/9e075b9d-5362-4bec-83c0-95256a364532

[sample_dock_flap.webm](https://github.com/user-attachments/assets/131b41e0-590c-4a33-b58d-ae51f04e3069)

[sample_dock_all_flap.webm](https://github.com/user-attachments/assets/ec00ee02-7f35-4f74-9ee9-6c4fed235482)

[landing_on_aruco_all.webm](https://github.com/user-attachments/assets/5933c292-b72c-4da7-89fa-2a91683e1948)

# Precision Landing on Dock Station in Gazebo

A quadcopter finds an **ArUco marker** with a **downward camera**, centers over it, descends, lands and disarms. The dock has motorized walls and flaps that open and close. Everything runs in Gazebo Harmonic with ArduPilot SITL. **Simulation only.**

**TO REMIND**, ArduPilot's precision landing action is **not used** in this project. The **dock station** model can be builded, configured and changed **on-demand**. One of the important roles of **simulation** is clearly **building pipelines**, testing the **visual process** and the **estimator**. Opening/closing dock station and drone's missions can be tested **together**. The library **OpenCV** and its functions can be tried to achieve best result. **Centering and precision landing algorithms** can be builded and tested in simulation as much as wished. 

No major changes is needed to test in real after a **properly** working project in the **simulation**. Also it's clear that an onboard companian computer like either Pi or Jetson is **mandatory** for image processing on the drone. Moving this project from simulation(which uses personal computer) to a real onboard computer will not be **challanging** much. The points to take into **consideration** are like **communicating with ground station** to watch the **telemetry** and **view** from camera on-demand. 

The estimation and control code in `scripts/dock_landing` is written from scratch in Python. It borrows ideas and one filter from ArduPilot's precision landing (see [What comes from ArduPilot](#what-comes-from-ardupilot)).

## How it works

```
Gazebo camera (gimbal, pointing straight down)
        |  H264 over UDP, port 5600
        v
camera.py     reader thread, always keeps the newest frame
        v
detector.py   ArUco detection (DICT_4X4_50), size/shape filters, 2-frame lock
        v
pose.py       solvePnP -> marker position, tilt, yaw, reprojection error
        v
height.py     height above the marker plane (learned from PnP, kept alive by EKF z)
        v
pose.py       pixel angle + height + roll/pitch -> (forward, right) offset in metres
        v     + vehicle position and yaw -> marker position in local NED
estimator.py  target filter (two PosVelEKF, one per axis, with outlier gate)
        v
controller.py P control + speed limit + acceleration limit + descent funnel
        v
vehicle.py    SET_POSITION_TARGET_LOCAL_NED (velocity) -> ArduPilot, GUIDED mode
        ^
mission.py    phase machine: ALIGN -> DESCEND -> FINAL -> TOUCH -> DISARM
```

Step by step, one loop iteration (one new camera frame):

1. **Frame.** `Camera.read()` blocks until a new frame arrives, so the loop runs at the camera frame rate.
2. **Detect.** `MarkerDetector` looks for the marker. If nothing is found it retries on a shadow-corrected image. A detection becomes `locked` after 2 consecutive frames.
3. **PnP.** `solvePnP` (IPPE_SQUARE) with the known marker size gives the marker position in the camera frame, its tilt, its yaw and a reprojection error. A detection is `good` only if reprojection error is at most 2 px and tilt at most 60 degrees.
4. **Height.** `MarkerHeight` turns the PnP vertical distance into a height above the marker plane. It also learns the marker's altitude relative to the EKF origin, so the height keeps working when the marker leaves the image.
5. **Measurement.** The marker centre pixel is converted to a viewing angle (undistorted). Angle, height and the vehicle's roll/pitch give the marker offset (forward, right). Adding the vehicle position and rotating by vehicle yaw gives the marker position in local NED. Only `locked` and `good` detections are used.
6. **Filter.** `TargetFilter` fuses the NED position of the marker (a fixed target). When the marker is lost it keeps predicting.
7. **Control.** The error (marker minus vehicle) becomes a velocity command with a P gain, limited in speed and acceleration. Descent speed depends on height and on how well centered the vehicle is.
8. **Mission.** `LandingSequencer` decides the phase, detects touchdown and sends disarm.
9. **Send.** `Vehicle.send_vel_ned` sends the velocity in the local NED frame (only when running and in GUIDED).

## Repository layout

```
bash/
  start.sh                  opens each component in its own terminal (check paths before use)
  config_files/
    camera_config           gz command that enables the camera video stream
    gimbal_config           ArduPilot gimbal parameters (nadir by default)
    gz_commands             example command to move the dock
models/
  arucotag1.png             marker texture
  dock_model ... _4         four docks with 1, 3, 5, 7 m pedestals
scripts/
  dock_landing/             main project: detection, estimation, control, mission
  station_control/          dock generator and the flap control GUI
  xbox_control/             gamepad ground station used to take off and fly manually
worlds/
  dock_world.sdf            world with the drone and four docks
```

## Files

### scripts/dock_landing (main project)

| File | Purpose |
|---|---|
| `main.py` | Thin loop that connects all modules. Keys: SPACE start/pause, `y` yaw alignment on/off, `q` quit. Starts paused. |
| `config.py` | All command line arguments in one place. Other modules only read `cfg.<name>`. |
| `camera.py` | GStreamer pipeline (UDP, H264) read in a background thread. The main loop always gets the newest frame, never a queued old one. |
| `detector.py` | ArUco detection with adaptive threshold windows 3 to 203 px, sub-pixel corners, rejection of tiny or very skewed candidates, 2-frame lock, shadow correction fallback. |
| `pose.py` | Camera model (ideal pinhole from FOV, or calibration file), PnP, pixel angle to body/level frame conversions, yaw filter. |
| `height.py` | Height above the marker plane without a rangefinder. See [Height](#height). |
| `estimator.py` | `PosVelEKF` (position and velocity state) and `TargetFilter` (north and east axis, outlier gate, re-initialisation). |
| `controller.py` | Velocity command: P control, speed limit, acceleration limit, descent funnel, optional yaw alignment. |
| `mission.py` | Landing phase machine, touchdown detection, disarm and verification. Pure logic, no MAVLink. |
| `vehicle.py` | MAVLink state and commands, non-blocking. |
| `hud.py` | Information panel above the video. Text scales with `--disp-w`. |
| `selftest.py` | Hardware-free tests (see below). |
| `classes_dockland.mmd`, `packages_dockland.mmd` | Mermaid class and module diagrams. |

`selftest.py` runs without camera, MAVLink or display. It checks filter convergence and outlier rejection, the angle to NED chain, PnP on synthetic marker poses, the landing sequence (including "no touchdown before takeoff"), speed and acceleration limits, behaviour when the target is lost, height learning, the descent funnel, close-range detection, detection under hard shadow and HUD drawing.

### Other scripts

| File | Purpose |
|---|---|
| `station_control/gen_dock.py` | Generates a dock `model.sdf`. `--h` pedestal height, `--ped-w` pedestal width, `--mark-h` height of the raised marker pad, `--out` output file. |
| `station_control/dock_flap_move.py` | Small Tk window with OPEN and CLOSE buttons for each dock and for all docks. Publishes the flap angle (1.2 rad when open) through gz-transport. |
| `station_control/gen_dock_start` | Example `gen_dock.py` commands. |
| `xbox_control/quad_xbox_controller.py` | Gamepad ground station (Tk and pygame). A takeoff to 5 m, X STABILIZE, Y LOITER, LT POSHOLD, RT GUIDED, sticks for velocity (GUIDED) or RC override (manual), gimbal control through RC6 to RC8. |

## Architecture: processes and threads

`scripts/dock_landing/main.py` is **one process with two threads**:

| Thread | Work |
|---|---|
| Camera thread (daemon) | Reads frames from GStreamer, stores the newest one, wakes the main thread. |
| Main thread | Waits for a frame, polls MAVLink (non-blocking), detection, PnP, height, filter, control, mission, HUD drawing, key handling. |

There is no multiprocessing. MAVLink is read in the main thread with `recv_match(blocking=False)`. Blocking pymavlink helpers (such as `motors_disarmed_wait`) are deliberately avoided because they would freeze the video loop.

A full simulation session is several separate processes:

| Process | Role |
|---|---|
| `gz sim` | Physics, rendering, camera, dock joints |
| `sim_vehicle.py` (ArduCopter SITL, MAVProxy console) | Autopilot, MAVLink on `udp:127.0.0.1:14550`, sysid 101 |
| `dock_landing/main.py` | Landing logic (above) |
| `xbox_control/quad_xbox_controller.py` | Manual flight, takeoff, gimbal |
| `station_control/dock_flap_move.py` | Dock flap buttons |

## The controller: where PID is and is not used

The landing controller is **not a full PID**. It is a **P controller with limits**:

- Horizontal: `v = kp * error`, with `kp = 0.5 1/s`. The vector is clipped to `--vmax` (0.5 m/s). The command then moves toward this target with a slew limit of `--amax` (0.6 m/s^2). The slew limit is what smooths the motion and avoids overshoot. There is no integral term and no derivative term.
- Yaw (optional): `yaw_rate = kyaw * yaw_error`, `kyaw = 0.8`, clipped to 30 deg/s, ignored below 2 degrees.
- Descent: `nominal = clip(0.3 * h, vfinal, 0.6)` m/s, multiplied by a **funnel** value. The funnel is 1 when the horizontal error is at most `tol` (0.15 m), falls linearly to 0 at `2 * tol`, so the drone stops descending when it drifts off. In the FINAL phase the funnel never goes below 0.3, so the descent continues when the marker is no longer visible.

PID loops that do exist in the system:

- **ArduPilot** runs its own attitude and position PID loops. This project only sends velocity targets and ArduPilot tracks them.
- **Gazebo `JointPositionController`** moves the dock flaps with P = 300, I = 2, D = 30 (and joint damping 5).
- **ArduPilot servo gimbal** holds the camera at nadir (`MNT1_NEUTRAL_Y = -90`).

## Landing detection and disarm

Touchdown is detected in the `FINAL` or `DESCEND` phase, and only after the vehicle was seen airborne (landed state IN_AIR, or height above 0.5 m, or descending faster than 0.1 m/s). This prevents a disarm command right at the start. Touchdown is declared when either is true:

- ArduPilot reports `EXTENDED_SYS_STATE.landed_state == ON_GROUND` (ArduPilot's own land detector; requested at 5 Hz).
- Backup: in FINAL, height above the marker is below `--touch-h` (0.6 m) and vertical speed is below 0.05 m/s for 1.5 seconds.

After touchdown (`TOUCH` phase) the script keeps a small downward velocity and sends `MAV_CMD_COMPONENT_ARM_DISARM` once per second. The result is verified from the `HEARTBEAT` armed flag. If ArduPilot refuses for `--disarm-timeout` (6 s), the HUD shows that a manual disarm is needed. With `--force-disarm` the script then sends the forced disarm (param2 = 21196), and only when height is below 1 m and vertical speed below 0.3 m/s.

With `--land-mode` the final step is different: when the marker is tracked, height is below `--land-h` (1 m) and error is below `tol`, the script switches to LAND and only verifies that ArduPilot disarms.

Phases:

| Phase | Condition |
|---|---|
| ALIGN | Marker tracked, error at least `2 * tol` (or `--no-descend`), or nothing tracked yet |
| DESCEND | Error below `2 * tol`, height above `--final-h` |
| FINAL | Error below `2 * tol` and height below `--final-h` (2 m). Latched: continues on the filter alone for up to `--final-lost-s` (20 s) without the marker. Cancelled if error grows above `3 * tol`. |
| TOUCH | Touchdown detected |
| LAND | `--land-mode` only |
| DONE | Disarm confirmed |

## Estimation details

### Height

`height.py` gives the height above the marker plane (not above the ground). The marker sits on a pad on a pedestal, so the plane height is different for every dock.

- While the marker is good, the vertical distance from PnP (corrected for roll and pitch) is the live height.
- From it the module learns the marker altitude relative to the EKF origin: `m_alt = -z_ekf - h_pnp`. The estimate is smoothed (rate 0.05 per frame) and not updated while the vehicle moves vertically faster than 0.3 m/s (camera delay would not match the EKF).
- Afterwards `h = -z_ekf - m_alt`. PnP noise does not leak into the height, and it still works when the marker is out of view or out of the frame. No rangefinder is needed and the dock height does not have to be known.
- A measurement more than 1.5 m away from the learned altitude is ignored. After 20 such measurements in a row a new dock is assumed.
- `--h-src` selects the source: `pnp` (default), `rng` (rangefinder `DISTANCE_SENSOR`, falls back to PnP) or `ekf` (fixed `--dock-h`). `--h-scale` multiplies the PnP depth if it needs a correction.

### Target filter

- State per axis: `[position, velocity]`. The target is fixed, so the velocity starts at 0 with small process noise (0.05 times dt).
- Measurement noise: `sigma = noise * h + 0.02` with `noise = 0.02` (`h` limited to at least 0.3 m). Farther away means less trust.
- Outlier gate: normalised innovation of both axes added up must stay below 18 (2 axes times 9). Otherwise the measurement is rejected and counted.
- If no measurement was accepted for 2 seconds, the filter restarts at the next measurement.
- The filter result is valid for `--lost-s` (1 s) after the last measurement, and for `--final-lost-s` (20 s) in the final phase.

## Camera, marker and dock data

### Camera (Gazebo sensor)

The camera is the sensor of the gimbal model `gimbal_small_3d` from `ardupilot_gazebo` (not part of this repository). Values below are the author's configuration.

| Item | Value |
|---|---|
| Resolution | 1920 x 1080 |
| Horizontal FOV | 1.151917 rad (66.0 degrees) |
| Vertical FOV | about 40 degrees |
| Focal length | fx = fy = 1478.27 px (from Gazebo `camera_info`) |
| Principal point | cx = 960, cy = 540 |
| Distortion | all coefficients 0 |
| Update rate | 20 Hz (model default was 10 Hz) |
| Noise | Gaussian, stddev 0.005 |
| Clip | near 0.08 m, far 300 m |
| Video stream | `GstCameraPlugin`, H264 RTP to `127.0.0.1:5600` |
| Orientation | Gimbal at nadir (`MNT1_NEUTRAL_Y = -90`). Image top = body forward. |

`pose.py` computes `fx = (width / 2) / tan(hfov / 2)`. If `--hfov` or the image size changes, the focal length follows. `--calib` loads a calibration file for a real camera.

**Important:** in `gimbal_small_3d/model.sdf`, the `CameraZoomPlugin` block is commented out. The plugin zooms the simulated image, so the real focal length no longer matches `camera_info`. See [Observed results](#observed-results).

The stream must be enabled once after the simulation starts (`bash/config_files/camera_config`).

### Marker

| Item | Value |
|---|---|
| Dictionary | `DICT_4X4_50`, ID 1 |
| Texture | `models/arucotag1.png`, 1024 x 1024 px. The black frame covers 853 px (83.3 %), the rest is a white border. |
| Size in the model | Texture plane 0.5 x 0.5 m, so the black frame is **0.4165 m** (`--marker 0.4165`). One cell is about 0.069 m. |
| White plate | 0.8 x 0.8 m under the marker. About 0.19 m (2.8 cells) of white around the black frame. |
| Pad | 0.8 x 0.8 x 0.35 m gray block holding plate and marker, so the drone lands on the pad and not on the dock base. |

The marker needs white around the black frame. On a black or gray base without the white plate, detection failed.

### Size in pixels

`side_px = 0.4165 * 1478.27 / distance = 615.7 / distance`

| Distance to marker | Marker side in image |
|---|---|
| 1 m | 616 px |
| 3 m | 205 px |
| 5 m | 123 px |
| 7 m | 88 px |
| 10 m | 62 px |
| 20 m | 31 px |
| 30 m | 21 px |

Candidates below `--min-side` (12 px) are rejected. At 1 m the image covers about 1.30 m by 0.73 m on the marker plane, so the 0.4165 m marker fits fully only above about 0.57 m. Below that the marker leaves the frame and the final descent relies on the filter.

### Docks

| Model | World position | Pedestal | Marker plane height |
|---|---|---|---|
| `dock_model` | (5, 5) | 1 m | about 1.40 m |
| `dock_model_2` | (5, -5) | 3 m | about 3.40 m |
| `dock_model_3` | (-5, 5) | 5 m | about 5.40 m |
| `dock_model_4` | (-5, -5) | 7 m | about 7.40 m |

Each dock: pedestal 1.8 x 1.8 m (mass 80 kg per metre of height), base 2 x 2 x 0.05 m, two side walls (0.05 x 2 x 0.9 m) each with a 1 m flap on top, hinged at the base edge, and front and back panels (1.88 x 0.05 x 0.89 m). All four hinged parts open together: side walls and flaps use `/<dock>/left/cmd` (negative angle) and `/<dock>/right/cmd` (positive angle), the front panel follows the left topic and the back panel the right topic. Joint limit 1.6 rad, controlled with 1.2 rad when open. A `VelocityControl` plugin lets a dock be moved with `/model/<dock>/cmd_vel`.

Docks are generated with `station_control/gen_dock.py`. The models in `models/` were made with `--ped-w 1.8 --mark-h 0.35` and `--h` 1, 3, 5, 7.

### World

`worlds/dock_world.sdf`: ArduPilot Gazebo style world. Physics step 1 ms, sensors with ogre2, directional sun with shadows, spherical coordinates at the CMAC field (-35.363262, 149.165237, 584 m). The drone `iris_with_gimbal` starts at (0, 0) facing 90 degrees. The four docks stand at (+-5, +-5).

## What comes from ArduPilot

- **`estimator.py`** is a Python port of ArduPilot's `PosVelEKF` (the position and velocity filter used by ArduPilot's precision landing estimator). The predict and fuse equations follow it. The target filter around it (two axes, outlier gate 9, restart after 2 s, process noise 0.05) is chosen in this project and is not claimed to equal ArduPilot's values.
- **Camera capture** keeps only the newest frame, following the `image_capture` idea noted in the code.
- **Viewing vector convention** `(-tan(angle_y), tan(angle_x), 1)` in the body frame is taken from ArduPilot's MAVLink precision landing backend, and `los_to_level_xy` uses the same convention.
- **`ardu_prec_land.py --precland`** and **`prec_land_only_ardu.py`** use ArduPilot's own precision landing: they send `LANDING_TARGET` messages (distance 0, so the height comes from a rangefinder). Needed parameters: `PLND_ENABLED 1`, `PLND_TYPE 1`, `PLND_EST_TYPE 1`, `PLND_LAG 0.15`, `PLND_STRICT 1`, and a rangefinder that sees the dock.
- Landing state, disarm and velocity control use standard MAVLink: `EXTENDED_SYS_STATE`, `MAV_CMD_COMPONENT_ARM_DISARM`, `SET_POSITION_TARGET_LOCAL_NED`.

## Observed results

Observations from simulation, not a measured benchmark:

- Height from PnP and the height used by the controller agree on the 1, 3, 5 and 7 m docks.
- Starting `main.py` directly, the marker distance from PnP is found correctly.
- With `CameraZoomPlugin` active, the image scale did not match `camera_info`. At a true distance of 4 m the marker measured 61 px (expected 154 px) and PnP returned about 10 m. With the plugin block commented out the values match.
- A black or gray base made detection fail. A marker on a white plate detects reliably, including directly above it.

## Requirements

Ubuntu 22.04, Gazebo Harmonic, ArduPilot SITL, `ardupilot_gazebo` plugin (provides `iris_with_gimbal`, `runway` and the camera plugins), Python 3, OpenCV with `aruco` and GStreamer support, pymavlink, numpy. The gamepad script also needs pygame and Tk.

## Run (short version)

```bash
export GZ_SIM_RESOURCE_PATH=$PWD/models:$PWD/worlds:$GZ_SIM_RESOURCE_PATH
gz sim -r worlds/dock_world.sdf

# ArduCopter SITL with the gimbal parameters
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console -I0 --sysid=101 \
  --add-param-file=<path to gazebo-iris-gimbal.parm>

# enable the camera stream (command in bash/config_files/camera_config)

python3 scripts/dock_landing/selftest.py     # optional, no hardware needed
python3 scripts/xbox_control/quad_xbox_controller.py     # take off, switch to GUIDED
python3 scripts/dock_landing/main.py --conn udp:127.0.0.1:14550
```

Press SPACE in the HUD window to start. The vehicle must be in GUIDED mode. The dock models load the marker as `model://arucotag1/arucotag1.png`, so the texture must be reachable under that path (a folder `arucotag1` containing the PNG inside a directory on `GZ_SIM_RESOURCE_PATH`).

A detailed guide for trying the code will follow.

## Assumptions and limits

- The camera points straight down and the top of the image is the vehicle's front.
- The marker lies flat and does not move. The target filter assumes a fixed target, so a moving dock is not tracked well.
- Camera delay is not compensated (the frame arrival time is used).
- No real camera was tested. Simulation uses an ideal pinhole camera without lens distortion, rolling shutter or motion blur.
- Close to the marker the image no longer shows it fully; the last part of the descent depends on the filter and the EKF altitude.

## License

MIT. The world is derived from the ArduPilot Gazebo iris runway world.
