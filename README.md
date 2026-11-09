<<<<<<< HEAD


https://github.com/user-attachments/assets/0a996c88-59ba-46aa-aa1e-0d7f7446f4e0


https://github.com/user-attachments/assets/34badff9-970c-49cb-85ec-bd321d4e8954

[dock_flap.webm](https://github.com/user-attachments/assets/cf2b7023-6117-4d6c-90e3-a2a608a8477d)
=======
https://github.com/user-attachments/assets/5ca7af52-cdfd-446f-985d-3ea453b09b7a
[dock_flap_test.webm](https://github.com/user-attachments/assets/9af74342-30e5-4168-8617-8f6afef13ad9)
https://github.com/user-attachments/assets/bb9f285d-4894-4a67-b6a7-a2c7bf2709e6
>>>>>>> e40e5fd (bigchange)

# dock-precision-landing-gazebo
**************************************************
README WILL BE REVISED SOON.
**************************************************
Quadcopter dock station and ArUco-based centering/precision landing in Gazebo Harmonic with ArduPilot SITL. **Simulation only.**

#![demo](docs/demo.gif)

## What's inside
- `dock_model`: dock with two side walls and flaps (revolute joints, `JointPositionController` on `/dock/left/cmd` and `/dock/right/cmd`), ArUco marker on the base
- `arucotag1`: ArUco marker model
- `detect_aruco.py`: marker detection from the gimbal camera stream (GStreamer, UDP 5600)

## Status
- [x] Dock model v1/v2
- [x] ArUco detection
- [x] Centering over the marker
- [x] Precision landing estimation (own implementation)

## Notes
Detection failed with a black or gray base and worked once the marker sat on a white plate. Contrast matters.

## Requirements
Ubuntu 22.04, Gazebo Harmonic, ArduPilot SITL + ardupilot_gazebo plugin, Python 3, OpenCV (with `aruco`), pymavlink

## Run
```bash
export GZ_SIM_RESOURCE_PATH=$PWD/models:$PWD/worlds:$GZ_SIM_RESOURCE_PATH
gz sim -r worlds/dock_world.sdf
sim_vehicle.py -v ArduCopter --model JSON ...
python3 scripts/detect_aruco.py
```

## License
MIT. World derived from the ArduPilot Gazebo iris runway world.
