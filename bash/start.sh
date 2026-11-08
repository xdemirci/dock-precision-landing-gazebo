#/bin/bash
clear
x-terminal-emulator -e sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console -I0 --sysid=101 --add-param-file=$HOME/gz_ws/src/ardupilot_gazebo/config/gazebo-iris-gimbal.parm
sleep 3
x-terminal-emulator -e python3 ../scripts/xbox_control/quad_xbox_controller.py
sleep 3
x-terminal-emulator -e python3 ../scripts/dock_station/dock_flap_move.py
sleep 3
x-terminal-emulator -e python3 ../scripts/dockland/main.py
sleep 3
gz sim -r dock_world.sdf
