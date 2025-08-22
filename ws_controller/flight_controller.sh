#! /bin/bash
colcon build --packages-select flight_controller_pkg
source install/setup.bash
ros2 run flight_controller_pkg fc_controller