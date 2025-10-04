#! /bin/bash
colcon build --packages-select csv_pkg
source install/setup.bash
ros2 run csv_pkg reader_um7