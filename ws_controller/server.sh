#! /bin/bash
colcon build --packages-select flask_pkg
source install/setup.bash
ros2 run flask_pkg flask_server