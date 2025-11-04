clear
colcon build
source install/setup.bash
chmod -R 777 .
ros2 run 6dof_ros2_controller $@
