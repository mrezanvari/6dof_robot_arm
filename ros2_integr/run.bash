clear
colcon build
source install/setup.bash
chmod -R 777 .
if [ $1 == "sim" ]; then
	ros2 launch AssemblyURDF_description $2.launch.py
else
	ros2 run six_dof_ros2_controller $@
fi
