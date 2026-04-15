cd blimp_clean && colcon build && source install/setup.bash
cd ..
cd blimp_msgs && colcon build && source install/setup.bash
cd .. && ros2 launch blimp_clean teleop_launch.launch.py
