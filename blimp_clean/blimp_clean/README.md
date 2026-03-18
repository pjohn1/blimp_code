# This contains an overview of each script's function

## agent_manager.py

This allows for dynamic creation of ROS nodes so that the GUI can update mappings in real time.

## blimp.py

In the future, this will be an API for easy use of the blimps

## low_level_controller.py

This handles the low level PWM control based on a position/velocity control using MPC

## optitrack_node.py

This receives the OptiTrack data and publishes the poses to the network

## serial_node.py

This node writes the motor commands to the respective ground stations

## setup_gui_node.py

This node runs the GUI

## teleop_receiver.py

This node receives the teleoperation commands from the Windows instance conncted to the joystick and handles the teleop logic
