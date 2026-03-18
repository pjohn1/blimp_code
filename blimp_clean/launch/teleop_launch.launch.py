# your_pkg/launch/multi_start_launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


import os

def generate_launch_description():

    agents = [ # Maps COM port from ground station to the agent's Motive ID
        '/dev/ttyUSB1','agent_1','/dev/ttyUSB0','agent_0']
    goal_map = [ # Maps goal ID to the targets that are trying to reach it
        'agent_62','agent_0','agent_63','agent_1'
    ]

    goal = [0.0]*12
    goal[2] = 1.5

    start_windows_forwarder = ExecuteProcess(
    cmd=[
        'powershell.exe',
        '-NoProfile',
        '-ExecutionPolicy', 'Bypass',
        '-Command',
        "& 'C:\\Users\\c3gro\\Desktop\\Blimp Code\\pj_test\\blimp\\Scripts\\python.exe' "
        "'C:\\Users\\c3gro\\Desktop\\Blimp Code\\pj_test\\basic_optitrack_with_wsl.py'"
    ],
        output='both',
        emulate_tty=True,
        # sigkill_timeout=2.0
    )

    start_windows_teleop_sender = ExecuteProcess(
    cmd=[
        'powershell.exe',
        '-NoProfile',
        '-ExecutionPolicy', 'Bypass',
        '-Command',
        "py -3 'C:\\Scripts\\basic_teleop_sender.py'"
    ],
        output='both',
        emulate_tty=True,
    )

    serial_node = Node(
        package='blimp_clean',
        executable='serial_node',
        name='serial_node',
        output='screen',
        parameters = [{'agents': agents}]
    )

    optitrack_node = Node(
        package='blimp_clean',
        executable='optitrack_node',  # for Python ROS nodes this is the console script name
        name='optitrack_node',
        output='screen',
        parameters = [{'agents':agents},{'goals':goal_map}]
    )

    setup_gui_node = Node(
        package='blimp_clean',
        executable='setup_gui_node',
        name='setup_gui_node',
        output='screen',
    )

    teleop_node = Node(
        package='blimp_clean',
        executable='teleop_receiver',
        name='teleop_node',
        output='screen',
    )


    ld = LaunchDescription()

    ld.add_action(start_windows_forwarder)
    ld.add_action(start_windows_teleop_sender)
    ld.add_action(serial_node)  
    ld.add_action(optitrack_node)  
    ld.add_action(teleop_node)
    ld.add_action(setup_gui_node)


    return ld
