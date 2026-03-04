# your_pkg/launch/multi_start_launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessExit, OnShutdown
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

    serial_node = Node(
        package='blimp_ros',
        executable='serial_node',
        name='serial_node',
        output='screen',
        parameters = [{'agents': agents}]
    )

    optitrack_node = Node(
        package='blimp_ros',
        executable='optitrack_node',  # for Python ROS nodes this is the console script name
        name='optitrack_node',
        output='screen',
        parameters = [{'agents':agents},{'goals':goal_map}]
    )

    cbf_node = Node(
        package="blimp_ros",
        executable="cbf",
        name="cbf",
        output="screen",
        parameters=[{"agents":agents,'goals':goal_map,"dmin":1.0}]
    )

    ld = LaunchDescription()

    ld.add_action(start_windows_forwarder)
    ld.add_action(serial_node)  
    ld.add_action(optitrack_node)  
    ld.add_action(cbf_node)

    for a in range(len(agents)):
        if a % 2 == 1:
            group = GroupAction([
                PushRosNamespace(f'{agents[a]}'),
                Node(
                    package='blimp_ros',
                    executable='low_level_controller',  # for Python ROS nodes this is the console script name
                    name='low_level_controller',
                    output='screen',
                    parameters = [{'goal':goal}]
                ),
                # Node(
                #     package='blimp_ros',
                #     executable='high_level_controller',
                #     name='high_level_controller',
                #     output='screen',
                #     parameters = [{'goal': goal
                #     }]

                # ),
            ])

            ld.add_action(group)


    return ld
