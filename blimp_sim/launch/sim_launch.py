# your_pkg/launch/multi_start_launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import os
import time

import numpy as np

def generate_launch_description():
    def circle_points(n, radius, center):
        """Equally spaced points on a circle (counterclockwise) in R^2.
        Directly ported from HW4 skeleton and updated to handle 3D 
        space as well as a different radius center
        """
        angles = np.linspace(0, 2*np.pi, n, endpoint=False)
        poses = np.stack([center+radius*np.cos(angles),center+radius*np.sin(angles)])
        I = np.ones_like(angles)
        v = np.stack([center*I,center*I]) - poses
        headings = np.arctan2(v[1],v[0])
        xy = np.stack([center+radius*np.cos(angles), center+radius*np.sin(angles),5.0*I,0*I,0*I,headings], axis=1)
        return xy

    def antipodal_goals(x0):
        """Goal for agent i is the antipode of its initial position (i+N/2 mod N).
            Code directly ported from HW4 and updated for 3D space
        """
        n = x0.shape[0]
        assert n % 2 == 0, "Antipodal swap needs an even number of robots."
        perm = [(i + n//2) % n for i in range(n)]
        # print(perm)
        goals = x0[perm][:,0:3].copy()
        # goals += 0.1*np.random.rand(goals.shape[0],goals.shape[1])
        # goals[1] += 0.5
        return goals

    N = 2
    center = np.array([6.0,6.0,3])
    params = [{
        'num_blimps': N,
        'map_size': [12.0,12.0],
        'p_center': list(center.astype(float)),
        'blimp_positions': list(circle_points(N,center[0],center[1]).flatten()),
        'blimp_goals': list(antipodal_goals(circle_points(N,center[0],center[1])).flatten()),
        'colors': list(np.random.rand(N*3)),
        'dmin': 1.0,
        'dt': 1/50,
        # 'use_sim_time':True
    }]

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # parameters=[{'use_sim_time':True}]
        )

    map_publisher = Node(
        package='blimp_sim',
        executable='map_publisher',  # for Python ROS nodes this is the console script name
        name='map_publisher',
        output='screen',
        parameters = params
    )

    delay = TimerAction(period=3.0,actions=[map_publisher])

    sim_run = Node(
        package='blimp_sim',
        executable='sim_run',  # for Python ROS nodes this is the console script name
        name='sim_run',
        output='screen',
        parameters = params
    )

    random_voltage_control = Node(
        package='blimp_sim',
        executable='random_voltage_control',  # for Python ROS nodes this is the console script name
        name='random_voltage_control',
        output='screen',
        parameters = params
    )

    ld = LaunchDescription()

    # Add actions in the order you want them started
    ld.add_action(rviz2)
    ld.add_action(delay)
    ld.add_action(sim_run)
    ld.add_action(random_voltage_control)

    return ld
