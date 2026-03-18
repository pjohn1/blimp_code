########################################################
# optitrack_node.py
#
# Receives data from Windows script that communicates with OptiTrack
# Note: THIS BINDS TO A SCRIPT ON THE WINDOWS SYSTEM
# That script is launched by the launch file, but this script will
# Not be able to communicate directly with OptiTrack as WSl
# is a virtual machine
#
########################################################


import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from blimp_msgs.msg import OptiTrackPose, GoalMsg, Blimps
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Int32

import threading
import socket
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import time
import re

import subprocess

LOCAL_IP = "0.0.0.0"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LOCAL_IP,1511))
sock.setblocking(True)

data_lock = threading.Lock()
AGENT_PATTERN = re.compile(r"^agent_\d+$")

class OptiTrackNode(Node):
    def __init__(self):
        super().__init__('optitrack_node')

        self.data_lock = threading.Lock()
        self.goal_to_agent = dict()
        self.goal_publisher_map = dict()
        self.publisher_map = dict()

        self.maps_init = False

        self.discovered_id_pub = self.create_publisher(Int32, '/optitrack_node/discovered_id', 20)
        self.create_subscription(Blimps, '/blimps_initialize', self.blimps_initialize_callback, 10)

    def blimps_initialize_callback(self, msg):
        with self.data_lock:
            ids, goals = [int(id_) for id_ in msg.ids], [int(goal) for goal in msg.goals]
            for id_, goal in zip(ids,goals):
                self.goal_to_agent[goal] = id_
                self.goal_publisher_map[id_] = self.create_publisher(GoalMsg, f'/agent_{id_}/controller/goal', 5)
                self.publisher_map[id_] = self.create_publisher(OptiTrackPose, f'/agent_{id_}/optitrack_node/pose', 5)
            self.maps_init = True

    def publish_goal(self,goal_values,publisher):
        msg = GoalMsg() #Custom goal message defined in blimp_msgs/msg/GoalMsg.msg
        msg.id = int(goal_values[-1])
        msg.x = goal_values[0]
        msg.y = goal_values[1]
        msg.z = goal_values[2] + 1.0
        msg.roll = goal_values[3]
        msg.pitch = goal_values[4]
        msg.yaw = goal_values[5]
        msg.ux = goal_values[6]
        msg.uy = goal_values[7]
        msg.uz = goal_values[8]
        msg.wx = goal_values[9]
        msg.wy = goal_values[10]
        msg.wz = goal_values[11]
        publisher.publish(msg)

    def socket_thread(self):
        '''
        Receives data from windows script and advertises it to Linux ROS

        This is necessary if you built ROS2 on WSL, Not necessary on native Linux
        '''
        while True:
            data, addr = sock.recvfrom(65565)
            data_entries = data.decode('utf-8',errors='replace').strip().split(',')
            if len(data_entries) > 1: #Goal publishing, only care about the x y
                id_,x,z,y,qx,qy,qz,qw = data_entries
                try:
                    id_int = int(id_)
                except ValueError:
                    continue
                
                # Update discovered IDS for the GUI
                discovered = Int32()
                discovered.data = id_int
                self.discovered_id_pub.publish(discovered)
                #

                if id_int in self.goal_to_agent and self.maps_init: # if goal, publish goal
                    goal_arr = [0.0]*13
                    goal_arr[0] = x
                    goal_arr[1] = y
                    goal_arr[-1] = id_int
                    self.publish_goal(np.array(goal_arr,dtype=np.float64),self.goal_publisher_map[self.goal_to_agent[id_int]])

                elif id_int in self.publisher_map and self.maps_init: #if pose, publish pose
                    roll,pitch,yaw = self.quat_to_euler(np.array([qx,qy,qz,qw]).astype(float))
                    msg = OptiTrackPose()
                    msg.id = id_int
                    msg.x = float(x)
                    msg.y = float(y)
                    msg.z = float(z)
                    msg.roll = float(roll)
                    msg.yaw = float(yaw)
                    msg.pitch = float(pitch)
                    msg.time = self.get_clock().now().nanoseconds/1e9
                    self.publisher_map[id_int].publish(msg)

    def quat_to_euler(self, q):
        # assume quaternion normalized (or normalize here)
        # rotation order: roll about x, pitch about z, yaw about y (intrinsic)
        qx, qy, qz, qw = q
        # qw *= -1
        r10 = 2.0*(qw*qz + qx*qy)
        r11 = 1.0 - 2.0*(qx*qx + qz*qz)
        r12 = 2.0*(qy*qz - qw*qx)
        r00 = 1.0 - 2.0*(qy*qy + qz*qz)
        r20 = 2.0*(qx*qz - qw*qy)

        # clamp for asin numeric stability
        if r10 > 1.0: r10 = 1.0
        if r10 < -1.0: r10 = -1.0

        pitch = math.asin(r10)              # pitch (about z)
        roll  = math.atan2(-r12, r11)      # roll  (about x)
        yaw   = math.atan2(-r20, r00)      # yaw   (about y)
        if yaw < 0: # yaw in range [-pi,pi] -> change to [0,2*pi]
            yaw += 2*np.pi

        return roll, pitch, yaw  # (phi, theta, psi) in radians
        
    
def main(args=None):
    rclpy.init(args=args)
    optitrack_node = OptiTrackNode()
    
    t = threading.Thread(target=optitrack_node.socket_thread,daemon=True)
    t.start()
    time.sleep(0.5)
    try:
        rclpy.spin(optitrack_node)
    except KeyboardInterrupt:
        t.join(timeout=2.0)
    finally:
        t.join(timeout=2.0)
        sock.close()
        optitrack_node.destroy_node()
        cmd = [
            "powershell.exe",           # kills all python scripts, specifically the one capturing optitrack data
            "-NoProfile",
            "-NonInteractive",
            "-Command",
            "Get-Process python | ForEach-Object { Stop-Process -Id $_.Id -Force }"
        ]
        subprocess.run(cmd)
        
if __name__ == '__main__':
    main()