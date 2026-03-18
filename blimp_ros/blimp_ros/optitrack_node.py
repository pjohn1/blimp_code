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
from blimp_msgs.msg import OptiTrackPose, GoalMsg
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

        self.declare_parameter('agents', Parameter.Type.STRING_ARRAY)
        self.agents = self.get_parameter('agents').value

        self.declare_parameter('goals', Parameter.Type.STRING_ARRAY)
        self.goals = self.get_parameter('goals').value

        self.publishers_created = []
        self.publish_mapping = dict()
        self.agent_set = set()
        self.goal_publishing = dict()
        self.goal_set = set()
        self.discovered_id_pub = self.create_publisher(Int32, '/optitrack_node/discovered_id', 20)
        self._configure_mappings(self.agents, self.goals)
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _validate_agents(self, agents):
        if len(agents) % 2 != 0:
            return False, "Parameter 'agents' must contain port/agent pairs."
        names = set()
        for i in range(1, len(agents), 2):
            name = str(agents[i]).strip()
            if not AGENT_PATTERN.match(name):
                return False, f"Invalid agent name '{name}'. Expected format agent_<id>."
            if name in names:
                return False, f"Duplicate agent name '{name}'."
            names.add(name)
        return True, ""

    def _validate_goals(self, goals):
        if len(goals) % 2 != 0:
            return False, "Parameter 'goals' must contain goal/target pairs."
        for i in range(0, len(goals), 2):
            goal = str(goals[i]).strip()
            target = str(goals[i + 1]).strip()
            if not AGENT_PATTERN.match(goal) or not AGENT_PATTERN.match(target):
                return False, "Goal entries must be formatted as agent_<id>."
        return True, ""

    def _destroy_dynamic_publishers(self):
        for pub in self.publishers_created:
            self.destroy_publisher(pub)
        self.publishers_created = []

    def _configure_mappings(self, agents, goals):
        valid_agents, reason_agents = self._validate_agents(agents)
        if not valid_agents:
            raise ValueError(reason_agents)
        valid_goals, reason_goals = self._validate_goals(goals)
        if not valid_goals:
            raise ValueError(reason_goals)

        self._destroy_dynamic_publishers()
        self.agents = list(agents)
        self.goals = list(goals)
        self.publish_mapping = {}
        self.goal_publishing = {}
        self.agent_set = set()
        self.goal_set = set()

        # list format is [port, agent, port, agent, ...]
        for i in range(1, len(self.agents), 2):
            agent = self.agents[i]
            self.get_logger().info(f'setting {agent}')
            self.agent_set.add(agent)
            pub = self.create_publisher(OptiTrackPose, f'/{agent}/optitrack_node/pose', 5)
            self.publish_mapping[agent] = pub
            self.publishers_created.append(pub)

        for i in range(0, len(self.goals), 2):
            goal_agent = self.goals[i]
            target_agent = self.goals[i + 1]
            self.goal_set.add(goal_agent)
            if goal_agent not in self.goal_publishing:
                self.goal_publishing[goal_agent] = []
            pub = self.create_publisher(
                GoalMsg, f'/{target_agent}/high_level_controller/goal', 5
            )
            self.goal_publishing[goal_agent].append(pub)
            self.publishers_created.append(pub)

    def _on_set_parameters(self, params):
        next_agents = list(self.agents)
        next_goals = list(self.goals)
        changed = False

        for param in params:
            if param.name == 'agents':
                if param.type_ != param.Type.STRING_ARRAY:
                    return SetParametersResult(
                        successful=False,
                        reason="Parameter 'agents' must be a string array.",
                    )
                valid, reason = self._validate_agents(param.value)
                if not valid:
                    return SetParametersResult(successful=False, reason=reason)
                next_agents = list(param.value)
                changed = True
            elif param.name == 'goals':
                if param.type_ != param.Type.STRING_ARRAY:
                    return SetParametersResult(
                        successful=False,
                        reason="Parameter 'goals' must be a string array.",
                    )
                valid, reason = self._validate_goals(param.value)
                if not valid:
                    return SetParametersResult(successful=False, reason=reason)
                next_goals = list(param.value)
                changed = True

        if changed:
            try:
                self._configure_mappings(next_agents, next_goals)
            except Exception as exc:
                return SetParametersResult(successful=False, reason=str(exc))
            self.get_logger().info(
                f"Updated OptiTrack mappings: agents={next_agents}, goals={next_goals}"
            )
        return SetParametersResult(successful=True)
        
    def publish_goal(self,goal_values,publisher):
        msg = GoalMsg() #Custom goal message defined in blimp_msgs/msg/GoalMsg.msg
        msg.id = int(goal_values[-1])
        msg.x = goal_values[0]
        msg.y = goal_values[1]
        msg.z = goal_values[2]
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
                discovered = Int32()
                discovered.data = id_int
                self.discovered_id_pub.publish(discovered)
                string_id = f'agent_{id_}'
                if string_id in self.goal_set: # if goal, publish goal
                    goal_arr = [0.0]*13
                    goal_arr[0] = x
                    goal_arr[1] = y
                    goal_arr[-1] = id_int

                    for pub in self.goal_publishing[string_id]:
                        self.publish_goal(np.array(goal_arr,dtype=np.float64),pub)

                elif string_id in self.agent_set: #if pose, publish pose
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
                    string_id = f'agent_{id_int}'
                    self.publish_mapping[string_id].publish(msg)

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