import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray, Bool
from blimp_msgs.msg import TeleopMode, MotorMsg, Blimps

import socket
import threading
import re
import time

LOCAL_IP = "0.0.0.0"
PORT = 1515
MAX_VOLTAGE = 0.5

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LOCAL_IP,PORT))
sock.setblocking(True)

data_lock = threading.Lock()
AGENT_PATTERN = re.compile(r"^agent_\d+$")

class TeleopReceiver(Node):
    def __init__(self):
        super().__init__('teleop_receiver')

        self.current_blimp = None
        self.current_mode = None
        self.teleop = True
        self.create_subscription(Blimps, '/blimps_initialize',self.update_blimps_callback, 10)
        self.create_subscription(TeleopMode, '/teleop_mode',self.update_teleop_callback, 10)

        self.motor_pub = self.create_publisher(MotorMsg, '/motor_cmd', 10)
        self.fly_to_goal_pub = self.create_publisher(Bool, "/fly_to_goal", 5)
    
    def update_blimps_callback(self, msg): # Maps blimps to com ports
        self.get_logger().info(f"Blimps: {msg.ids}, {msg.coms}")
        ids, coms = list(msg.ids), list(msg.coms)
        self.blimps = dict(zip(ids, coms))

    def update_teleop_callback(self, msg):
        self.current_blimp = msg.id
        self.current_mode = msg.mode
        self.get_logger().info(f"Teleop: id={self.current_blimp}, mode={self.current_mode}")

    def socket_thread(self):
        self.get_logger().info("Started receiving teleop")
        while rclpy.ok():
            data, addr = sock.recvfrom(65565)
            data_entries = data.decode('utf-8',errors='replace').strip().split(',')
            data_entries = [float(entry) for entry in data_entries]
            lj_horizontal, lj_vert, rj_horizontal, rj_vert, button_a, button_b, button_x, button_y, left_bumper, right_bumper = data_entries
            # self.get_logger().info(f"Current mode: {self.current_mode}, Current blimp: {lj_vert}")
            if button_a == 1:
                self.teleop = True
                self.fly_to_goal_pub.publish(Bool(data=False))
            elif button_b == 1:
                self.teleop = False
                self.fly_to_goal_pub.publish(Bool(data=True))

            if self.teleop and self.current_mode == 0 and self.current_blimp is not None: # Manual mode
                voltages = [0.0] * 6
                vertical = -lj_vert * MAX_VOLTAGE
                yaw = rj_horizontal * MAX_VOLTAGE
                forward = -rj_vert * MAX_VOLTAGE
                voltages[2] = vertical
                voltages[3] = -vertical
                if yaw > 0:
                    voltages[1] = voltages[4] = yaw
                else:
                    voltages[0] = voltages[5] = abs(yaw)
                
                if forward > 0:
                    voltages[4] += forward
                    voltages[5] += forward
                elif forward < 0:
                    voltages[0] += abs(forward)
                    voltages[1] += abs(forward)

                self.motor_pub.publish(MotorMsg(id=self.current_blimp, com=self.blimps[self.current_blimp], voltages=Float32MultiArray(data=voltages)))





def main(args=None):
    rclpy.init(args=args)
    teleop_receiver = TeleopReceiver()

    t = threading.Thread(target=teleop_receiver.socket_thread,daemon=True)
    t.start()
    time.sleep(0.5)
    try:
        rclpy.spin(teleop_receiver)
    except KeyboardInterrupt:
        t.join(timeout=2.0)
    finally:
        t.join(timeout=2.0)
        sock.close()

    teleop_receiver.destroy_node()
    rclpy.shutdown()
'''
AXES:
0: Left Joystick Left/Right (-1 left)
1: Left Joystick Up/Down (-1 up)
2: Right Joystick Left/Right (-1 left)
3: Right Joystick Up/Down (-1 up)

BUTTONS:
0: A
1: B
2: X
3: Y
4: LB
5: RB
'''