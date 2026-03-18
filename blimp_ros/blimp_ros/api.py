########################################################
# TODO: Setup API for blimps                           #
########################################################

import rclpy
from rclpy.node import Node
from rclpy.msgs import MotorMsg

class BlimpAPI(Node):
    def __init__(self):
        super().__init__('blimp_api')
        self.declare_parameter('agents', Parameter.Type.STRING_ARRAY)
        self.agents = self.get_parameter('agents').value

        self.declare_parameter('goals', Parameter.Type.STRING_ARRAY)
        self.goals = self.get_parameter('goals').value

    def run(self):
        rclpy.spin(self)

    def write_motor_commands(self, id_, voltages):
        msg = MotorMsg()
        msg.id = id_
        msg.voltages = voltages
        self.publisher.publish(msg)

    def fly_at_velocity(self, id_, velocity):
        return
    