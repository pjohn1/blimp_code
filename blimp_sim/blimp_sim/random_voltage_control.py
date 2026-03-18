import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import numpy as np


class RandomVoltageControl(Node):
    def __init__(self):
        super().__init__("random_voltage_control")
        self.publisher = self.create_publisher(
            Float32MultiArray, "/controls/control_update", 5
        )
        self.subscriber = self.create_subscription(
            Float32MultiArray, "/sim/robot_pos", self.update_control, 5
        )

        self.declare_parameter("num_blimps", 2)
        self.num_blimps = int(self.get_parameter("num_blimps").value)
        self.rng = np.random.default_rng()

    def update_control(self, _msg):
        controls = []
        for _ in range(self.num_blimps):
            vertical = self.rng.uniform(-1.0, 1.0, size=2)
            corner = self.rng.uniform(0.0, 1.0, size=4)
            controls.extend(vertical.tolist() + corner.tolist())

        msg = Float32MultiArray()
        msg.data = controls
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RandomVoltageControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
