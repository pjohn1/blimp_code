import math
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from blimp_msgs.msg import GoalMsg
from sensor_msgs.msg import Joy


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")

        self.declare_parameter("goal_topic", "controller/goal")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("linear_speed", 0.4)
        self.declare_parameter("vertical_speed", 0.25)
        self.declare_parameter("yaw_rate", 0.8)
        self.declare_parameter("forward_pitch", 0.2)
        self.declare_parameter("default_altitude", 1.5)
        self.declare_parameter("agent_id", -1)

        self.declare_parameter("keyboard_enabled", True)
        self.declare_parameter("keyboard_timeout_s", 0.25)
        self.declare_parameter("joystick_enabled", True)
        self.declare_parameter("joystick_timeout_s", 0.35)

        # Xbox-style defaults
        self.declare_parameter("button_a_index", 0)
        self.declare_parameter("button_b_index", 1)
        self.declare_parameter("button_lb_index", 4)
        self.declare_parameter("button_rb_index", 5)
        self.declare_parameter("forward_backward_axis_index", 7)  # D-pad vertical
        self.declare_parameter("axis_threshold", 0.5)

        self.goal_topic = self.get_parameter("goal_topic").value
        self.joy_topic = self.get_parameter("joy_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.vertical_speed = float(self.get_parameter("vertical_speed").value)
        self.yaw_rate = float(self.get_parameter("yaw_rate").value)
        self.forward_pitch = float(self.get_parameter("forward_pitch").value)
        self.default_altitude = float(self.get_parameter("default_altitude").value)
        self.keyboard_enabled = bool(self.get_parameter("keyboard_enabled").value)
        self.keyboard_timeout_s = float(self.get_parameter("keyboard_timeout_s").value)
        self.joystick_enabled = bool(self.get_parameter("joystick_enabled").value)
        self.joystick_timeout_s = float(self.get_parameter("joystick_timeout_s").value)

        self.button_a_index = int(self.get_parameter("button_a_index").value)
        self.button_b_index = int(self.get_parameter("button_b_index").value)
        self.button_lb_index = int(self.get_parameter("button_lb_index").value)
        self.button_rb_index = int(self.get_parameter("button_rb_index").value)
        self.forward_backward_axis_index = int(
            self.get_parameter("forward_backward_axis_index").value
        )
        self.axis_threshold = float(self.get_parameter("axis_threshold").value)

        configured_id = int(self.get_parameter("agent_id").value)
        self.agent_id = configured_id if configured_id >= 0 else self._id_from_namespace()

        self.goal_pub = self.create_publisher(GoalMsg, self.goal_topic, 5)
        self.joy_sub = None
        if self.joystick_enabled:
            self.joy_sub = self.create_subscription(
                Joy, self.joy_topic, self.joy_callback, 10
            )

        self.command_lock = threading.Lock()
        self.key_cmd = {"forward": 0, "vertical": 0, "yaw": 0}
        self.joy_cmd = {"forward": 0, "vertical": 0, "yaw": 0}
        self.key_updated_at = 0.0
        self.joy_updated_at = 0.0

        self.target_yaw = 0.0
        self.last_publish_t = time.monotonic()
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_goal)

        self._stdin_fd = None
        self._stdin_old = None
        self._keyboard_thread = None
        self._keyboard_stop = threading.Event()
        if self.keyboard_enabled:
            self._start_keyboard_thread()

        self.get_logger().info(
            "Teleop started. Keyboard: W/S forward/back, R/F up/down, A/D yaw, "
            "Space stop, Q quit."
        )

    def _id_from_namespace(self):
        ns = self.get_namespace().strip("/")
        if ns.startswith("agent_"):
            suffix = ns.split("_")[-1]
            if suffix.isdigit():
                return int(suffix)
        return 0

    def _start_keyboard_thread(self):
        if not sys.stdin.isatty():
            self.get_logger().warn("Keyboard input disabled: stdin is not a TTY.")
            self.keyboard_enabled = False
            return
        self._stdin_fd = sys.stdin.fileno()
        self._stdin_old = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True
        )
        self._keyboard_thread.start()

    def _keyboard_loop(self):
        while rclpy.ok() and not self._keyboard_stop.is_set():
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not ready:
                continue
            key = sys.stdin.read(1).lower()
            now = time.monotonic()
            with self.command_lock:
                if key == "w":
                    self.key_cmd["forward"] = 1
                    self.key_updated_at = now
                elif key == "s":
                    self.key_cmd["forward"] = -1
                    self.key_updated_at = now
                elif key == "r":
                    self.key_cmd["vertical"] = 1
                    self.key_updated_at = now
                elif key == "f":
                    self.key_cmd["vertical"] = -1
                    self.key_updated_at = now
                elif key == "a":
                    self.key_cmd["yaw"] = 1
                    self.key_updated_at = now
                elif key == "d":
                    self.key_cmd["yaw"] = -1
                    self.key_updated_at = now
                elif key == " ":
                    self.key_cmd = {"forward": 0, "vertical": 0, "yaw": 0}
                    self.joy_cmd = {"forward": 0, "vertical": 0, "yaw": 0}
                    self.key_updated_at = now
                    self.joy_updated_at = 0.0
                elif key == "q":
                    self.get_logger().info("Quit requested from keyboard.")
                    self._keyboard_stop.set()
                    rclpy.shutdown()

    def joy_callback(self, msg):
        forward = 0
        vertical = 0
        yaw = 0

        if self.forward_backward_axis_index < len(msg.axes):
            axis = msg.axes[self.forward_backward_axis_index]
            if axis > self.axis_threshold:
                forward = 1
            elif axis < -self.axis_threshold:
                forward = -1

        if self.button_a_index < len(msg.buttons) and msg.buttons[self.button_a_index]:
            vertical += 1
        if self.button_b_index < len(msg.buttons) and msg.buttons[self.button_b_index]:
            vertical -= 1

        if self.button_lb_index < len(msg.buttons) and msg.buttons[self.button_lb_index]:
            yaw += 1
        if self.button_rb_index < len(msg.buttons) and msg.buttons[self.button_rb_index]:
            yaw -= 1

        vertical = max(-1, min(1, vertical))
        yaw = max(-1, min(1, yaw))

        with self.command_lock:
            self.joy_cmd["forward"] = forward
            self.joy_cmd["vertical"] = vertical
            self.joy_cmd["yaw"] = yaw
            self.joy_updated_at = time.monotonic()

    def _active_commands(self):
        now = time.monotonic()
        with self.command_lock:
            joy_fresh = self.joystick_enabled and (
                now - self.joy_updated_at <= self.joystick_timeout_s
            )
            key_fresh = self.keyboard_enabled and (
                now - self.key_updated_at <= self.keyboard_timeout_s
            )

            cmd = {"forward": 0, "vertical": 0, "yaw": 0}
            if key_fresh:
                cmd = self.key_cmd.copy()
            if joy_fresh:
                # Joystick overrides keyboard when both are active.
                cmd = self.joy_cmd.copy()
            return cmd

    def publish_goal(self):
        now = time.monotonic()
        dt = now - self.last_publish_t
        self.last_publish_t = now
        if dt <= 0.0:
            dt = 1.0 / self.publish_rate_hz

        cmd = self._active_commands()
        wz_cmd = cmd["yaw"] * self.yaw_rate
        self.target_yaw += wz_cmd * dt
        self.target_yaw = math.atan2(math.sin(self.target_yaw), math.cos(self.target_yaw))

        goal = GoalMsg()
        goal.id = int(self.agent_id)
        goal.x = 0.0
        goal.y = 0.0
        goal.z = self.default_altitude
        goal.roll = 0.0
        goal.pitch = cmd["forward"] * self.forward_pitch
        goal.yaw = self.target_yaw
        goal.ux = cmd["forward"] * self.linear_speed
        goal.uy = 0.0
        goal.uz = cmd["vertical"] * self.vertical_speed
        goal.wx = 0.0
        goal.wy = 0.0
        goal.wz = wz_cmd
        self.goal_pub.publish(goal)

    def destroy_node(self):
        self._keyboard_stop.set()
        if self._stdin_fd is not None and self._stdin_old is not None:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._stdin_old)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
