########################################################
# setup_gui_node.py
#
# Minimal GUI for mapping OptiTrack IDs to COM ports
# and assigning goal targets. Subscribes to
# /optitrack_node/discovered_id to auto-populate IDs.
########################################################

import sys
import threading
from glob import glob

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray
from blimp_msgs.msg import MotorMsg, Blimps, TeleopMode, OptiTrackPose
from blimp_clean.agent_manager import AgentManager

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QHBoxLayout, QHeaderView,
    QMainWindow, QPushButton, QTabWidget, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

VERIFY_VOLTAGES = [0.0, 0.0, 0.4, 0.4, 0.0, 0.0]
VERIFY_DURATION = 0.5   # seconds
VERIFY_RATE = 30         # Hz


class SetupGuiNode(Node):
    def __init__(self):
        super().__init__("setup_gui_node")
        self.id_lock = threading.Lock()
        self.discovered_ids = set()
        self.create_subscription(
            Int32, "/optitrack_node/discovered_id", self._on_discovered_id, 50
        )

        # Publishers
        self.motor_pub = self.create_publisher(MotorMsg, "/motor_cmd", 10)
        self.blimps_pub = self.create_publisher(Blimps, "/blimps_initialize", 10)
        self.teleop_mode_pub = self.create_publisher(TeleopMode, "/teleop_mode", 10)

        # Pose tracking
        self.pose_lock = threading.Lock()
        self.latest_poses = {}   # id -> OptiTrackPose
        self.pose_subs = {}      # id -> subscription

    def _on_discovered_id(self, msg: Int32):
        with self.id_lock:
            self.discovered_ids.add(int(msg.data))

    def get_discovered_ids(self):
        with self.id_lock:
            return sorted(self.discovered_ids)

    def publish_motor(self, agent_id, com_port):
        self.get_logger().info(f"Publishing motor commands to {com_port}")
        msg = MotorMsg()
        msg.id = agent_id
        msg.com = com_port
        msg.voltages = Float32MultiArray(data=VERIFY_VOLTAGES)
        self.motor_pub.publish(msg)

    def publish_motor_stop(self, agent_id, com_port):
        msg = MotorMsg()
        msg.id = agent_id
        msg.com = com_port
        msg.voltages = Float32MultiArray(data=[0.0] * 6)
        self.motor_pub.publish(msg)

    def subscribe_to_poses(self, agent_ids):
        # Destroy old subs
        for sub in self.pose_subs.values():
            self.destroy_subscription(sub)
        self.pose_subs.clear()
        with self.pose_lock:
            self.latest_poses.clear()

        for aid in agent_ids:
            topic = f"/agent_{aid}/optitrack_node/pose"
            self.pose_subs[aid] = self.create_subscription(
                OptiTrackPose, topic, lambda msg, a=aid: self._on_pose(a, msg), 10
            )
            self.get_logger().info(f"Subscribed to {topic}")

    def _on_pose(self, agent_id, msg):
        with self.pose_lock:
            self.latest_poses[agent_id] = msg

    def get_latest_poses(self):
        with self.pose_lock:
            return dict(self.latest_poses)

    def publish_blimps(self, ids, coms, goals):
        msg = Blimps()
        msg.ids = ids
        msg.coms = coms
        msg.goals = goals
        self.blimps_pub.publish(msg)
        self.get_logger().info(f"Published /blimps_initialize: ids={ids}, coms={coms}, goals={goals}")


class SetupGuiWindow(QMainWindow):
    def __init__(self, ros_node: SetupGuiNode):
        super().__init__()
        self.ros_node = ros_node
        self.agent_manager = AgentManager()
        self.setWindowTitle("Blimp Setup")
        self.resize(600, 300)
        self.verify_timers = {}  # row -> (publish_timer, stop_timer)

        tabs = QTabWidget()
        tabs.addTab(self._build_setup_tab(), "Setup")
        tabs.addTab(self._build_teleop_tab(), "Teleop")
        tabs.addTab(self._build_telemetry_tab(), "Telemetry")
        self.setCentralWidget(tabs)

        # Refresh dropdowns every 500ms
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh_options)
        self.refresh_timer.start(500)

        # Refresh telemetry table at 5Hz
        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.timeout.connect(self._refresh_telemetry)
        self.telemetry_timer.start(200)



    def _build_setup_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["ID", "COM", "Goal"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.table)

        btn_row = QHBoxLayout()
        verify_btn = QPushButton("Verify Setup")
        apply_btn = QPushButton("\u2714 Apply")
        verify_btn.clicked.connect(self._verify)
        apply_btn.clicked.connect(self._apply)
        btn_row.addWidget(verify_btn)
        btn_row.addWidget(apply_btn)
        layout.addLayout(btn_row)

        return tab

    def _build_teleop_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        row = QHBoxLayout()
        self.teleop_blimp_combo = QComboBox()
        self.teleop_blimp_combo.setEditable(False)
        self.teleop_blimp_combo.addItem("")
        self.teleop_mode_combo = QComboBox()
        self.teleop_mode_combo.setEditable(False)
        self.teleop_mode_combo.addItems(["Manual", "Controlled"])
        set_teleop_btn = QPushButton("Set Teleop")
        set_teleop_btn.clicked.connect(self._set_teleop)
        row.addWidget(self.teleop_blimp_combo)
        row.addWidget(self.teleop_mode_combo)
        row.addWidget(set_teleop_btn)
        layout.addLayout(row)

        layout.addStretch()
        return tab

    def _set_teleop(self):
        selected = self.teleop_blimp_combo.currentText().strip()
        if not selected:
            return
        mode_text = self.teleop_mode_combo.currentText()
        mode_val = 0 if mode_text == "Manual" else 1
        msg = TeleopMode()
        msg.id = int(selected)
        msg.mode = mode_val
        self.ros_node.teleop_mode_pub.publish(msg)
        self.ros_node.get_logger().info(f"Teleop: id={selected}, mode={mode_text} ({mode_val})")

    def _build_telemetry_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        self.telemetry_table = QTableWidget(0, 8)
        self.telemetry_table.setHorizontalHeaderLabels(
            ["ID", "X", "Y", "Z", "Roll", "Pitch", "Yaw", "Time"]
        )
        self.telemetry_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.telemetry_table.setEditTriggers(QTableWidget.NoEditTriggers)
        layout.addWidget(self.telemetry_table)

        return tab

    def _refresh_telemetry(self):
        poses = self.ros_node.get_latest_poses()
        agent_ids = sorted(poses.keys())
        self.telemetry_table.setRowCount(len(agent_ids))
        for row, aid in enumerate(agent_ids):
            p = poses[aid]
            values = [
                str(aid),
                f"{p.x:.3f}", f"{p.y:.3f}", f"{p.z:.3f}",
                f"{p.roll:.3f}", f"{p.pitch:.3f}", f"{p.yaw:.3f}",
                f"{p.time:.2f}",
            ]
            for col, val in enumerate(values):
                item = self.telemetry_table.item(row, col)
                if item is None:
                    item = QTableWidgetItem(val)
                    item.setTextAlignment(Qt.AlignCenter)
                    self.telemetry_table.setItem(row, col, item)
                else:
                    item.setText(val)

    # -- combo helpers --

    def _make_combo(self, options):
        combo = QComboBox()
        combo.setEditable(False)
        combo.addItem("")
        combo.addItems(options)
        return combo

    def _update_combo(self, combo, options):
        current = combo.currentText()
        combo.blockSignals(True)
        combo.clear()
        combo.addItem("")
        combo.addItems(options)
        idx = combo.findText(current)
        combo.setCurrentIndex(idx if idx >= 0 else 0)
        combo.blockSignals(False)

    # -- row management driven by discovery --

    def _add_row(self):
        row = self.table.rowCount()
        self.table.insertRow(row)
        ids = [str(i) for i in self.ros_node.get_discovered_ids()]
        ports = sorted(glob("/dev/ttyUSB*"))
        self.table.setCellWidget(row, 0, self._make_combo(ids))
        self.table.setCellWidget(row, 1, self._make_combo(ports))
        self.table.setCellWidget(row, 2, self._make_combo(ids))

    def _refresh_options(self):
        ids = [str(i) for i in self.ros_node.get_discovered_ids()]
        ports = sorted(glob("/dev/ttyUSB*"))

        # Row count matches number of USB ports found
        desired = max(1, len(ports))
        while self.table.rowCount() < desired:
            self._add_row()
        while self.table.rowCount() > desired:
            self.table.removeRow(self.table.rowCount() - 1)

        num_rows = self.table.rowCount()

        # IDs assigned sequentially to the ID column
        assigned_ids = ids[:num_rows]
        # Leftover IDs populate the Goal column
        leftover_ids = ids[num_rows:]

        for row in range(num_rows):
            self._update_combo(self.table.cellWidget(row, 0), ids)
            self._update_combo(self.table.cellWidget(row, 1), ports)
            self._update_combo(self.table.cellWidget(row, 2), ids)

            # Auto-select unset combos
            id_combo = self.table.cellWidget(row, 0)
            port_combo = self.table.cellWidget(row, 1)
            goal_combo = self.table.cellWidget(row, 2)
            if not id_combo.currentText() and row < len(assigned_ids):
                idx = id_combo.findText(assigned_ids[row])
                if idx >= 0:
                    id_combo.setCurrentIndex(idx)
            if not port_combo.currentText() and row < len(ports):
                idx = port_combo.findText(ports[row])
                if idx >= 0:
                    port_combo.setCurrentIndex(idx)
            if not goal_combo.currentText() and row < len(leftover_ids):
                idx = goal_combo.findText(leftover_ids[row])
                if idx >= 0:
                    goal_combo.setCurrentIndex(idx)

    # -- apply configuration --

    def _apply(self):
        ids = []
        coms = []
        goals = []

        for row in range(self.table.rowCount()):
            id_text = self.table.cellWidget(row, 0).currentText().strip()
            port = self.table.cellWidget(row, 1).currentText().strip()
            goal_text = self.table.cellWidget(row, 2).currentText().strip()
            if not id_text or not port:
                continue
            ids.append(int(id_text))
            coms.append(port)
            goals.append(goal_text if goal_text else "")

        self.agent_manager.initialize_blimps(ids, coms, goals)
        self.ros_node.publish_blimps(ids, coms, goals)
        self.ros_node.subscribe_to_poses(ids)

        # Populate teleop dropdown with applied IDs, default to first
        id_strs = [str(i) for i in ids]
        self.teleop_blimp_combo.blockSignals(True)
        self.teleop_blimp_combo.clear()
        self.teleop_blimp_combo.addItems(id_strs)
        if id_strs:
            self.teleop_blimp_combo.setCurrentIndex(0)
        self.teleop_blimp_combo.blockSignals(False)

    # -- verify setup --

    def _verify(self):
        total_publishes = int(VERIFY_DURATION * VERIFY_RATE)

        for row in range(self.table.rowCount()):
            id_text = self.table.cellWidget(row, 0).currentText().strip()
            port = self.table.cellWidget(row, 1).currentText().strip()
            if not id_text or not port:
                continue
            agent_id = int(id_text)

            # Cancel any existing verify for this row
            self._cancel_verify(row)

            count = [0]

            def make_publish_cb(aid, com, r):
                def cb():
                    count[0] += 1
                    if count[0] <= total_publishes:
                        self.ros_node.publish_motor(aid, com)
                    else:
                        self._cancel_verify(r)
                        self.ros_node.publish_motor_stop(aid, com)
                return cb

            timer = QTimer(self)
            timer.timeout.connect(make_publish_cb(agent_id, port, row))
            timer.start(int(1000 / VERIFY_RATE))
            self.verify_timers[row] = timer

            self.ros_node.get_logger().info(
                f"Verify: agent {agent_id} on {port} for {VERIFY_DURATION}s"
            )

    def _cancel_verify(self, row):
        timer = self.verify_timers.pop(row, None)
        if timer is not None:
            timer.stop()


def main(args=None):
    rclpy.init(args=args)
    ros_node = SetupGuiNode()
    app = QApplication(sys.argv)
    window = SetupGuiWindow(ros_node)
    window.show()

    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.0))
    spin_timer.start(20)

    try:
        app.exec_()
    finally:
        spin_timer.stop()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
