########################################################
# setup_gui_node.py
#
# Minimal GUI for mapping OptiTrack IDs to COM ports
# and assigning goal targets. Subscribes to
# /optitrack_node/discovered_id to auto-populate IDs.
########################################################

import json
import shutil
import sys
import threading
from glob import glob
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray
from blimp_msgs.msg import MotorMsg, Blimps, TeleopMode, OptiTrackPose
from blimp_clean.agent_manager import AgentManager

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QFileDialog, QHBoxLayout, QHeaderView,
    QInputDialog, QLineEdit, QMainWindow, QPushButton, QTabWidget,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

SETUPS_DIR = Path.home() / ".blimp_setups"
LAST_SETUP_PATH = SETUPS_DIR / "last.json"

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
        self.verify_timers = {}  # row -> QTimer
        self.manual_rows = set()  # row indices added manually (no OptiTrack)

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

        self.mode_map={
            "Manual":0,
            "All":1,
            "Controlled":2
        }



    def _build_setup_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["ID", "COM", "Goal"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.table)

        btn_row = QHBoxLayout()

        add_btn = QPushButton("+ Add Blimp")
        add_btn.clicked.connect(self._add_manual_row)

        save_btn = QPushButton("Save Setup")
        save_btn.clicked.connect(self._save_setup)

        load_btn = QPushButton("Load Setup\u2026")
        load_btn.clicked.connect(self._load_setup)

        self._load_last_btn = QPushButton("Load Last")
        self._load_last_btn.clicked.connect(self._load_last_setup)
        self._load_last_btn.setEnabled(LAST_SETUP_PATH.exists())

        verify_btn = QPushButton("Verify Setup")
        verify_btn.clicked.connect(self._verify)

        apply_btn = QPushButton("\u2714 Apply")
        apply_btn.clicked.connect(self._apply)

        btn_row.addWidget(add_btn)
        btn_row.addWidget(save_btn)
        btn_row.addWidget(load_btn)
        btn_row.addWidget(self._load_last_btn)
        btn_row.addStretch()
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
        self.teleop_mode_combo.addItems(["Manual", "Controlled","All"])
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
        mode_val = self.mode_map[mode_text]
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

    def _add_optitrack_row(self, ids, ports):
        """Insert an OptiTrack-discovered row before any manually added rows."""
        insert_pos = self.table.rowCount() - len(self.manual_rows)
        self.table.insertRow(insert_pos)
        # Shift manual row indices up since a row was inserted above them
        self.manual_rows = {r + 1 if r >= insert_pos else r for r in self.manual_rows}
        self.table.setCellWidget(insert_pos, 0, self._make_combo(ids))
        self.table.setCellWidget(insert_pos, 1, self._make_combo(ports))
        self.table.setCellWidget(insert_pos, 2, self._make_combo(ids))

    def _remove_last_optitrack_row(self):
        """Remove the last non-manual row."""
        for row in range(self.table.rowCount() - 1, -1, -1):
            if row not in self.manual_rows:
                self.table.removeRow(row)
                self.manual_rows = {r - 1 if r > row else r for r in self.manual_rows}
                return

    def _add_manual_row(self):
        """Append a manually configured blimp row (no OptiTrack goal)."""
        ports = sorted(glob("/dev/ttyUSB*"))
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.manual_rows.add(row)

        id_edit = QLineEdit()
        id_edit.setPlaceholderText("Enter ID")
        id_edit.setAlignment(Qt.AlignCenter)

        goal_item = QTableWidgetItem("—")
        goal_item.setFlags(Qt.NoItemFlags)
        goal_item.setTextAlignment(Qt.AlignCenter)

        self.table.setCellWidget(row, 0, id_edit)
        self.table.setCellWidget(row, 1, self._make_combo(ports))
        self.table.setItem(row, 2, goal_item)

    def _refresh_options(self):
        ids = [str(i) for i in self.ros_node.get_discovered_ids()]
        ports = sorted(glob("/dev/ttyUSB*"))

        # Manage only OptiTrack rows; manual rows are user-controlled
        desired_optitrack = max(1, len(ports))
        current_optitrack = self.table.rowCount() - len(self.manual_rows)

        while current_optitrack < desired_optitrack:
            self._add_optitrack_row(ids, ports)
            current_optitrack += 1

        while current_optitrack > desired_optitrack:
            self._remove_last_optitrack_row()
            current_optitrack -= 1

        # IDs assigned sequentially; leftovers go to Goal
        assigned_ids = ids[:desired_optitrack]
        leftover_ids = ids[desired_optitrack:]
        optitrack_idx = 0

        for row in range(self.table.rowCount()):
            if row in self.manual_rows:
                # Keep COM dropdown fresh for manual rows
                self._update_combo(self.table.cellWidget(row, 1), ports)
                continue

            self._update_combo(self.table.cellWidget(row, 0), ids)
            self._update_combo(self.table.cellWidget(row, 1), ports)
            self._update_combo(self.table.cellWidget(row, 2), ids)

            id_combo = self.table.cellWidget(row, 0)
            port_combo = self.table.cellWidget(row, 1)
            goal_combo = self.table.cellWidget(row, 2)
            if not id_combo.currentText() and optitrack_idx < len(assigned_ids):
                idx = id_combo.findText(assigned_ids[optitrack_idx])
                if idx >= 0:
                    id_combo.setCurrentIndex(idx)
            if not port_combo.currentText() and optitrack_idx < len(ports):
                idx = port_combo.findText(ports[optitrack_idx])
                if idx >= 0:
                    port_combo.setCurrentIndex(idx)
            if not goal_combo.currentText() and optitrack_idx < len(leftover_ids):
                idx = goal_combo.findText(leftover_ids[optitrack_idx])
                if idx >= 0:
                    goal_combo.setCurrentIndex(idx)
            optitrack_idx += 1

    # -- save / load setup --

    def _current_setup_data(self):
        """Return the current table state as a serialisable dict."""
        rows = []
        for row in range(self.table.rowCount()):
            if row in self.manual_rows:
                id_widget = self.table.cellWidget(row, 0)
                port_widget = self.table.cellWidget(row, 1)
                rows.append({
                    "manual": True,
                    "id":  id_widget.text().strip() if id_widget else "",
                    "com": port_widget.currentText().strip() if port_widget else "",
                    "goal": "",
                })
            else:
                id_combo   = self.table.cellWidget(row, 0)
                port_combo = self.table.cellWidget(row, 1)
                goal_combo = self.table.cellWidget(row, 2)
                rows.append({
                    "manual": False,
                    "id":   id_combo.currentText().strip()   if id_combo   else "",
                    "com":  port_combo.currentText().strip() if port_combo else "",
                    "goal": goal_combo.currentText().strip() if goal_combo else "",
                })
        return {"rows": rows}

    def _write_setup(self, path, data):
        SETUPS_DIR.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=2))
        if path != LAST_SETUP_PATH:
            shutil.copy(path, LAST_SETUP_PATH)
        self._load_last_btn.setEnabled(True)

    def _save_setup(self):
        SETUPS_DIR.mkdir(parents=True, exist_ok=True)
        name, ok = QInputDialog.getText(self, "Save Setup", "Setup name:")
        if not ok or not name.strip():
            return
        dest = SETUPS_DIR / f"{name.strip()}.json"
        self._write_setup(dest, self._current_setup_data())

    def _load_setup(self):
        SETUPS_DIR.mkdir(parents=True, exist_ok=True)
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Setup", str(SETUPS_DIR), "JSON Files (*.json)"
        )
        if path:
            self._do_load(Path(path))

    def _load_last_setup(self):
        if LAST_SETUP_PATH.exists():
            self._do_load(LAST_SETUP_PATH)

    def _do_load(self, path):
        data = json.loads(path.read_text())
        rows = data.get("rows", [])

        # Remove all existing manual rows (highest index first to keep indices valid)
        for row in sorted(self.manual_rows, reverse=True):
            self.table.removeRow(row)
        self.manual_rows.clear()

        # Re-add manual rows from the save
        for row_data in rows:
            if not row_data.get("manual"):
                continue
            self._add_manual_row()
            new_row = self.table.rowCount() - 1
            id_widget = self.table.cellWidget(new_row, 0)
            if id_widget:
                id_widget.setText(row_data.get("id", ""))
            port_widget = self.table.cellWidget(new_row, 1)
            if port_widget:
                idx = port_widget.findText(row_data.get("com", ""))
                if idx >= 0:
                    port_widget.setCurrentIndex(idx)

        # Apply saved dropdown selections to optitrack rows (best-effort; port/id
        # must be present in the current system for selection to stick)
        optitrack_saves = [r for r in rows if not r.get("manual")]
        optitrack_row_indices = [r for r in range(self.table.rowCount())
                                 if r not in self.manual_rows]
        for save, row in zip(optitrack_saves, optitrack_row_indices):
            id_combo   = self.table.cellWidget(row, 0)
            port_combo = self.table.cellWidget(row, 1)
            goal_combo = self.table.cellWidget(row, 2)
            for combo, key in ((id_combo, "id"), (port_combo, "com"), (goal_combo, "goal")):
                if combo and save.get(key):
                    idx = combo.findText(save[key])
                    if idx >= 0:
                        combo.setCurrentIndex(idx)

    # -- apply configuration --

    def _apply(self):
        ids = []
        coms = []
        goals = []

        for row in range(self.table.rowCount()):
            if row in self.manual_rows:
                id_widget = self.table.cellWidget(row, 0)
                id_text = id_widget.text().strip() if id_widget else ""
                port_widget = self.table.cellWidget(row, 1)
                port = port_widget.currentText().strip() if port_widget else ""
                goal_text = ""
            else:
                id_text = self.table.cellWidget(row, 0).currentText().strip()
                port = self.table.cellWidget(row, 1).currentText().strip()
                goal_text = self.table.cellWidget(row, 2).currentText().strip()

            if not id_text or not port:
                continue
            try:
                ids.append(int(id_text))
            except ValueError:
                continue
            coms.append(port)
            goals.append(goal_text)

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
            if row in self.manual_rows:
                id_widget = self.table.cellWidget(row, 0)
                id_text = id_widget.text().strip() if id_widget else ""
                port_widget = self.table.cellWidget(row, 1)
                port = port_widget.currentText().strip() if port_widget else ""
            else:
                id_text = self.table.cellWidget(row, 0).currentText().strip()
                port = self.table.cellWidget(row, 1).currentText().strip()

            if not id_text or not port:
                continue
            try:
                agent_id = int(id_text)
            except ValueError:
                continue

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
