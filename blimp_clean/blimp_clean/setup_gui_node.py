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
from collections import deque
from glob import glob
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float32MultiArray
from blimp_msgs.msg import MotorMsg, Blimps, TeleopMode, OptiTrackPose
from blimp_clean.agent_manager import AgentManager

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QFileDialog,
    QHBoxLayout, QHeaderView, QInputDialog, QLabel, QLineEdit,
    QMainWindow, QPushButton, QTabWidget, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
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

        # Track externally-published teleop state (e.g. from teleop_receiver dpad cycle)
        self.teleop_lock = threading.Lock()
        self.latest_teleop = None   # (id, mode)
        self.create_subscription(TeleopMode, "/teleop_mode", self._on_teleop_mode, 10)

        # Pose tracking
        self.pose_lock = threading.Lock()
        self.latest_poses = {}   # id -> OptiTrackPose
        self.latest_dts = {}     # id -> float, seconds between last two messages
        self.pose_subs = {}      # id -> subscription

        # Calibration / param estimation tracking
        self.calib_lock = threading.Lock()
        self.latest_state_est = {}   # id -> list[5]
        self.latest_covariance = {}  # id -> list[25]
        self.calib_state_subs = {}
        self.calib_covar_subs = {}
        self.calib_pubs = {}         # id -> publisher(Bool)

    def _on_discovered_id(self, msg: Int32):
        with self.id_lock:
            self.discovered_ids.add(int(msg.data))

    def _on_teleop_mode(self, msg: TeleopMode):
        with self.teleop_lock:
            self.latest_teleop = (int(msg.id), int(msg.mode))

    def get_latest_teleop(self):
        with self.teleop_lock:
            return self.latest_teleop

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
            self.latest_dts.clear()

        for aid in agent_ids:
            topic = f"/agent_{aid}/optitrack_node/pose"
            self.pose_subs[aid] = self.create_subscription(
                OptiTrackPose, topic, lambda msg, a=aid: self._on_pose(a, msg), 10
            )
            self.get_logger().info(f"Subscribed to {topic}")

    def _on_pose(self, agent_id, msg):
        with self.pose_lock:
            prev = self.latest_poses.get(agent_id)
            if prev is not None:
                self.latest_dts[agent_id] = float(msg.time - prev.time)
            self.latest_poses[agent_id] = msg

    def get_latest_poses(self):
        with self.pose_lock:
            return dict(self.latest_poses), dict(self.latest_dts)

    def setup_calibration_topics(self, ids):
        for sub in self.calib_state_subs.values():
            self.destroy_subscription(sub)
        for sub in self.calib_covar_subs.values():
            self.destroy_subscription(sub)
        for pub in self.calib_pubs.values():
            self.destroy_publisher(pub)
        self.calib_state_subs.clear()
        self.calib_covar_subs.clear()
        self.calib_pubs.clear()
        with self.calib_lock:
            self.latest_state_est.clear()
            self.latest_covariance.clear()

        for aid in ids:
            self.calib_state_subs[aid] = self.create_subscription(
                Float32MultiArray, f"/agent_{aid}/state_est",
                lambda msg, a=aid: self._on_state_est(a, msg), 10,
            )
            self.calib_covar_subs[aid] = self.create_subscription(
                Float32MultiArray, f"/agent_{aid}/covariance",
                lambda msg, a=aid: self._on_covariance(a, msg), 10,
            )
            self.calib_pubs[aid] = self.create_publisher(Bool, f"/agent_{aid}/start_calibration", 10)

    def _on_state_est(self, agent_id, msg):
        with self.calib_lock:
            self.latest_state_est[agent_id] = list(msg.data)

    def _on_covariance(self, agent_id, msg):
        with self.calib_lock:
            self.latest_covariance[agent_id] = list(msg.data)

    def get_latest_calib_data(self):
        with self.calib_lock:
            return dict(self.latest_state_est), dict(self.latest_covariance)

    def publish_start_calibration(self, blimp_id, active=True):
        msg = Bool(data=active)
        if blimp_id == -1:
            for pub in self.calib_pubs.values():
                pub.publish(msg)
        elif blimp_id in self.calib_pubs:
            self.calib_pubs[blimp_id].publish(msg)

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
        self.resize(1200, 600)
        self.verify_timers = {}  # row -> QTimer
        self.manual_rows = set()  # row indices added manually (no OptiTrack)

        tabs = QTabWidget()
        tabs.addTab(self._build_setup_tab(), "Setup")
        tabs.addTab(self._build_teleop_tab(), "Teleop")
        tabs.addTab(self._build_telemetry_tab(), "Telemetry")
        tabs.addTab(self._build_calibration_tab(), "Calibration")
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

        self.telemetry_table = QTableWidget(0, 9)
        self.telemetry_table.setHorizontalHeaderLabels(
            ["ID", "X", "Y", "Z", "Roll", "Pitch", "Yaw", "Time", "dt (Hz)"]
        )
        self.telemetry_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.telemetry_table.setEditTriggers(QTableWidget.NoEditTriggers)
        layout.addWidget(self.telemetry_table)

        return tab

    def _refresh_telemetry(self):
        poses, dts = self.ros_node.get_latest_poses()
        agent_ids = sorted(poses.keys())
        self.telemetry_table.setRowCount(len(agent_ids))
        for row, aid in enumerate(agent_ids):
            p = poses[aid]
            dt = dts.get(aid)
            values = [
                str(aid),
                f"{p.x:.3f}", f"{p.y:.3f}", f"{p.z:.3f}",
                f"{p.roll:.3f}", f"{p.pitch:.3f}", f"{p.yaw:.3f}",
                f"{p.time:.2f}",
                f"{1/(dt+1e-12):.1f}" if dt is not None else "-",
            ]
            for col, val in enumerate(values):
                item = self.telemetry_table.item(row, col)
                if item is None:
                    item = QTableWidgetItem(val)
                    item.setTextAlignment(Qt.AlignCenter)
                    self.telemetry_table.setItem(row, col, item)
                else:
                    item.setText(val)

    def _build_calibration_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        ctrl_row = QHBoxLayout()
        ctrl_row.addWidget(QLabel("Blimp:"))
        self.calib_blimp_combo = QComboBox()
        self.calib_blimp_combo.setEditable(False)
        self.calib_blimp_combo.addItem("All")
        ctrl_row.addWidget(self.calib_blimp_combo)
        start_btn = QPushButton("Start Calibration")
        start_btn.clicked.connect(self._start_calibration)
        ctrl_row.addWidget(start_btn)
        stop_btn = QPushButton("Stop Calibration")
        stop_btn.clicked.connect(self._stop_calibration)
        ctrl_row.addWidget(stop_btn)
        ctrl_row.addStretch()
        layout.addLayout(ctrl_row)

        self._calib_fig = Figure(figsize=(8, 8), tight_layout=True)
        self._calib_canvas = FigureCanvasQTAgg(self._calib_fig)
        self._calib_axes = self._calib_fig.subplots(5, 1, sharex=True)
        _STATE_LABELS = ["z (m)", "ż (m/s)", "V (m³)", "Kv", "Cd"]
        for ax, label in zip(self._calib_axes, _STATE_LABELS):
            ax.set_ylabel(label)
            ax.grid(True)
        self._calib_axes[-1].set_xlabel("Sample")
        layout.addWidget(self._calib_canvas)

        # Rolling history per blimp: id -> {'state', 'sigma', 'n_state'} (n_state 4 or 5)
        self._calib_history = {}

        self.calib_plot_timer = QTimer(self)
        self.calib_plot_timer.timeout.connect(self._refresh_calibration_plot)
        self.calib_plot_timer.start(100)

        return tab

    def _start_calibration(self):
        text = self.calib_blimp_combo.currentText().strip()
        blimp_id = -1 if (not text or text == "All") else int(text)
        self.ros_node.publish_start_calibration(blimp_id, active=True)

    def _stop_calibration(self):
        text = self.calib_blimp_combo.currentText().strip()
        blimp_id = -1 if (not text or text == "All") else int(text)
        self.ros_node.publish_start_calibration(blimp_id, active=False)

    def _refresh_calibration_plot(self):
        state_data, covar_data = self.ros_node.get_latest_calib_data()

        for aid, state in state_data.items():
            n = len(state)
            if n not in (4, 5):
                continue
            covar = covar_data.get(aid, [])
            if n == 5 and len(covar) >= 25:
                sigma = [np.sqrt(max(covar[i * 5 + i], 0.0)) for i in range(5)]
            elif n == 4 and len(covar) >= 16:
                sigma = [np.sqrt(max(covar[i * 4 + i], 0.0)) for i in range(4)]
            else:
                sigma = [0.0] * n
            if aid not in self._calib_history:
                self._calib_history[aid] = {
                    'state': deque(maxlen=200),
                    'sigma': deque(maxlen=200),
                    'n_state': n,
                }
            else:
                entry = self._calib_history[aid]
                if entry['n_state'] != n:
                    entry['state'].clear()
                    entry['sigma'].clear()
                    entry['n_state'] = n
            self._calib_history[aid]['state'].append(list(state[:n]))
            self._calib_history[aid]['sigma'].append(sigma)

        if not self._calib_history:
            return

        for ax in self._calib_axes:
            ax.cla()
            ax.grid(True)

        def _hist_n_state(h):
            if h['state']:
                return len(h['state'][-1])
            return h.get('n_state', 5)

        show_v = any(
            h['state'] and len(h['state'][-1]) == 5
            for h in self._calib_history.values()
        )
        self._calib_axes[2].set_visible(show_v)

        _STATE_LABELS = ["z (m)", "ż (m/s)", "V (m³)", "Kv", "Cd"]
        for ax, label in zip(self._calib_axes, _STATE_LABELS):
            ax.set_ylabel(label)

        # 4-state layout: [z, ż, Kv, Cd] -> subplot rows 0,1,3,4 (skip V)
        _PLOT_ROWS_4 = ((0, 0), (1, 1), (2, 3), (3, 4))

        for aid, hist in self._calib_history.items():
            if not hist['state']:
                continue
            n = _hist_n_state(hist)
            states = np.array(hist['state'])
            sigmas = np.array(hist['sigma'])
            xs = np.arange(len(states))
            if n == 5:
                rows = [(i, i) for i in range(5)]
            else:
                rows = _PLOT_ROWS_4
            for si, ri in rows:
                ax = self._calib_axes[ri]
                ax.plot(xs, states[:, si], label=f"blimp {aid}")
                ax.fill_between(
                    xs,
                    states[:, si] - 2 * sigmas[:, si],
                    states[:, si] + 2 * sigmas[:, si],
                    alpha=0.25,
                )

        if len(self._calib_history) > 1:
            self._calib_axes[0].legend(fontsize=7, loc="upper right")

        self._calib_axes[-1].set_xlabel("Sample")
        self._calib_canvas.draw_idle()

    # -- combo helpers --

    def _make_combo(self, options):
        combo = QComboBox()
        combo.setEditable(False)
        combo.addItem("")
        combo.addItems(options)
        # Track when the user explicitly selects the empty slot so _refresh_options
        # doesn't override that choice.  Signals are blocked during _update_combo,
        # so only real user interactions (or unblocked programmatic sets) reach here.
        combo.currentIndexChanged.connect(
            lambda idx, c=combo: setattr(c, "_user_cleared", idx == 0)
        )
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
            if (not id_combo.currentText()
                    and not getattr(id_combo, "_user_cleared", False)
                    and optitrack_idx < len(assigned_ids)):
                idx = id_combo.findText(assigned_ids[optitrack_idx])
                if idx >= 0:
                    id_combo.setCurrentIndex(idx)
            if (not port_combo.currentText()
                    and not getattr(port_combo, "_user_cleared", False)
                    and optitrack_idx < len(ports)):
                idx = port_combo.findText(ports[optitrack_idx])
                if idx >= 0:
                    port_combo.setCurrentIndex(idx)
            if (not goal_combo.currentText()
                    and not getattr(goal_combo, "_user_cleared", False)
                    and optitrack_idx < len(leftover_ids)):
                idx = goal_combo.findText(leftover_ids[optitrack_idx])
                if idx >= 0:
                    goal_combo.setCurrentIndex(idx)
            optitrack_idx += 1

        self._sync_teleop_combos()

    def _sync_teleop_combos(self):
        latest = self.ros_node.get_latest_teleop()
        if latest is None:
            return
        blimp_id, mode = latest
        id_str = str(blimp_id)
        idx = self.teleop_blimp_combo.findText(id_str)
        if idx >= 0 and self.teleop_blimp_combo.currentIndex() != idx:
            self.teleop_blimp_combo.blockSignals(True)
            self.teleop_blimp_combo.setCurrentIndex(idx)
            self.teleop_blimp_combo.blockSignals(False)
        mode_text = next((k for k, v in self.mode_map.items() if v == mode), None)
        if mode_text is not None:
            m_idx = self.teleop_mode_combo.findText(mode_text)
            if m_idx >= 0 and self.teleop_mode_combo.currentIndex() != m_idx:
                self.teleop_mode_combo.blockSignals(True)
                self.teleop_mode_combo.setCurrentIndex(m_idx)
                self.teleop_mode_combo.blockSignals(False)

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

        # Populate calibration dropdown: "All" + each id, and (re)subscribe to calib topics
        self.calib_blimp_combo.blockSignals(True)
        self.calib_blimp_combo.clear()
        self.calib_blimp_combo.addItem("All")
        self.calib_blimp_combo.addItems(id_strs)
        self.calib_blimp_combo.blockSignals(False)
        self.ros_node.setup_calibration_topics(ids)
        self._calib_history.clear()

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
