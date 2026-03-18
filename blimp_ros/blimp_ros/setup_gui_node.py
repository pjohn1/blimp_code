########################################################
# setup_gui_node.py
#
# Generated using GenAI, GUI for configuring the blimps
# and tracking telemtry, teleop, etc.
########################################################

import json
import os
import re
import sys
from typing import List, Tuple
from glob import glob
import threading

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter as RosParameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
from blimp_msgs.msg import GoalMsg
from std_msgs.msg import Int32, Bool
from std_msgs.msg import Float32MultiArray

from blimp_msgs.msg import OptiTrackPose

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QGroupBox,
    QHeaderView,
    QHBoxLayout,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


AGENT_PATTERN = re.compile(r"^agent_(\d+)$")
DEFAULT_TARGET_NODES = ("/serial_node", "/optitrack_node", "/cbf")
DEFAULT_TELEOP_NODE = "/teleop_node"
CONFIG_PATH = os.path.expanduser("~/.blimp_setup_config.json")


def _agent_name_from_id(raw_id: str) -> str:
    raw_id = str(raw_id).strip()
    if raw_id.startswith("agent_"):
        return raw_id
    if not raw_id.isdigit():
        raise ValueError(f"Invalid agent id '{raw_id}'")
    return f"agent_{int(raw_id)}"


class SetupGuiNode(Node):
    def __init__(self):
        super().__init__("setup_gui_node")
        self.set_clients = {}
        self.goal_publishers = {}
        self.serial_publishers = {}
        self.stop_publishers = {}
        self.id_lock = threading.Lock()
        self.discovered_ids = set()
        self.test_timer = None
        self.test_active_agents = []
        self.test_tick_count = 0
        self.test_max_ticks = 20
        self.test_stop_ticks = 5
        self.test_stop_tick_count = 0
        self.test_vertical_speed = 0.25
        self.test_publish_count = 0
        self.test_last_log_tick = 0
        self.id_sub = self.create_subscription(
            Int32, "/optitrack_node/discovered_id", self._discovered_id_cb, 50
        )
        self.pose_lock = threading.Lock()
        self.latest_poses = {}
        self.pose_subs = {}

    def set_string_array_parameter(
        self, target_node: str, parameter_name: str, values: List[str], timeout_sec: float = 2.0
    ) -> Tuple[bool, str]:
        if target_node not in self.set_clients:
            self.set_clients[target_node] = self.create_client(
                SetParameters, f"{target_node}/set_parameters"
            )
        client = self.set_clients[target_node]
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, f"Service unavailable: {target_node}/set_parameters"

        p = RosParameter()
        p.name = parameter_name
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=list(values)
        )
        req = SetParameters.Request()
        req.parameters = [p]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return False, f"Timeout while setting {parameter_name} on {target_node}"
        res = future.result()
        if not res.results:
            return False, f"No response for {parameter_name} on {target_node}"
        if not res.results[0].successful:
            reason = res.results[0].reason or "unknown reason"
            return False, f"Rejected {parameter_name} on {target_node}: {reason}"
        return True, ""

    def set_string_parameter(
        self, target_node: str, parameter_name: str, value: str, timeout_sec: float = 2.0
    ) -> Tuple[bool, str]:
        if target_node not in self.set_clients:
            self.set_clients[target_node] = self.create_client(
                SetParameters, f"{target_node}/set_parameters"
            )
        client = self.set_clients[target_node]
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, f"Service unavailable: {target_node}/set_parameters"

        p = RosParameter()
        p.name = parameter_name
        p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(value))
        req = SetParameters.Request()
        req.parameters = [p]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return False, f"Timeout while setting {parameter_name} on {target_node}"
        res = future.result()
        if not res.results:
            return False, f"No response for {parameter_name} on {target_node}"
        if not res.results[0].successful:
            reason = res.results[0].reason or "unknown reason"
            return False, f"Rejected {parameter_name} on {target_node}: {reason}"
        return True, ""

    def set_bool_parameter(
        self, target_node: str, parameter_name: str, value: bool, timeout_sec: float = 2.0
    ) -> Tuple[bool, str]:
        if target_node not in self.set_clients:
            self.set_clients[target_node] = self.create_client(
                SetParameters, f"{target_node}/set_parameters"
            )
        client = self.set_clients[target_node]
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, f"Service unavailable: {target_node}/set_parameters"

        p = RosParameter()
        p.name = parameter_name
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(value))
        req = SetParameters.Request()
        req.parameters = [p]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return False, f"Timeout while setting {parameter_name} on {target_node}"
        res = future.result()
        if not res.results:
            return False, f"No response for {parameter_name} on {target_node}"
        if not res.results[0].successful:
            reason = res.results[0].reason or "unknown reason"
            return False, f"Rejected {parameter_name} on {target_node}: {reason}"
        return True, ""

    def low_level_node_paths(self) -> List[str]:
        nodes = []
        for node_name, namespace in self.get_node_names_and_namespaces():
            if node_name != "low_level_controller":
                continue
            ns = namespace if namespace.startswith("/") else f"/{namespace}"
            full_path = f"{ns}/{node_name}".replace("//", "/")
            nodes.append(full_path)
        return sorted(nodes)

    def teleop_node_path(self) -> str:
        for node_name, namespace in self.get_node_names_and_namespaces():
            if node_name != "teleop_node":
                continue
            ns = namespace if namespace.startswith("/") else f"/{namespace}"
            return f"{ns}/{node_name}".replace("//", "/")
        return DEFAULT_TELEOP_NODE

    def _discovered_id_cb(self, msg: Int32):
        if msg.data < 0:
            return
        aid = int(msg.data)
        with self.id_lock:
            self.discovered_ids.add(aid)
        if aid not in self.pose_subs:
            agent_name = f"agent_{aid}"
            topic = f"/{agent_name}/optitrack_node/pose"
            self.pose_subs[aid] = self.create_subscription(
                OptiTrackPose, topic, lambda m, a=aid: self._pose_cb(a, m), 10
            )
            self.get_logger().info(f"Subscribed to pose: {topic}")

    def _pose_cb(self, agent_id: int, msg: OptiTrackPose):
        with self.pose_lock:
            self.latest_poses[agent_id] = msg

    def get_discovered_ids(self) -> List[int]:
        with self.id_lock:
            return sorted(self.discovered_ids)

    def get_latest_poses(self) -> dict:
        with self.pose_lock:
            return dict(self.latest_poses)

    def _goal_publisher(self, agent_name: str):
        topic = f"/{agent_name}/controller/goal"
        if topic not in self.goal_publishers:
            self.goal_publishers[topic] = self.create_publisher(GoalMsg, topic, 5)
        return self.goal_publishers[topic]

    def _serial_publisher(self, agent_name: str):
        topic = f"/{agent_name}/serial/voltages"
        if topic not in self.serial_publishers:
            self.serial_publishers[topic] = self.create_publisher(Float32MultiArray, topic, 5)
        return self.serial_publishers[topic]

    def _stop_publisher(self, agent_name: str):
        topic = f"/{agent_name}/controller/stop"
        if topic not in self.stop_publishers:
            self.stop_publishers[topic] = self.create_publisher(Bool, topic, 5)
        return self.stop_publishers[topic]

    def _publish_vertical_command(self, agent_name: str, uz: float):
        msg = GoalMsg()
        msg.id = int(agent_name.split("_")[-1])
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 1.5
        msg.roll = 0.0
        msg.pitch = 0.0
        msg.yaw = 0.0
        msg.ux = 0.0
        msg.uy = 0.0
        msg.uz = float(uz)
        msg.wx = 0.0
        msg.wy = 0.0
        msg.wz = 0.0
        self._goal_publisher(agent_name).publish(msg)
        # Also publish directly to serial/voltages so motor writes occur even when
        # low_level_controller is not running. Motors 2,3 are vertical (altitude).
        uid = int(agent_name.split("_")[-1])
        m2 = min(0.9, max(0.0, float(uz)))
        m3 = max(-0.9, min(0.0, -float(uz)))
        serial_msg = Float32MultiArray()
        serial_msg.data = [0.0, 0.0, m2, m3, 0.0, 0.0, float(uid)]
        self._serial_publisher(agent_name).publish(serial_msg)

    def _publish_serial_stop(self, agent_name: str):
        stop_msg = Float32MultiArray()
        uid = int(agent_name.split("_")[-1])
        # serial_node expects 6 motor commands + agent id at the end
        stop_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(uid)]
        self._serial_publisher(agent_name).publish(stop_msg)

    def _publish_controller_stop(self, agent_name: str):
        stop_msg = Bool()
        stop_msg.data = True
        self._stop_publisher(agent_name).publish(stop_msg)

    def start_upward_test(self, agent_names: List[str], duration_sec: float = 2.0, uz: float = 0.25):
        if self.test_timer is not None:
            self.test_timer.cancel()
            self.destroy_timer(self.test_timer)
            self.test_timer = None
        self.test_active_agents = list(agent_names)
        self.test_vertical_speed = float(uz)
        # 50 Hz timer -> about 2s when max_ticks=100.
        self.test_max_ticks = max(1, int(round(float(duration_sec) * 50.0)))
        self.test_tick_count = 0
        self.test_stop_tick_count = 0
        self.test_publish_count = 0
        self.test_last_log_tick = 0
        self.get_logger().info(
            f"[TEMP TEST] Starting upward test: agents={self.test_active_agents}, "
            f"duration={duration_sec:.2f}s, ticks={self.test_max_ticks}, uz={self.test_vertical_speed:.3f}"
        )
        self.test_timer = self.create_timer(0.05, self._test_timer_cb)

    def _test_timer_cb(self):
        self.test_tick_count += 1
        running = self.test_tick_count <= self.test_max_ticks
        uz = self.test_vertical_speed if running else 0.0
        for agent_name in self.test_active_agents:
            self._publish_vertical_command(agent_name, uz)
            self.test_publish_count += 1

        if running and (self.test_tick_count - self.test_last_log_tick >= 25):
            self.get_logger().info(
                f"[TEMP TEST] Publishing fly command: agents={len(self.test_active_agents)}, "
                f"uz={uz:.3f}, tick={self.test_tick_count}/{self.test_max_ticks}, "
                f"sends={self.test_publish_count}"
            )
            self.test_last_log_tick = self.test_tick_count

        if not running and self.test_timer is not None:
            for agent_name in self.test_active_agents:
                self._publish_serial_stop(agent_name)
                self._publish_controller_stop(agent_name)
                self.test_publish_count += 1
            self.test_stop_tick_count += 1
            if self.test_stop_tick_count == 1:
                self.get_logger().info(
                    f"[TEMP TEST] Test complete. Sending {self.test_stop_ticks} direct serial stop bursts."
                )
            if self.test_stop_tick_count >= self.test_stop_ticks:
                self.get_logger().info(
                    f"[TEMP TEST] Stop burst complete. "
                    f"ticks={self.test_tick_count}, total_sends={self.test_publish_count}"
                )
                self.test_timer.cancel()
                self.destroy_timer(self.test_timer)
                self.test_timer = None


class SetupGuiWindow(QMainWindow):
    def __init__(self, ros_node: SetupGuiNode):
        super().__init__()
        self.ros_node = ros_node
        self.agent_port_combos = []
        self.agent_id_combos = []
        self.goal_id_combos = []
        self.goal_target_combos = []
        self.setWindowTitle("Blimp Setup GUI")
        self.resize(920, 640)
        self._build_ui()

    def _build_ui(self):
        root = QWidget()
        root_layout = QVBoxLayout(root)
        tabs = QTabWidget()

        id_mapping_tab = QWidget()
        tab_layout = QVBoxLayout(id_mapping_tab)

        agents_group = QGroupBox("Agents (serial_port <-> agent_id)")
        agents_layout = QVBoxLayout()
        self.agents_table = QTableWidget(2, 2)
        self.agents_table.setHorizontalHeaderLabels(["Serial Port", "Agent ID"])
        self.agents_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.agents_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.agents_table.setColumnWidth(1, 140)
        agents_layout.addWidget(self.agents_table)
        row_controls = QHBoxLayout()
        self.add_agent_row_btn = QPushButton("Add Agent Row")
        self.remove_agent_row_btn = QPushButton("Remove Agent Row")
        self.autopopulate_btn = QPushButton("Refresh Detected USB/OptiTrack")
        row_controls.addWidget(self.add_agent_row_btn)
        row_controls.addWidget(self.remove_agent_row_btn)
        row_controls.addWidget(self.autopopulate_btn)
        agents_layout.addLayout(row_controls)
        agents_group.setLayout(agents_layout)

        goals_group = QGroupBox("Goal Map (goal_id -> target_agent)")
        goals_layout = QVBoxLayout()
        self.goals_table = QTableWidget(2, 2)
        self.goals_table.setHorizontalHeaderLabels(["Goal Agent ID", "Target Agent ID"])
        self.goals_table.horizontalHeader().setStretchLastSection(True)
        goals_layout.addWidget(self.goals_table)
        goal_controls = QHBoxLayout()
        self.add_goal_row_btn = QPushButton("Add Goal Row")
        self.remove_goal_row_btn = QPushButton("Remove Goal Row")
        goal_controls.addWidget(self.add_goal_row_btn)
        goal_controls.addWidget(self.remove_goal_row_btn)
        goals_layout.addLayout(goal_controls)
        goals_group.setLayout(goals_layout)

        teleop_group = QGroupBox("Teleoperation")
        teleop_layout = QHBoxLayout()
        self.teleop_agent_combo = self._build_combo([])
        self.teleop_start_btn = QPushButton("Start Teleop")
        self.teleop_stop_btn = QPushButton("Stop Teleop")
        teleop_layout.addWidget(self.teleop_agent_combo)
        teleop_layout.addWidget(self.teleop_start_btn)
        teleop_layout.addWidget(self.teleop_stop_btn)
        teleop_group.setLayout(teleop_layout)

        apply_group = QGroupBox("Apply Configuration")
        apply_layout = QVBoxLayout()
        self.apply_btn = QPushButton("Apply Configuration")
        self.apply_btn.setMinimumHeight(36)
        apply_layout.addWidget(self.apply_btn)
        self.test_up_btn = QPushButton("Test Configuration (2s Upward Velocity)")
        self.test_up_btn.setMinimumHeight(36)
        apply_layout.addWidget(self.test_up_btn)
        config_btns = QHBoxLayout()
        self.save_config_btn = QPushButton("Save Configuration")
        self.load_config_btn = QPushButton("Load Configuration")
        config_btns.addWidget(self.save_config_btn)
        config_btns.addWidget(self.load_config_btn)
        apply_layout.addLayout(config_btns)
        apply_group.setLayout(apply_layout)

        tab_layout.addWidget(agents_group)
        tab_layout.addWidget(goals_group)
        tab_layout.addWidget(teleop_group)
        tab_layout.addWidget(apply_group)

        tabs.addTab(id_mapping_tab, "ID Mapping")

        tracking_tab = QWidget()
        tracking_layout = QVBoxLayout(tracking_tab)
        self.tracking_table = QTableWidget(0, 7)
        self.tracking_table.setHorizontalHeaderLabels(
            ["Agent ID", "X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        )
        self.tracking_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tracking_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.tracking_table.setSelectionBehavior(QTableWidget.SelectRows)
        tracking_layout.addWidget(self.tracking_table)
        tabs.addTab(tracking_tab, "Tracking")

        root_layout.addWidget(tabs)
        self.setCentralWidget(root)

        self._seed_defaults()
        self.add_agent_row_btn.clicked.connect(self._add_agent_row)
        self.remove_agent_row_btn.clicked.connect(self._remove_agent_row)
        self.autopopulate_btn.clicked.connect(self._autopopulate_agents)
        self.add_goal_row_btn.clicked.connect(self._add_goal_row)
        self.remove_goal_row_btn.clicked.connect(self._remove_goal_row)
        self.apply_btn.clicked.connect(self._apply_configuration)
        self.test_up_btn.clicked.connect(self._run_upward_test)
        self.save_config_btn.clicked.connect(self._save_configuration)
        self.load_config_btn.clicked.connect(self._load_configuration)
        self.teleop_start_btn.clicked.connect(self._start_teleop)
        self.teleop_stop_btn.clicked.connect(self._stop_teleop)
        self._autopopulate_agents()
        self.live_update_timer = QTimer(self)
        self.live_update_timer.timeout.connect(self._autopopulate_agents)
        self.live_update_timer.start(500)

        self.tracking_timer = QTimer(self)
        self.tracking_timer.timeout.connect(self._refresh_tracking)
        self.tracking_timer.start(200)

    def _seed_defaults(self):
        # Keep startup state blank until auto-discovery finds data.
        self.agents_table.setRowCount(0)
        self.goals_table.setRowCount(0)
        self._add_agent_row()
        self._add_goal_row()

    @staticmethod
    def _build_combo(options: List[str], selected: str = "") -> QComboBox:
        combo = QComboBox()
        combo.setEditable(False)
        combo.addItem("")
        combo.addItems(options)
        if selected:
            idx = combo.findText(selected)
            if idx >= 0:
                combo.setCurrentIndex(idx)
        return combo

    @staticmethod
    def _set_combo_options(combo: QComboBox, options: List[str]):
        current = combo.currentText()
        combo.blockSignals(True)
        combo.clear()
        combo.addItem("")
        combo.addItems(options)
        idx = combo.findText(current)
        combo.setCurrentIndex(idx if idx >= 0 else 0)
        combo.blockSignals(False)

    def _add_agent_row(self):
        row = self.agents_table.rowCount()
        self.agents_table.insertRow(row)
        port_combo = self._build_combo([])
        id_combo = self._build_combo([])
        self.agents_table.setCellWidget(row, 0, port_combo)
        self.agents_table.setCellWidget(row, 1, id_combo)
        self.agent_port_combos.append(port_combo)
        self.agent_id_combos.append(id_combo)
        self._autopopulate_agents()

    def _remove_agent_row(self):
        if self.agents_table.rowCount() <= 1:
            return
        last = self.agents_table.rowCount() - 1
        self.agents_table.removeRow(last)
        self.agent_port_combos.pop()
        self.agent_id_combos.pop()

    @staticmethod
    def _numeric_suffix(text: str) -> int:
        match = re.search(r"(\d+)$", text)
        return int(match.group(1)) if match else 10**9

    def _discover_linux_usb_ports(self) -> List[str]:
        ports = sorted(glob("/dev/ttyUSB*"), key=self._numeric_suffix)
        return [p for p in ports if p.startswith("/dev/ttyUSB")]

    def _discover_optitrack_agent_ids(self) -> List[int]:
        return self.ros_node.get_discovered_ids()

    def _autopopulate_agents(self):
        ports = self._discover_linux_usb_ports()
        ids = self._discover_optitrack_agent_ids()
        id_options = [str(v) for v in ids]

        desired_rows = max(1, len(ports), len(id_options))
        while self.agents_table.rowCount() < desired_rows:
            row = self.agents_table.rowCount()
            self.agents_table.insertRow(row)
            port_combo = self._build_combo([])
            id_combo = self._build_combo([])
            self.agents_table.setCellWidget(row, 0, port_combo)
            self.agents_table.setCellWidget(row, 1, id_combo)
            self.agent_port_combos.append(port_combo)
            self.agent_id_combos.append(id_combo)

        for row in range(self.agents_table.rowCount()):
            port_combo = self.agent_port_combos[row]
            id_combo = self.agent_id_combos[row]
            self._set_combo_options(port_combo, ports)
            self._set_combo_options(id_combo, id_options)
            if not port_combo.currentText() and row < len(ports):
                idx = port_combo.findText(ports[row])
                if idx >= 0:
                    port_combo.setCurrentIndex(idx)
            if not id_combo.currentText() and row < len(id_options):
                idx = id_combo.findText(id_options[row])
                if idx >= 0:
                    id_combo.setCurrentIndex(idx)

        self._refresh_goal_dropdowns(id_options)
        self._set_combo_options(self.teleop_agent_combo, id_options)

    def _add_goal_row(self):
        row = self.goals_table.rowCount()
        self.goals_table.insertRow(row)
        id_options = [str(v) for v in self._discover_optitrack_agent_ids()]
        goal_combo = self._build_combo(id_options)
        target_combo = self._build_combo(id_options)
        self.goals_table.setCellWidget(row, 0, goal_combo)
        self.goals_table.setCellWidget(row, 1, target_combo)
        self.goal_id_combos.append(goal_combo)
        self.goal_target_combos.append(target_combo)

    def _remove_goal_row(self):
        if self.goals_table.rowCount() <= 1:
            return
        last = self.goals_table.rowCount() - 1
        self.goals_table.removeRow(last)
        self.goal_id_combos.pop()
        self.goal_target_combos.pop()

    def _refresh_goal_dropdowns(self, id_options: List[str]):
        for row in range(self.goals_table.rowCount()):
            self._set_combo_options(self.goal_id_combos[row], id_options)
            self._set_combo_options(self.goal_target_combos[row], id_options)

    def _collect_agents(self) -> List[str]:
        agents = []
        seen_ports = set()
        seen_agents = set()
        for row in range(self.agents_table.rowCount()):
            port = self.agent_port_combos[row].currentText().strip()
            aid = self.agent_id_combos[row].currentText().strip()
            if not port and not aid:
                continue
            if not port or not aid:
                # Allow partially filled rows; they are ignored.
                continue
            agent_name = _agent_name_from_id(aid)
            if port in seen_ports:
                raise ValueError(f"Duplicate port '{port}' in agents table.")
            if agent_name in seen_agents:
                raise ValueError(f"Duplicate agent '{agent_name}' in agents table.")
            seen_ports.add(port)
            seen_agents.add(agent_name)
            agents.extend([port, agent_name])
        return agents

    def _collect_goals(self) -> List[str]:
        goals = []
        for row in range(self.goals_table.rowCount()):
            goal_id = self.goal_id_combos[row].currentText().strip()
            target_id = self.goal_target_combos[row].currentText().strip()
            if not goal_id and not target_id:
                continue
            if not goal_id or not target_id:
                # Allow partially filled rows; they are ignored.
                continue
            goal_name = _agent_name_from_id(goal_id)
            target_name = _agent_name_from_id(target_id)
            goals.extend([goal_name, target_name])
        return goals

    def _apply_configuration(self):
        try:
            agents = self._collect_agents()
            goals = self._collect_goals()
        except ValueError as exc:
            QMessageBox.critical(self, "Invalid Configuration", str(exc))
            return

        serial_node, opti_node, cbf_node = DEFAULT_TARGET_NODES

        updates = []
        if agents:
            updates.extend(
                [
                    (serial_node, "agents", agents),
                    (opti_node, "agents", agents),
                    (cbf_node, "agents", agents),
                ]
            )
        if goals:
            updates.extend(
                [
                    (opti_node, "goals", goals),
                    (cbf_node, "goals", goals),
                ]
            )
        if not updates:
            QMessageBox.information(
                self,
                "No Updates",
                "No complete agent or goal pairs selected. Nothing was updated.",
            )
            return

        errors = []
        for target_node, name, values in updates:
            ok, err = self.ros_node.set_string_array_parameter(target_node, name, values)
            if not ok:
                errors.append(err)

        # Rebind low-level controller subscriptions/publishers to current agent IDs.
        # low_level nodes are static processes, so we update their agent_name parameter.
        if agents:
            agent_names = [agents[i + 1] for i in range(0, len(agents), 2)]
            low_level_nodes = self.ros_node.low_level_node_paths()
            for idx, node_path in enumerate(low_level_nodes):
                agent_name = agent_names[idx] if idx < len(agent_names) else ""
                ok, err = self.ros_node.set_string_parameter(
                    node_path, "agent_name", agent_name
                )
                if not ok:
                    errors.append(err)

        if errors:
            QMessageBox.critical(self, "Apply Failed", "\n".join(errors))
            return
        try:
            cfg = {"agents": self._collect_agents_raw(), "goals": self._collect_goals_raw()}
            with open(CONFIG_PATH, "w") as f:
                json.dump(cfg, f, indent=2)
        except OSError:
            pass
        QMessageBox.information(self, "Success", "Configuration applied to running nodes.")

    def _run_upward_test(self):
        agents = self._collect_agents()
        agent_names = [agents[i + 1] for i in range(0, len(agents), 2)]
        if not agent_names:
            QMessageBox.information(
                self,
                "No Test Targets",
                "No complete agent row selected for testing.",
            )
            return
        self.ros_node.start_upward_test(agent_names, duration_sec=2.0, uz=0.25)
        QMessageBox.information(
            self,
            "Test Started",
            f"Sending upward velocity command to {len(agent_names)} agent(s) for 2 seconds.",
        )

    def _collect_agents_raw(self) -> List[Tuple[str, str]]:
        """Collect all agent rows (port, agent_id) including partial/empty."""
        rows = []
        for row in range(self.agents_table.rowCount()):
            port = self.agent_port_combos[row].currentText().strip()
            aid = self.agent_id_combos[row].currentText().strip()
            rows.append((port, aid))
        return rows

    def _collect_goals_raw(self) -> List[Tuple[str, str]]:
        """Collect all goal rows (goal_id, target_id) including partial/empty."""
        rows = []
        for row in range(self.goals_table.rowCount()):
            goal_id = self.goal_id_combos[row].currentText().strip()
            target_id = self.goal_target_combos[row].currentText().strip()
            rows.append((goal_id, target_id))
        return rows

    def _save_configuration(self):
        agents_raw = self._collect_agents_raw()
        goals_raw = self._collect_goals_raw()
        cfg = {"agents": agents_raw, "goals": goals_raw}
        try:
            with open(CONFIG_PATH, "w") as f:
                json.dump(cfg, f, indent=2)
            QMessageBox.information(
                self, "Saved", f"Configuration saved to {CONFIG_PATH}"
            )
        except OSError as exc:
            QMessageBox.critical(self, "Save Failed", str(exc))

    def _load_configuration(self):
        if not os.path.isfile(CONFIG_PATH):
            QMessageBox.information(
                self,
                "No Config",
                f"No saved configuration found at {CONFIG_PATH}. Save first.",
            )
            return
        try:
            with open(CONFIG_PATH) as f:
                cfg = json.load(f)
        except (OSError, json.JSONDecodeError) as exc:
            QMessageBox.critical(self, "Load Failed", str(exc))
            return

        agents_raw = cfg.get("agents", [])
        goals_raw = cfg.get("goals", [])
        current_ids = [str(v) for v in self._discover_optitrack_agent_ids()]
        current_id_set = set(current_ids)

        def map_id(saved_id: str, slot: int) -> str:
            """Use saved_id if in current set, else map by slot to current IDs."""
            if saved_id and saved_id in current_id_set:
                return saved_id
            if slot < len(current_ids):
                return current_ids[slot]
            return saved_id or ""

        # Ensure enough agent rows
        while self.agents_table.rowCount() < len(agents_raw):
            self._add_agent_row()
        while self.agents_table.rowCount() > len(agents_raw) and self.agents_table.rowCount() > 1:
            self._remove_agent_row()

        ports = self._discover_linux_usb_ports()
        id_options = current_ids
        for row, (port, saved_aid) in enumerate(agents_raw):
            if row >= self.agents_table.rowCount():
                break
            port_combo = self.agent_port_combos[row]
            id_combo = self.agent_id_combos[row]
            self._set_combo_options(port_combo, ports)
            self._set_combo_options(id_combo, id_options)
            if port:
                idx = port_combo.findText(port)
                if idx >= 0:
                    port_combo.setCurrentIndex(idx)
            mapped_id = map_id(saved_aid, row)
            if mapped_id:
                idx = id_combo.findText(mapped_id)
                if idx >= 0:
                    id_combo.setCurrentIndex(idx)

        # Ensure enough goal rows
        while self.goals_table.rowCount() < len(goals_raw):
            self._add_goal_row()
        while self.goals_table.rowCount() > len(goals_raw) and self.goals_table.rowCount() > 1:
            self._remove_goal_row()

        for row, (saved_goal, saved_target) in enumerate(goals_raw):
            if row >= self.goals_table.rowCount():
                break
            goal_combo = self.goal_id_combos[row]
            target_combo = self.goal_target_combos[row]
            self._set_combo_options(goal_combo, id_options)
            self._set_combo_options(target_combo, id_options)
            mapped_goal = map_id(saved_goal, row)
            mapped_target = map_id(saved_target, (row + 1) % max(1, len(current_ids)))
            if mapped_goal:
                idx = goal_combo.findText(mapped_goal)
                if idx >= 0:
                    goal_combo.setCurrentIndex(idx)
            if mapped_target:
                idx = target_combo.findText(mapped_target)
                if idx >= 0:
                    target_combo.setCurrentIndex(idx)

        self._set_combo_options(self.teleop_agent_combo, id_options)
        QMessageBox.information(
            self, "Loaded",
            f"Configuration loaded from {CONFIG_PATH}. Agent IDs matched to current OptiTrack.",
        )

    def _refresh_tracking(self):
        poses = self.ros_node.get_latest_poses()
        agent_ids = sorted(poses.keys())
        self.tracking_table.setRowCount(len(agent_ids))
        for row, aid in enumerate(agent_ids):
            p = poses[aid]
            values = [str(aid), f"{p.x:.3f}", f"{p.y:.3f}", f"{p.z:.3f}",
                      f"{p.roll:.3f}", f"{p.pitch:.3f}", f"{p.yaw:.3f}"]
            for col, val in enumerate(values):
                item = self.tracking_table.item(row, col)
                if item is None:
                    item = QTableWidgetItem(val)
                    item.setTextAlignment(Qt.AlignCenter)
                    self.tracking_table.setItem(row, col, item)
                else:
                    item.setText(val)

    def _start_teleop(self):
        selected_id = self.teleop_agent_combo.currentText().strip()
        if not selected_id:
            QMessageBox.information(
                self,
                "No Teleop Target",
                "Select an agent ID in the Teleoperation dropdown.",
            )
            return
        target_agent = _agent_name_from_id(selected_id)
        teleop_node = self.ros_node.teleop_node_path()
        ok1, err1 = self.ros_node.set_string_parameter(teleop_node, "target_agent", target_agent)
        ok2, err2 = self.ros_node.set_bool_parameter(teleop_node, "teleop_enabled", True)
        errors = []
        if not ok1:
            errors.append(err1)
        if not ok2:
            errors.append(err2)
        if errors:
            QMessageBox.critical(self, "Teleop Start Failed", "\n".join(errors))
            return
        QMessageBox.information(self, "Teleop", f"Teleop started for {target_agent}.")

    def _stop_teleop(self):
        teleop_node = self.ros_node.teleop_node_path()
        ok, err = self.ros_node.set_bool_parameter(teleop_node, "teleop_enabled", False)
        if not ok:
            QMessageBox.critical(self, "Teleop Stop Failed", err)
            return
        QMessageBox.information(self, "Teleop", "Teleop stopped.")


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
