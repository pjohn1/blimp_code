import re
import sys
from typing import List, Tuple
from glob import glob

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter as RosParameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
from blimp_msgs.msg import GoalMsg

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QGroupBox,
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
        self.test_timer = None
        self.test_active_agents = []
        self.test_end_time = None
        self.test_vertical_speed = 0.25

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

    def _goal_publisher(self, agent_name: str):
        topic = f"/{agent_name}/controller/goal"
        if topic not in self.goal_publishers:
            self.goal_publishers[topic] = self.create_publisher(GoalMsg, topic, 5)
        return self.goal_publishers[topic]

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

    def start_upward_test(self, agent_names: List[str], duration_sec: float = 2.0, uz: float = 0.25):
        if self.test_timer is not None:
            self.test_timer.cancel()
            self.destroy_timer(self.test_timer)
            self.test_timer = None
        self.test_active_agents = list(agent_names)
        self.test_vertical_speed = float(uz)
        self.test_end_time = self.get_clock().now().nanoseconds / 1e9 + float(duration_sec)
        self.test_timer = self.create_timer(0.05, self._test_timer_cb)

    def _test_timer_cb(self):
        now_s = self.get_clock().now().nanoseconds / 1e9
        running = now_s < self.test_end_time
        uz = self.test_vertical_speed if running else 0.0
        for agent_name in self.test_active_agents:
            self._publish_vertical_command(agent_name, uz)
        if not running and self.test_timer is not None:
            self.test_timer.cancel()
            self.destroy_timer(self.test_timer)
            self.test_timer = None


class SetupGuiWindow(QMainWindow):
    def __init__(self, ros_node: SetupGuiNode):
        super().__init__()
        self.ros_node = ros_node
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
        self.agents_table.horizontalHeader().setStretchLastSection(True)
        agents_layout.addWidget(self.agents_table)
        row_controls = QHBoxLayout()
        self.add_agent_row_btn = QPushButton("Add Agent Row")
        self.remove_agent_row_btn = QPushButton("Remove Agent Row")
        self.autofill_ids_btn = QPushButton("Fill IDs Sequentially")
        self.autopopulate_btn = QPushButton("Auto Populate (OptiTrack + /dev/ttyUSB)")
        row_controls.addWidget(self.add_agent_row_btn)
        row_controls.addWidget(self.remove_agent_row_btn)
        row_controls.addWidget(self.autofill_ids_btn)
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

        apply_group = QGroupBox("Apply Configuration")
        apply_layout = QVBoxLayout()
        self.apply_btn = QPushButton("Apply Configuration")
        self.apply_btn.setMinimumHeight(36)
        apply_layout.addWidget(self.apply_btn)
        self.test_up_btn = QPushButton("Test Configuration (2s Upward Velocity)")
        self.test_up_btn.setMinimumHeight(36)
        apply_layout.addWidget(self.test_up_btn)
        apply_group.setLayout(apply_layout)

        tab_layout.addWidget(agents_group)
        tab_layout.addWidget(goals_group)
        tab_layout.addWidget(apply_group)

        tabs.addTab(id_mapping_tab, "ID Mapping")
        root_layout.addWidget(tabs)
        self.setCentralWidget(root)

        self._seed_defaults()
        self.add_agent_row_btn.clicked.connect(self._add_agent_row)
        self.remove_agent_row_btn.clicked.connect(self._remove_agent_row)
        self.autofill_ids_btn.clicked.connect(self._fill_ids_sequentially)
        self.autopopulate_btn.clicked.connect(self._autopopulate_agents)
        self.add_goal_row_btn.clicked.connect(self._add_goal_row)
        self.remove_goal_row_btn.clicked.connect(self._remove_goal_row)
        self.apply_btn.clicked.connect(self._apply_configuration)
        self.test_up_btn.clicked.connect(self._run_upward_test)
        self._autopopulate_agents()

    def _seed_defaults(self):
        # Keep startup state blank until auto-discovery finds data.
        self.agents_table.setRowCount(1)
        self.agents_table.setItem(0, 0, QTableWidgetItem(""))
        self.agents_table.setItem(0, 1, QTableWidgetItem(""))

        self.goals_table.setRowCount(1)
        self.goals_table.setItem(0, 0, QTableWidgetItem(""))
        self.goals_table.setItem(0, 1, QTableWidgetItem(""))

    def _add_agent_row(self):
        self.agents_table.insertRow(self.agents_table.rowCount())

    def _remove_agent_row(self):
        rows = self.agents_table.rowCount()
        if rows > 1:
            self.agents_table.removeRow(rows - 1)

    def _fill_ids_sequentially(self):
        for i in range(self.agents_table.rowCount()):
            self.agents_table.setItem(i, 1, QTableWidgetItem(str(i)))

    @staticmethod
    def _numeric_suffix(text: str) -> int:
        match = re.search(r"(\d+)$", text)
        return int(match.group(1)) if match else 10**9

    def _discover_linux_usb_ports(self) -> List[str]:
        ports = sorted(glob("/dev/ttyUSB*"), key=self._numeric_suffix)
        return [p for p in ports if p.startswith("/dev/ttyUSB")]

    def _discover_optitrack_agent_ids(self) -> List[int]:
        discovered = set()
        for topic_name, _topic_types in self.ros_node.get_topic_names_and_types():
            match = re.match(r"^/agent_(\d+)/optitrack_node/pose$", topic_name)
            if match:
                discovered.add(int(match.group(1)))
        return sorted(discovered)

    def _autopopulate_agents(self):
        ports = self._discover_linux_usb_ports()
        ids = self._discover_optitrack_agent_ids()

        if not ports and not ids:
            self.agents_table.setRowCount(1)
            self.agents_table.setItem(0, 0, QTableWidgetItem(""))
            self.agents_table.setItem(0, 1, QTableWidgetItem(""))
            return

        row_count = max(1, len(ports), len(ids))
        self.agents_table.setRowCount(row_count)
        for row in range(row_count):
            port = ports[row] if row < len(ports) else ""
            aid = str(ids[row]) if row < len(ids) else ""
            self.agents_table.setItem(row, 0, QTableWidgetItem(port))
            self.agents_table.setItem(row, 1, QTableWidgetItem(aid))

    def _add_goal_row(self):
        self.goals_table.insertRow(self.goals_table.rowCount())

    def _remove_goal_row(self):
        rows = self.goals_table.rowCount()
        if rows > 1:
            self.goals_table.removeRow(rows - 1)

    @staticmethod
    def _table_text(table: QTableWidget, row: int, col: int) -> str:
        item = table.item(row, col)
        return "" if item is None else item.text().strip()

    def _collect_agents(self) -> List[str]:
        agents = []
        seen_ports = set()
        seen_agents = set()
        for row in range(self.agents_table.rowCount()):
            port = self._table_text(self.agents_table, row, 0)
            aid = self._table_text(self.agents_table, row, 1)
            if not port and not aid:
                continue
            if not port or not aid:
                raise ValueError(f"Agents row {row + 1} must include both port and ID.")
            agent_name = _agent_name_from_id(aid)
            if port in seen_ports:
                raise ValueError(f"Duplicate port '{port}' in agents table.")
            if agent_name in seen_agents:
                raise ValueError(f"Duplicate agent '{agent_name}' in agents table.")
            seen_ports.add(port)
            seen_agents.add(agent_name)
            agents.extend([port, agent_name])
        if not agents:
            raise ValueError("At least one agent mapping is required.")
        return agents

    def _collect_goals(self) -> List[str]:
        goals = []
        for row in range(self.goals_table.rowCount()):
            goal_id = self._table_text(self.goals_table, row, 0)
            target_id = self._table_text(self.goals_table, row, 1)
            if not goal_id and not target_id:
                continue
            if not goal_id or not target_id:
                raise ValueError(f"Goals row {row + 1} must include both goal and target IDs.")
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

        updates = [
            (serial_node, "agents", agents),
            (opti_node, "agents", agents),
            (cbf_node, "agents", agents),
        ]
        if goals:
            updates.extend(
                [
                    (opti_node, "goals", goals),
                    (cbf_node, "goals", goals),
                ]
            )

        errors = []
        for target_node, name, values in updates:
            ok, err = self.ros_node.set_string_array_parameter(target_node, name, values)
            if not ok:
                errors.append(err)

        if errors:
            QMessageBox.critical(self, "Apply Failed", "\n".join(errors))
            return
        QMessageBox.information(self, "Success", "Configuration applied to running nodes.")

    def _run_upward_test(self):
        try:
            agents = self._collect_agents()
        except ValueError as exc:
            QMessageBox.critical(self, "Invalid Configuration", str(exc))
            return
        agent_names = [agents[i + 1] for i in range(0, len(agents), 2)]
        self.ros_node.start_upward_test(agent_names, duration_sec=2.0, uz=0.25)
        QMessageBox.information(
            self,
            "Test Started",
            f"Sending upward velocity command to {len(agent_names)} agent(s) for 2 seconds.",
        )


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
