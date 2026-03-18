import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from blimp_clean.low_level_controller import ControllerNode

class AgentManager(object):
    def __init__(self):
        self.executor = MultiThreadedExecutor()
        self.agents = {}
        self._thread = None

    def initialize_blimps(self, ids, coms, goals):
        for id_, com in zip(ids, coms):
            self.spawn_agent(id_, com)
        self.start()  # spin only after all blimps are registered

    def spawn_agent(self, agent_id: int, com_port: str):
        node = ControllerNode(f'agent_{agent_id}', com_port)
        self.agents[agent_id] = node
        self.executor.add_node(node)

    def destroy_agent(self, agent_id: str):
        node = self.agents.pop(agent_id)
        self.executor.remove_node(node)
        node.destroy_node()

    def start(self):
        """Start executor in background thread after blimps are initialized"""
        print("Starting executor")
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def shutdown(self):
        """Clean up all nodes and stop executor"""
        self.executor.shutdown()
        for node in self.agents.values():
            node.destroy_node()
        self.agents.clear()
        rclpy.shutdown()