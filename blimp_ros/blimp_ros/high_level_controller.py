import rclpy
from rclpy.node import Node
from blimp_msgs.msg import OptiTrackPose, GoalMsg

import numpy as np

MAX_VOLTAGE = 0.6
NOMIMAL_V = 1.0

class HighLevelController(Node):
    def __init__(self):

        ## ROS NODE PARAMS #################
        super().__init__("high_level_controller")

        self.declare_parameter('goal',[0.0]*12)
        #x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz
        self.goal = self.get_parameter('goal').value

        self.ns = self.get_namespace()
        self.publisher = self.create_publisher(GoalMsg,f"{self.ns}/controller/goal",2)

        self.subscriber1 = self.create_subscription(GoalMsg,f"{self.ns}/high_level_controller/goal",self.update_goal,5)
        self.subscriber2 = self.create_subscription(OptiTrackPose,f"{self.ns}/optitrack_node/pose",self.controller,5)

    def publish_goal(self,goal_values):
        msg = GoalMsg()
        msg.x = goal_values[0]
        msg.y = goal_values[1]
        msg.z = goal_values[2]
        msg.roll = goal_values[3]
        msg.pitch = goal_values[4]
        msg.yaw = goal_values[5]
        msg.ux = goal_values[6]
        msg.uy = goal_values[7]
        msg.uz = goal_values[8]
        msg.wx = goal_values[9]
        msg.wy = goal_values[10]
        msg.wz = goal_values[11]
        self.publisher.publish(msg)


    def update_goal(self,msg):
        goal_values = self.goal.copy()
        goal_values[0] = msg.x
        goal_values[1] = msg.y
        self.publish_goal(np.array(goal_values,dtype=np.float64))
        self.goal = goal_values
    
    def controller(self,msg):
        p = np.array([msg.x,msg.y,msg.z])
        goal = np.array(goal_values[0:2])
        u0 = goal - p


def main(args=None):
    rclpy.init(args=args)
    high_level_controller = HighLevelController()
    rclpy.spin(high_level_controller)
    high_level_controller.destroy_node()

if __name__ == '__main__':
    main()