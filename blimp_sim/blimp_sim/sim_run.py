import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
import transforms3d.euler as euler

import numpy as np
import threading
import time

class RunSim(Node):
    def __init__(self):
        super().__init__('node')
        self.publisher = self.create_publisher(Float32MultiArray,'/sim/robot_pos',4)
        self.subscriber = self.create_subscription(Float32MultiArray,'/controls/control_update',self.update_control,5)
        self.declare_parameter('num_blimps',2)
        self.num_blimps = self.get_parameter('num_blimps').value
        self.control = np.array([[0.0,0.0,0.0,0.0,0.0,0.0]*self.num_blimps]).flatten()
        self.rate = self.create_rate(30.0)
        self.declare_parameter('map_size',[10.0,10.0])
        self.map_size = self.get_parameter('map_size').value

        self.running = True
        self.create_subscription(PointStamped,'/clicked_point',self.start,1)

        self.declare_parameter('blimp_positions',list(np.array([[float(i),0.0,0.0,0.0,0.0,0.0] for i in range(self.num_blimps)]).flatten()))
        self.positions = self.get_parameter('blimp_positions').value
        self.last_t = self.get_clock().now().nanoseconds/1e9

        self.declare_parameter('dt',0.05)
        self.dt = self.get_parameter('dt').value

        self.last=None

        self.thread = threading.Thread(target=self.run_sim)
    
    def start(self,msg):
        self.thread.start()

    def update_control(self,msg):
        self.control = msg.data

    def run_sim(self):
        def in_bounds(x,y,z):
            width,height = self.map_size
            if x <= width and x>=0.0 and y<=width and y>=0.0 and z<=height and z>=0.0:
                return True
            return False


        while True:
            t = self.get_clock().now().nanoseconds/1e9
            # dt = t - self.last_t
            dt = self.dt
            if dt > 0 and abs(dt) < 0.1:
                i=0
                while i < len(self.control):
                    vx,vy,vz,wx,wy,wz = self.control[i:i+6]
                    if self.last is not None:
                        self.get_logger().info(f'{np.linalg.norm(np.array(self.last) - np.array(self.control[i:i+6]))}')
                    self.last = self.control[i:i+6]

                    
                    posx = self.positions[i] + vx*dt
                    posy = self.positions[i+1] + vy*dt
                    posz = self.positions[i+2] + vz*dt

                    if in_bounds(posx,posy,posz):
                        self.positions[i] = posx
                        self.positions[i+1] = posy
                        self.positions[i+2] = posz
                    
                    yaw = np.arctan2(vy,vx)
                    self.positions[i+5] = yaw
                    i+=6

                msg = Float32MultiArray()
                msg.data = self.positions
                self.publisher.publish(msg)
                # self.rate.sleep()
            self.last_t = t
            time.sleep(self.dt)


def main(args=None):
    rclpy.init(args=args)
    sim = RunSim()
    try:
        rclpy.spin(sim)
    except KeyboardInterrupt:
        pass
    sim.thread.join(timeout=2.0)
    sim.destroy_node()
    rclpy.shutdown()
    
