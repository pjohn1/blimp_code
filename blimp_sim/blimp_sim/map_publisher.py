import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
import transforms3d.euler as euler
import numpy as np
import os
import time

class MapPublisher(Node):
    def __init__(self,
                 width_m=10.0,
                 height_m=10.0,
                 resolution=0.05,
                 frame_id='map'):
        super().__init__('map_publisher')
        self.declare_parameter('map_size',[10.0,10.0])
        self.map_size = self.get_parameter('map_size').value

        self.declare_parameter('num_blimps',2)
        self.num_blimps = self.get_parameter('num_blimps').value

        self.declare_parameter('colors',[0.0,0.0,0.0]*self.num_blimps)
        self.colors = self.get_parameter('colors').value

        self.pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self.timer = self.create_timer(0.5, self.publish_map)
        self.resolution = resolution
        self.width = int(round(self.map_size[0] / resolution))
        self.height = int(round(self.map_size[1] / resolution))
        self.frame_id = frame_id
        self.get_logger().info(f'Creating map {width_m}x{height_m} m, {self.width}x{self.height} cells')

        self.declare_parameter('blimp_positions',list(np.array([[float(i),0.0,0.0,0.0,0.0,0.0] for i in range(self.num_blimps)]).flatten()))
        initial_positions = self.get_parameter('blimp_positions').value

        self.marker_pub = self.create_publisher(MarkerArray, '/robot_markers', 1)

        self.pos_sub = self.create_subscription(Float32MultiArray,'/sim/robot_pos',self.publish_markers,10)


        # build the map once
        grid = np.zeros((self.height, self.width), dtype=np.int8)  # free = 0
        grid[0, :] = 100
        grid[-1, :] = 100
        grid[:, 0] = 100
        grid[:, -1] = 100
        self.grid_flat = grid.flatten().tolist()
    
        time.sleep(1.0)
        self.publish_markers(Float32MultiArray(data=initial_positions))

    def publish_markers(self,msg):
        blimp_positions = msg.data
        ma = MarkerArray()
        i=0
        c=0
        while i < len(blimp_positions):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = int(i//6)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(blimp_positions[i])
            m.pose.position.y = float(blimp_positions[i+1])
            m.pose.position.z = float(blimp_positions[i+2])  # z is up
            roll,pitch,yaw = blimp_positions[i+3:i+6]
            qw,qx,qy,qz = euler.euler2quat(roll,pitch,yaw)

            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw

            m.scale.x = 1.0
            m.scale.y = 0.8
            m.scale.z = 0.6
            # color: blue
            m.color.a = 1.0
            m.color.r = self.colors[c]
            m.color.g = self.colors[c+1]
            m.color.b = self.colors[c+2]
            m.lifetime.sec = 0
            ma.markers.append(m)
            i+=6
            c+=3

        self.marker_pub.publish(ma)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info = MapMetaData()
        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.width)
        msg.info.height = int(self.height)
        msg.info.origin = Pose()
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.data = list(self.grid_flat)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
