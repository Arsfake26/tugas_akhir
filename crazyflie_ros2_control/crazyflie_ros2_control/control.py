import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

import tf_transformations
import math
import numpy as np
from bresenham import bresenham


class Controller(Node):
    def __init__(self):
        super().__init__('control')
        #self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_subcribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_subsribe_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.ranges = [0.0, 0.0, 0.0, 0.0]

    def scan_subsribe_callback(self, msg):
        self.ranges[2] = msg.ranges
        


        if min(msg.ranges) < 0.3:
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 1.5708
        else:
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.2


        self.publisher.publish(vel_cmd)
        

def main(args=None):

    rclpy.init(args=args)
    control = Controller()
    rclpy.spin(control)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    
    main()








