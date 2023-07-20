import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import tf2_ros
import geometry_msgs.msg
import tf_transformations
import math
import numpy as np
from bresenham import bresenham
import time
import csv


class Controller(Node):
    def __init__(self):
        super().__init__('control')
        #self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        #self.ranges_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_subsribe_callback, 10)
        #self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.ranges = [0.0,0.0,0.0,0.0,0.0]
        self.position =  [0.0, 0.0, 0.0]
        self.angles =  [0.0, 0.0, 0.0]
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.point_subscriber = self.create_subscription(PointStamped, 'target_points', self.point_callback, 100)
        # Create a list to store the recorded coordinates
        self.coordinates = []

        # Publisher for the drone's current position
        self.pose_publisher = self.create_publisher(PoseStamped, 'drone_pose', 10)

        self.target_reached = False
        

    def point_callback(self, point):
        if not self.target_reached:
            # Get the robot's current position and orientation in the map frame
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.get_logger().warn("Transform unavailable. Unable to reach the target point.")
                return

            # Calculate the difference between the target point and the robot's current position
            dx = point.point.x - transform.transform.translation.x
            dy = point.point.y - transform.transform.translation.y

            # Calculate the distance to the target point
            distance_to_target = math.sqrt(dx**2 + dy**2)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            self.coordinates.append(pose)
            # If the robot is close enough to the target point, stop moving
            if distance_to_target < 0.1:  # Adjust the distance threshold as needed
                self.target_reached = True
                vel_cmd = Twist()
                vel_cmd.linear.x = 0.0
                vel_cmd.angular.z = 0.0
                self.cmd_vel_publisher.publish(vel_cmd)
                self.save_coordinates_to_csv()
            else:
                # Calculate the angle between the robot's orientation and the target point
                target_angle = math.atan2(dy, dx)

                # Calculate the required linear and angular velocities to reach the target point
                vel_cmd = Twist()
                vel_cmd.linear.x = 0.1  # Adjust the linear velocity as needed
                vel_cmd.angular.z = 1.5 * target_angle  # Adjust the angular velocity as needed

                # Publish the velocity command
                self.cmd_vel_publisher.publish(vel_cmd)
                self.save_coordinates_to_csv()
    
    def save_coordinates_to_csv(self):
        # Save the recorded coordinates to a CSV file (e.g., coordinates.csv)
        with open('coordinates.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['X', 'Y', 'Z'])  # Write header row
            for pose in self.coordinates:
                writer.writerow([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])

    """
    def scan_subsribe_callback(self, msg):
        self.ranges[2] = msg.ranges
        
        if min(msg.ranges) < 0.4:
            vel_cmd = Twist()
            vel_cmd.linear.x = -0.05
            vel_cmd.angular.z = 1.4
            
            self.ranges[2] = msg.ranges
            
        else:
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.08


        self.publisher.publish(vel_cmd)
        """
    
    

def main(args=None):

    rclpy.init(args=args)
    control = Controller()
    rclpy.spin(control)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    
    main()








