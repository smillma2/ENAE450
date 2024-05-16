import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan as Scan
from geometry_msgs.msg import Twist
import math

class SolveMaze(Node):
    def __init__(self):
        super().__init__('turtle_follower')

        # Publisher initialization
        self.cmdvel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer initialization
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Subscription initialization
        self.scan_subscriber = self.create_subscription(Scan, '/scan', self.get_scan_data, 10)
                
        self.data = None
        

    def get_scan_data(self, data):
        # Extract distances at specific angles
        self.data = data.ranges
        self.data = [x if x != float('inf') else 12 for x in self.data]
        
       
    def timer_callback(self):
        message = Twist()
        
        linear_x = 0.3
        angular_z = 0.0

        # Find closest point (index) from indices 0 to 360
        # if the closest point's index is below 180, turn right
        # otherwise turn left
        # Turn proportionally to how far the index is from 180
        if self.data:
            min_pos = self.data[:360].index(min(self.data[:360]))
            angular_z = (180 - min_pos)/360

            self.get_logger().info("min index: %s" % str(min_pos))
            self.get_logger().info("min val %s" % str(min(self.data[:360])))
            self.get_logger().info("angular val %s" % str(angular_z))
        # Add a threshold check for basically all around the robot
        # if you are too close just rotate, no linear
        # ggs
        if self.data and min(self.data[:360]) < 0.45:
            linear_x = 0.0

        message.linear.x = linear_x
        message.angular.z = angular_z

        self.cmdvel_publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    turtle_follower = SolveMaze()
    rclpy.spin(turtle_follower)
    turtle_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()