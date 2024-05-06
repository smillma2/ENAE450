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

        # State initialization
        self.state = 1  # Start following the right wall

        # Parameters
        self.safe_distance = 0.5  # Safe distance from walls
        self.right_wall_distance = 0.3  # Desired distance from the right wall
        self.front_threshold = 0.5  # Threshold distance for front wall

        # Initialize sensor data
        self.front_distance = float('inf')
        self.right_distance = float('inf')
        self.left_distance = float('inf')

    def get_scan_data(self, data):
        # Extract distances at specific angles
        self.front_distance = min(data.ranges[0:10] + data.ranges[-10:])  # Front
        self.right_distance = min(data.ranges[260:280])  # Right
        self.left_distance = min(data.ranges[80:100])  # Left

    def timer_callback(self):
        message = Twist()

        if self.state == 1:  # Following the right wall
            if self.front_distance < self.front_threshold:
                self.state = 3
            elif self.right_distance > self.right_wall_distance + 0.1:
                self.state = 2
            else:
                message.linear.x = 0.2
                # Adjust angular velocity to maintain distance to the right wall
                message.angular.z = -0.5 * (self.right_distance - self.right_wall_distance)
        
        elif self.state == 2:  # Right wall disappeared
            if self.right_distance < self.right_wall_distance + 0.1:
                self.state = 1
            message.linear.x = 0.15
            message.angular.z = -0.3  # Turning right to find the wall
        
        elif self.state == 3:  # Front wall too close
            if self.front_distance > self.front_threshold:
                self.state = 1
            message.linear.x = 0
            message.angular.z = 1.0  # Turning left in place

        self.cmdvel_publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    turtle_follower = SolveMaze()
    rclpy.spin(turtle_follower)
    turtle_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
