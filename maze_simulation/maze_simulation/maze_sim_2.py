import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan as Scan
from geometry_msgs.msg import Twist
import math


class SolveMaze(Node):
    def __init__(self):
        super().__init__('turtle_follower')

        # Initialization of publisher
        self.cmdvel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialization of main timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialization of scan subscription
        self.scan_subscriber = self.create_subscription(Scan, '/scan', self.get_scan_data, 10)

        # Parameters
        self.safe_distance = 0.5  # Safe distance from walls in meters
        self.right_wall_distance = 0.3  # Desired distance from right wall
        self.front_threshold = 0.5  # Threshold distance to consider a wall ahead
        self.back_threshold = 0.5  # Threshold distance for back wall
        self.left_threshold = 0.5  # Threshold distance for left wall
        self.angular_speed = 0.5  # Angular speed for turning
        self.linear_speed = 0.15  # Linear speed for forward movement

        # Initialize sensor data
        self.front_distance = float('inf')
        self.right_distance = float('inf')
        self.left_distance = float('inf')
        self.back_distance = float('inf')

    # Get relevant data from lidar scan
    def get_scan_data(self, data):
        # Extract distances at specific angles:
        # Front: 0 degrees
        # Right: -90 degrees
        # Left: +90 degrees
        # Back: 180 degrees
        self.front_distance = min(data.ranges[0:10] + data.ranges[-10:])  # 0 degrees
        self.right_distance = min(data.ranges[260:280])  # -90 degrees
        self.left_distance = min(data.ranges[80:100])  # +90 degrees
        self.back_distance = min(data.ranges[170:190])  # 180 degrees

    def is_close(self, distance, threshold):
        return distance < threshold

    def timer_callback(self):
        message = Twist()

        # Determine proximity of walls
        close_left = self.is_close(self.left_distance, self.left_threshold)
        close_right = self.is_close(self.right_distance, self.right_wall_distance)
        close_front = self.is_close(self.front_distance, self.front_threshold)
        close_back = self.is_close(self.back_distance, self.back_threshold)
        
        print("Front: ", self.front_distance)
        print("Right: ", self.right_distance)
        print("Left: ", self.left_distance)
        print("Back: ", self.back_distance)
        print("Close Left: ", close_left)
        print("Close Right: ", close_right)
        print("Close Front: ", close_front)
        print("Close Back: ", close_back)
        

        # Decision making based on combinations of wall distances
        if close_front:
            if close_left and close_right:
                if close_back:
                    # All sides are close
                    message.linear.x = 0.0
                    message.angular.z = self.angular_speed
                else:
                    # Left, Right, Front are close, Back is far
                    message.linear.x = -self.linear_speed
                    message.angular.z = self.angular_speed
            elif close_left and not close_right:
                # Front and Left are close, Right is far
                message.linear.x = 0.0
                message.angular.z = -self.angular_speed
            elif not close_left and close_right:
                # Front and Right are close, Left is far
                message.linear.x = 0.0
                message.angular.z = self.angular_speed
            else:
                # Only Front is close
                message.linear.x = 0.0
                message.angular.z = self.angular_speed
        else:
            if close_left and close_right:
                if close_back:
                    # Left, Right, Back are close, Front is far
                    message.linear.x = 0.0
                    message.angular.z = self.angular_speed if self.left_distance < self.right_distance else -self.angular_speed
                else:
                    # Left and Right are close, Front and Back are far
                    message.linear.x = self.linear_speed * 1/2
                    message.angular.z = self.angular_speed if self.left_distance < self.right_distance else -self.angular_speed
            elif close_left and not close_right:
                # Left is close, Right is far
                message.linear.x = self.linear_speed
                message.angular.z = -self.angular_speed
            elif not close_left and close_right:
                # Right is close, Left is far
                message.linear.x = self.linear_speed
                message.angular.z = self.angular_speed
            else:
                # All sides are far
                message.linear.x = self.linear_speed
                message.angular.z = 0.0

        self.cmdvel_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    turtle_follower = SolveMaze()

    rclpy.spin(turtle_follower)

    turtle_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
