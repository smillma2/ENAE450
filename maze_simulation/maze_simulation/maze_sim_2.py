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
        self.angular_speed = 0.5  # Angular speed for turning
        self.linear_speed = 0.15  # Linear speed for forward movement

        # Initialize sensor data
        self.front_distance = float('inf')
        self.right_distance = float('inf')
        self.left_distance = float('inf')

    # Get relevant data from lidar scan
    def get_scan_data(self, data):
        # Extract distances at specific angles:
        # Front: 0 degrees
        # Right: -90 degrees
        # Left: +90 degrees
        num_angles = len(data.ranges)
        self.front_distance = min(data.ranges[0:10] + data.ranges[-10:])  # 0 degrees
        self.right_distance = min(data.ranges[int(0.75 * num_angles):int(0.85 * num_angles)])  # -90 degrees
        self.left_distance = min(data.ranges[int(0.15 * num_angles):int(0.25 * num_angles)])  # +90 degrees

    # Main control loop
    def timer_callback(self):
        message = Twist()

        if self.front_distance < self.front_threshold:
            # Wall in front, turn left until the wall is to the right
            message.linear.x = 0.0
            message.angular.z = self.angular_speed
        elif self.right_distance > self.right_wall_distance:
            # No wall on the right, turn right while moving forward
            message.linear.x = self.linear_speed
            message.angular.z = -self.angular_speed
        else:
            # Wall to the right, move forward while staying parallel to it
            message.linear.x = self.linear_speed

            # Adjust angular velocity to maintain the desired distance to the right wall
            if self.right_distance < self.right_wall_distance:
                # Too close to the right wall, turn left slightly
                message.angular.z = self.angular_speed / 2
            elif self.right_distance > self.right_wall_distance:
                # Too far from the right wall, turn right slightly
                message.angular.z = -self.angular_speed / 2
            else:
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
