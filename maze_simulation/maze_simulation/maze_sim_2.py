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
        self.angle_correction_factor = 0.3  # Factor for correcting heading

        # Initialize sensor data
        self.front_distance = float('inf')
        self.right_distance = float('inf')
        self.right_points = []

    # Get relevant data from lidar scan
    def get_scan_data(self, data):
        # Extract distances at specific angles:
        # Front: 0 degrees
        # Right: -90 degrees
        num_angles = len(data.ranges)
        self.front_distance = min(data.ranges[0:10] + data.ranges[-10:])  # 0 degrees
        self.right_points = data.ranges[int(0.75 * num_angles):int(0.85 * num_angles)]  # -90 degrees
        self.right_distance = min(self.right_points)

    # Calculate angle of the wall to the right based on lidar points
    def calculate_right_wall_angle(self):
        # Use two points from the right side to calculate the angle
        if len(self.right_points) < 2:
            return 0.0

        # Point 1 and Point 2 correspond to the nearest and farthest points to the right
        point1 = self.right_points[0]
        point2 = self.right_points[-1]

        if point1 == float('inf') or point2 == float('inf'):
            return 0.0

        # Calculate the difference in distance and angle between the two points
        angle_increment = (math.pi / 180) * 10  # 10-degree increment for simplicity
        angle_diff = angle_increment * (len(self.right_points) - 1)
        distance_diff = point2 - point1

        # Wall angle relative to the robot (positive means wall angled towards the front)
        wall_angle = math.atan2(distance_diff, angle_diff)

        return wall_angle

    # Main control loop
    def timer_callback(self):
        message = Twist()

        right_wall_angle = self.calculate_right_wall_angle()

        if self.front_distance < self.front_threshold:
            # Wall in front, turn left until the wall is to the right
            message.linear.x = 0.0
            message.angular.z = self.angular_speed
        elif self.right_distance > self.right_wall_distance:
            # No wall on the right, turn right while moving forward
            message.linear.x = self.linear_speed / math.cos(right_wall_angle)
            message.angular.z = -self.angular_speed
        else:
            # Wall to the right, move forward while staying parallel to it
            message.linear.x = self.linear_speed / math.cos(right_wall_angle)

            # Adjust angular velocity to maintain the desired distance to the right wall
            if self.right_distance < self.right_wall_distance:
                # Too close to the right wall, turn left slightly
                message.angular.z = self.angle_correction_factor
            elif self.right_distance > self.right_wall_distance:
                # Too far from the right wall, turn right slightly
                message.angular.z = -self.angle_correction_factor
            else:
                # Correct alignment based on wall angle
                message.angular.z = -right_wall_angle * self.angle_correction_factor

        self.cmdvel_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    turtle_follower = SolveMaze()

    rclpy.spin(turtle_follower)

    turtle_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
