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
        
        # states = ["finding wall", "turning_right_start", "turning_right_end", "turning_left_start", "turning_left_end"]
        self.state = "finding_wall"
        self.turn_counter = 10
        
        # storage for the previous angle sums while turning
        self.previous_angle_sums = []
        
        self.data = None
        

    def get_scan_data(self, data):
        # Extract distances at specific angles
        self.data = data.ranges
        
    def is_right_wall_contiguous(self, right_data):
        right_data = [12 if x == float('inf') else x for x in right_data]
        # Initialize a counter for persistent jumps
        persistent_jump_count = 0
        
        # Iterate over the data starting from the second element
        for i in range(1, len(right_data)):
            # Check if there is a significant jump from the previous data point
            if right_data[i] - right_data[i-1 - persistent_jump_count] > 1:
                # Increment the jump counter
                persistent_jump_count += 1
                # If the jump has persisted for at least 4 indices, return the start index of the first jump
                if persistent_jump_count >= 3:
                    return i - persistent_jump_count
            else:
                # Reset the counter if the jump does not persist
                persistent_jump_count = 0
                
        # Return -1 if no persistent significant jump is found
        return -1
    
    def get_right_wall_angle_with_interp(self, right_data):
        # interpolate out the infs
        for i in range(1, len(right_data)):
            if right_data[i] == float('inf'):
                right_data[i] = right_data[i-1]
        
        for _ in range(10):
            i = random.randint(0, len(right_data)//2 - 1)
            angle_sum += right_data[i] - right_data[len(right_data) - i - 1]
        
        return angle_sum
    
    def get_right_wall_angle(self, right_data):
        right_data = [12 if x == float('inf') else x for x in right_data]
        # randomly choose a number i from 0 to len(right_data)/2
        # sample the data right_data[i] and right_data[len(right_data) - i]
        # sum up the differences between the two samples and return that sum
        for _ in range(10):
            i = random.randint(0, len(right_data)//2 - 1)
            angle_sum += right_data[i] - right_data[len(right_data) - i - 1]
        
        return angle_sum
        
    def timer_callback(self):
        message = Twist()
        
        linear_x = 0.0
        angular_z = 0.0
        
        if self.state == "finding_wall":
            # look for wall in front of the robot
            if self.data:
                forward_data = self.data[345:375]
                right_data = self.data[90:270]
                minimum_distance = min(forward_data)
                if minimum_distance < 0.3:
                    self.state = "turning_left_start"
                    self.turn_counter = 10
                else:
                    contiguous_result = self.is_right_wall_contiguous(right_data)
                    # # if the right wall is contiguous, keep moving forward
                    # if contiguous_result != -1 and contiguous_result < 10:
                    #     self.state = "turning_right_start"
                    #     self.turn_counter = 10
                    # else:
                    linear_x = 0.5
                    angular_z = self.get_right_wall_angle(right_data)//30
                        
        if self.state == "turning_right_start":
            if self.turn_counter > 0:
                linear_x = 0.0
                angular_z = -0.5
                self.turn_counter -= 1
            else:
                self.state = "turning_right_end"
        
        if self.state == "turning_left_start":
            if self.turn_counter > 0:
                linear_x = 0.0
                angular_z = 0.5
                self.turn_counter -= 1
                self.previous_angle_sums.append(abs(self.get_right_wall_angle(self.data[90:270])))
            else:
                self.state = "turning_left_end"
                
        if self.state == "turning_left_end":
            if len(self.previous_angle_sums) < 5:
                linear_x = 0.0
                angular_z = 0.5
                self.previous_angle_sums.append(abs(self.get_right_wall_angle(self.data[90:270])))
            else:
                # check if the last 5 angle sums are below a certain threshold
                self.get_logger().info("Sum: %s" % sum(self.previous_angle_sums[-5:]))
                
                if sum(self.previous_angle_sums[-5:]) < 10:
                    self.state = "finding_wall"
                    self.previous_angle_sums = []
                    
                else:
                    linear_x = 0.0
                    angular_z = 0.5
                    self.previous_angle_sums.append(abs(self.get_right_wall_angle(self.data[90:270])))
                
                    
        self.get_logger().info("State: %s" % self.state)
            

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
