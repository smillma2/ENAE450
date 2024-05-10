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
        right_data[0] = 6 if right_data[0] == float('inf') else right_data[0]
        for i in range(1, len(right_data)):
            if right_data[i] == float('inf'):
                right_data[i] = right_data[i-1]
                
        clipped_right_data = [[]]
        for i in range(1, len(right_data)):
            if right_data[i] - right_data[i-1] > 0.5:
                clipped_right_data.append([])
            clipped_right_data[-1].append(right_data[i])
            
        # get longest sublist
        clipped_right_data = max(clipped_right_data, key=len)
                
        angle_sum = 0
        
        for _ in range(30):
            i = random.randint(0, len(clipped_right_data)//2 - 1)
            angle_sum += clipped_right_data[i] - clipped_right_data[len(clipped_right_data) - i - 1]
        
        return angle_sum / 10
    
    def get_right_wall_angle(self, right_data):
        right_data = [12 if x == float('inf') else x for x in right_data]
        # randomly choose a number i from 0 to len(right_data)/2
        # sample the data right_data[i] and right_data[len(right_data) - i]
        # sum up the differences between the two samples and return that sum
        angle_sum = 0
        for _ in range(30):
            i = random.randint(0, len(right_data)//2 - 1)
            angle_sum += right_data[i] - right_data[len(right_data) - i - 1]
        
        return angle_sum / 30
    
    def linearize_right_wall(self, right_data):
        right_data[0] = 6 if right_data[0] == float('inf') else right_data[0]
        for i in range(1, len(right_data)):
            if right_data[i] == float('inf'):
                right_data[i] = right_data[i-1]
        center_index = len(right_data) // 2
        
        linearized_data = []
        
        for i in range(len(right_data)):
            angle = (i - center_index) / 2
            linearized_data.append(right_data[i] * math.cos(math.radians(angle)))
            
        return linearized_data
    
    def get_right_wall_variance(self, right_data):
        
        linearized_data = self.linearize_right_wall(right_data)

        self.get_logger().info("linearized %s" % str(linearized_data))
        
        return sum([(x - sum(linearized_data)/len(linearized_data))**2 for x in linearized_data]) / len(linearized_data)
        
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
                if minimum_distance < 0.6:
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
                    angular_z = self.get_right_wall_angle_with_interp(right_data)
                        
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
            linear_x = 0.0
            angular_z = 0.5
            self.previous_angle_sums.append(self.get_right_wall_angle(self.data[90:270]))
            variance = self.get_right_wall_variance(self.data[90:270])
            self.get_logger().info("variance %s" % str(variance))
            self.get_logger().info("angle %s" % str(self.previous_angle_sums[-1]))
            # check if the signs of the last 3 elements in previous_angle_sums are different from the sign
            # of the sum of the last 10 elements in previous_angle_sums
            are_last_three_signs_same = all([x > 0 for x in self.previous_angle_sums[-3:]]) or all([x < 0 for x in self.previous_angle_sums[-3:]])
            last_10_sum = sum(self.previous_angle_sums[-10:])
            if are_last_three_signs_same and last_10_sum * self.previous_angle_sums[-1] < 0 and variance < 0.1:
                self.state = "finding_wall"
                self.previous_angle_sums = []

            

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