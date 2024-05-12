import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan as Scan
from geometry_msgs.msg import Twist
#from std_msgs.msg import String, Float64MultiArray
import math

class SolveMaze(Node):
    def __init__(self):
        super().__init__('turtle_follower')
        
        # front and side distance thresholds; if robot rear corner is hitting wall when turning, consider increasing side threshold.
        # if robot rear corner is hitting wall when "done" with turn and adjusting to be proper distance again, consider increasing fron threshold
        self.front_threshold = 0.35
        self.side_threshold = 0.35
        
        self.front = 999
        self.e_dist_l = 999
        self.e_dist_r = 999
        self.e_ang_r = 0

        # initialization of publisher
        self.cmdvel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # initialization of main timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # initialization of scan subscription
        self.scan_subscriber = self.create_subscription(Scan, '/scan', self.get_scan_data, 10)

    # get relevant data from lidar scan
    def get_scan_data(self, data):
        # ON REAL ROBOT, CHANGE TO 360 instead of 0
        self.front = data.ranges[0]
        
        # ON REAL ROBOT, CHANGE TO PROPER DATA RANGES
        data_left = [12.0 if x == float('inf') else x for x in data.ranges[45:135]]
        data_right = [12.0 if x == float('inf') else x for x in data.ranges[225:315]]
        
        self.e_dist_l = min(data_left)
        self.e_dist_r = min(data_right)
        
        self.e_ang_r = data.ranges.index(self.e_dist_r)
        
        #self.get_logger().info('Chunk 4: %s' % str(chunk_4))
    
    # main control loop
    def timer_callback(self):
        message = Twist()
        
        # ON REAL ROBOT, CHANGE 270 to RIGHTMOST ANGLE (540? idk)
        # try to align w/ rightmost angle
        aim_angle = self.e_ang_r - 270
        
        # attempt to stay a specific distance away from right wall, unless left wall is too close, in which case stay centered between left and right
        wall_dist_parameter = self.side_threshold if self.side_threshold < self.e_dist_l else self.e_dist_l
        
        # calculate how much to modify turn to align w/ walls
        wall_dist_modifier = (wall_dist_parameter - self.e_dist_r) / ((wall_dist_parameter + self.e_dist_r)/2)
        
        # this case should only trigger if it's time to turn left at an inner corner to keep following the right wall
        if self.front < self.front_threshold:
            # add hysteresis so that robot makes a smooth turn instead of turning a bit, moving forward a bit, and repeating
            self.front_threshold = 0.6
            
            # sit and turn instead of moving forward
            message.linear.x = 0.0
            message.angular.z = 0.2
        # otherwise, just follow wall (this may include making outside turns)
        else:
            # reset front threshold after hysteresis, if it happened
            self.front_threshold = 0.35
            # move forward at constant rate
            message.linear.x = 0.1
            
            # ON REAL ROBOT, CHANGE 180 to 360 (i think, maybe test to make sure) TO ADJUST FOR DOUBLED ANGLE RANGE
            # turn specific amount to stay the correct distance from the right wall (or in between right and left if they're close)
            message.angular.z = (aim_angle * math.pi/180) + wall_dist_modifier
        
        # send commands to robot
        self.cmdvel_publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    turtle_follower = SolveMaze()

    rclpy.spin(turtle_follower)

    turtle_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
