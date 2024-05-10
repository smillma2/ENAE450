import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan as Scan
from geometry_msgs.msg import Twist
#from std_msgs.msg import String, Float64MultiArray
import math

class SolveMaze(Node):
    def __init__(self):
        super().__init__('turtle_follower')
        
        # distance threshold
        self.threshold = 0.4
        
        self.front = 999
        self.e_dist_l = 999
        self.e_ang_l = 0
        self.e_dist_r = 999
        self.e_ang_r = 0
        self.data = [0]

        # initialization of publisher
        self.cmdvel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # initialization of main timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # initialization of scan subscription
        self.scan_subscriber = self.create_subscription(Scan, '/scan', self.get_scan_data, 10)

    # get relevant data from lidar scan
    def get_scan_data(self, data):
        self.data = data.ranges
        self.front = min(data.ranges[360:380] + data.ranges[340:360])
        
        data_left = [12.0 if x == float('inf') else x for x in data.ranges[450:630]] #45:135
        data_right = [12.0 if x == float('inf') else x for x in data.ranges[90:270]] #225:315
        
        self.e_dist_l = min(data_left)
        self.e_dist_r = min(data_right)

        #self.get_logger().info('e_dist_l: %s' % str(self.e_dist_l))
        #self.get_logger().info('e_dist_r: %s' % str(self.e_dist_r))
        
        self.e_ang_l = data.ranges.index(self.e_dist_l)
        self.e_ang_r = data.ranges.index(self.e_dist_r)

    # main control loop
    def timer_callback(self):
        message = Twist()
        self.get_logger().info("Min Index: %s" % str(self.data.index(min(self.data))))

        aim_angle = (self.e_ang_l + self.e_ang_r)/2
        
        wall_dist_modifier = (self.e_dist_l - self.e_dist_r) / ((self.e_dist_l + self.e_dist_r)/2)
        
        if self.front < self.threshold:
            message.linear.x = 0.0
            message.angular.z = 0.1 if self.e_dist_l > self.e_dist_r else -0.1 
            self.get_logger().info('Front: %s' % str(self.front))
        else:
            message.linear.x = 2.0
            message.angular.z = 0.3
            #message.angular.z = (aim_angle * (2*(math.pi/180)) + wall_dist_modifier)
            #self.get_logger().info('Wall Mod: %s' % str((aim_angle * math.pi/180) + wall_dist_modifier))
            
        # log current state
        #self.get_logger().info(self.state)
        
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
