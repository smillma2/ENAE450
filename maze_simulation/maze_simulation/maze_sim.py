import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan as Scan
from geometry_msgs.msg import Twist
#from std_msgs.msg import String, Float64MultiArray
import math

class SolveMaze(Node):
    def __init__(self):
        super().__init__('turtle_follower')

        # distance to and angle from wall
        self.euclidean_distance = 999
        self.euclidean_angle = 0
        
        self.octant_distances = []
        self.local_maxima = []
        self.furthest_maxima = 0
        
        # distance threshold
        self.threshold = 0.5
        
        # initial state
        self.state = "sit_and_stare"

        # initialization of publisher
        self.cmdvel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # initialization of main timer
        self.timer = self.create_timer(1, self.timer_callback)
        
        # initialization of scan subscription
        self.scan_subscriber = self.create_subscription(Scan, '/scan', self.get_scan_data, 10)

    # Octants:
    #
    #        <0
    #    1        <7
    #   V           ^
    #  2             6
    #   V           ^
    #    3>        5
    #         4>


    # get relevant data from lidar scan
    def get_scan_data(self, data):
        slice_width = 10
        rough_maxima = []
        
        self.octant_distances = [0, 0, 0, 0, 0, 0, 0, 0]
        self.local_maxima = []
        self.euclidean_distance = min(data.ranges)
        self.euclidean_angle = data.ranges.index(self.euclidean_distance)
        
        self.temp = len(data.ranges)
        
        #self.get_logger().info('data_length: %s' % str(self.temp))
        
        
        for i in range(int(len(data.ranges)/slice_width)):
            pieslice = [x if x != float('inf') else 12.0 for x in data.ranges[(i*slice_width):(i+1)*slice_width]]
            rough_maxima.append(sum(pieslice) / len(pieslice))
        
        for i in range(int(len(data.ranges)/slice_width)):
            if rough_maxima[i] > rough_maxima[i-1] and rough_maxima[i] > rough_maxima[(i+1) % len(rough_maxima)]:
                self.local_maxima.append(i)
        
        for i in range(len(self.local_maxima)):
            index = self.local_maxima[i]
            self.local_maxima[i] = data.ranges.index(max(data.ranges[(index*slice_width):(index+1)*slice_width]))
        
        for i in range(len(data.ranges)):
            octant = int(i/45)
            val = data.ranges[i] if data.ranges[i] != float('inf') else 12.0
            self.octant_distances[octant] += val / 45
        
        self.furthest_maxima = data.ranges.index(min(data.ranges))
        
        for i in self.local_maxima:
            if data.ranges[i] > data.ranges[self.furthest_maxima]:
                self.furthest_maxima = i
        
        
    # main control loop
    def timer_callback(self):
        message = Twist()
        
        if self.state == 'sit_and_stare':
            if self.octant_distances:
                furthest_octant = self.octant_distances.index(max(self.octant_distances))
            else:
                furthest_octant = -1
            # if furthest_octant == 0:
            #     message.angular.z = 0.0
            #     message.linear.x = 0.2
            # else:
            #     message.angular.z = (4 - furthest_octant) * 0.25
            #     message.linear.x = 0.0
            #self.get_logger().info('Furthest octant: %s' % str(furthest_octant))
            self.get_logger().info('Maxima: %s' % str(self.local_maxima))
            self.get_logger().info('Furthest Maxima: %s' % str(self.furthest_maxima))
            
        # main state where robot follows wall and turns at corners
        if self.state == 'running_maze':
            # value by which to adjust angular velocity to maintain threshold distance from wall
            adjustment = 1.2 * (self.threshold - self.euclidean_distance)
            
            # set angular velocity based on current angle and distance relative to wall/corner
            message.angular.z = adjustment -(270 - self.euclidean_angle) * (2 * math.pi / 360)
            
            # set linear velocity
            message.linear.x = 0.5
        
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