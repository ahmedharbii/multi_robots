
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy as np

#publishing to /cmd_vel
class ObstacleAvoidance(Node): 

    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.vel_forward)
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning
        self.obstacle = 0 #obstacle flag

    def lidar_callback(self, msg):
        lidar_range = np.array(msg.ranges[0:25] + msg.ranges[325:360]) # -10 to 10 degrees range

        #this to take the minimum non zeros numbers "as lidar will give 0.0 if it is out of range too"
        if np.sum(lidar_range) > 0.1: #
            min_dist = lidar_range[lidar_range != 0.0].min()
        else:
        # in some cases all of the lidar values are zero, means the lidar has no obstacles in 3.5 meter range
            min_dist = 3.5
        # print(min_dist)
        # print(np.sum(lidar_range))
        if min_dist > 0.4: #~ 60 cm
            self.obstacle = 0 #There is no obstacle, you might proceed
            
        else:
            self.obstacle = 1 #There is an obstacle

        self.get_logger().info('Lidar ranges : "%s"' % lidar_range)
        

    def vel_forward(self):
        msg = Twist()
        
        if self.obstacle == 0: #There is no obstacle

            msg.linear.x = 0.15
            msg.angular.z = 0.0
        else: # There is no obstacle

            msg.linear.x = 0.0
            msg.angular.z = 0.2
            self.obstacle = 1

        self.publisher_.publish(msg)
        self.get_logger().info('x: "%0.2f"' % msg.linear.x)
        self.get_logger().info('theta: "%0.2f"' % msg.angular.z)



def main(args=None):
    rclpy.init(args=args) #intialization

    obstacle_avoidance = ObstacleAvoidance() #instanitating the sub class

    rclpy.spin(obstacle_avoidance)
    #if KeyboardInterrupt:
        

    # try: to stop the robot if there is a keyboard interrupt
    #     rclpy.spin(obstacle_avoidance)
    # except KeyboardInterrupt:
    #     obstacle_avoidance.vel_forward(interrupt=True)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
