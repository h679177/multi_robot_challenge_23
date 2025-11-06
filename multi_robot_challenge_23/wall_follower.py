import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class wallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.scan_sub1 = self.create_subscription(LaserScan, '/tb3_0/scan', self.callback_laser_tb3_0, 10)
        self.scan_sub2 = self.create_subscription(LaserScan, '/tb3_1/scan', self.callback_laser_tb3_1, 10)

        self.cmd_vel_pub1 = self.create_publisher(Twist, '/tb3_0/cmd_vel', 10)
        self.cmd_vel_pub2 = self.create_publisher(Twist, '/tb3_1/cmd_vel', 10)

        self.odom_sub1 = self.create_subscription(Odometry, '/tb3_0/odom', self.clbk_odom_tb3_0, 10) 
        self.odom_sub2 = self.create_subscription(Odometry, '/tb3_1/odom', self.clbk_odom_tb3_1, 10) 
       
       
        self.lidar_front_0 = 100
        self.lidar_left_0 = 100
        self.lidar_front_1 = 100
        self.lidar_right_1 = 100
        self.position_0 = None
        self.position_1 = None
        self.state = "start"
        self.distance_from_wall = 0.6
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback) 
    


    def callback_laser_tb3_0(self, msg):
        self.lidar_front_0 = msg.ranges[0]
        self.lidar_left_0 = msg.ranges[45]
        
    def callback_laser_tb3_1(self, msg):
        self.lidar_front_1 = msg.ranges[0]
        self.lidar_right_1 = msg.ranges[315]

    def clbk_odom_tb3_0(self, msg):
        if self.position_0 is None:
            self.starting_postition_0 = msg.pose.pose.postion
        self.position_0 = msg.pose.pose.postion

    def clbk_odom_tb3_1(self, msg):
        if self.position_1 is None:
            self.starting_postition_1 = msg.pose.pose.postion
        self.position_1 = msg.pose.pose.postion

    def timer_callback(self):
        if self.postion_1 is not None and self.position_0 is not None:
            vel_msg_pippi = self.wall_follower(self.lidar_front_0, self.lidar_left_0, 1, self.starting_postition_0)
            vel_msg_fiona = self.wall_follower(self.lidar_front_1, self.lidar_right_1, -1, self.starting_postition_1)


        self.cmd_vel_pub1.publish(vel_msg_pippi)
        self.cmd_vel_pub2.publish(vel_msg_fiona)        
        
    def wall_follower(self, front, atAngle, direction, start):
        vel_msg = Twist()

        if self.state == "start":
            vel_msg.linear.x = 0.5
            vel_msg.angular.z = 0.0
            
            if front <= self.distance_from_wall or atAngle <= self.distance_from_wall:
                self.state = "wall_found"
       
        elif self.state == "wall_found":
            vel_msg.linear.x = 0.2
            vel_msg.angular.z = 0.0

            # if x = startpos and y < startpos:
            #   stopp.

            #wall to follow on left side
            if (self.distance_from_wall - 0.1) <= atAngle <= (self.distance_from_wall + 0.1):
                vel_msg.linear.x = 0.5
                vel_msg.angular.z = 0.0

            #Wall in front
            elif front <= self.distance_from_wall:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = -0.5 * direction
                if front <= self.distance_from_wall - 0.3:
                    vel_msg.linear.x = -0.3

            #Wall too close on left side
            elif atAngle < (self.distance_from_wall - 0.1):
                vel_msg.linear.x = 0.5
                vel_msg.angular.z = -0.1 * direction

            #Lost wall
            elif atAngle > self.distance_from_wall:
                vel_msg.linear.x = 0.1
                vel_msg.angular.z = 0.5 * direction
            

        return vel_msg


def main(args=None):
    rclpy.init(args=args)

    wall_follower2 = wallFollower()

    rclpy.spin(wall_follower2)

    wall_follower2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()