import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class wallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.scan_sub1 = self.create_subscription(LaserScan, '/tb3_0/scan', self.callback_laser, 10)
        #self.scan_sub2 = self.create_subscription(LaserScan, '/tb3_1/scan', self.callback_laser, 10)

        self.cmd_vel_pub1 = self.create_publisher(Twist, '/tb3_0/cmd_vel', 10)
        #self.cmd_vel_pub2 = self.create_publisher(Twist, '/tb3_1/cmd_vel', 10)
       
        self.lidar_front = 100
        self.lidar_left = 100
        self.lidar_right = 100
        self.state = "start"
        self.distance_from_wall = 1.0
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback) 
    


    def callback_laser(self, msg):
        self.lidar_front = msg.ranges[0]
        self.lidar_left = msg.ranges[45]
        self.lidar_left_back = msg.ranges[100]
        #self.lidar_right = msg.ranges[315]
        #self.lidar_right_back = msg.ranges[225]

    def timer_callback(self):
        vel_msg = Twist()

        if self.state == "start":
            vel_msg.linear.x = 0.4
            vel_msg.angular.z = 0.0
            
            if self.lidar_front <= self.distance_from_wall or self.lidar_left <= self.distance_from_wall or self.lidar_right <= self.distance_from_wall:
                self.state = "wall_found"
       
        elif self.state == "wall_found":
            vel_msg.linear.x = 0.2
            vel_msg.angular.z = 0.0

            #wall to follow on left side
            if (self.distance_from_wall - 0.1) <= self.lidar_left <= (self.distance_from_wall + 0.1):
                vel_msg.linear.x = 0.3
                vel_msg.angular.z = 0.0

            #Wall in front
            elif self.lidar_front <= self.distance_from_wall:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = -0.5

            #Wall too close on left side
            elif self.lidar_left < (self.distance_from_wall - 0.1):
                vel_msg.linear.x = 0.3
                vel_msg.angular.z = -0.1

            #Lost wall
            elif self.lidar_left > self.distance_from_wall:
                vel_msg.linear.x = 0.1
                vel_msg.angular.z = 0.5
            

    
    
        self.cmd_vel_pub1.publish(vel_msg)
        #self.cmd_vel_pub2.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    wall_follower2 = wallFollower()

    rclpy.spin(wall_follower2)

    wall_follower2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()