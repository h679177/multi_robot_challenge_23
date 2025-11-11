import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class Robot:
    def __init__(self, name, node):

        self.name = name
        self.node = node
        self.direction = 0
        self.lidar_front = 100
        self.lidar_side = 100
        self.lidar_back = 100
        self.lidar_prev_back = 100
        self.position = None
        self.starting_position = None
        self.state = "start"

        self.scan_sub = node.create_subscription(LaserScan, f'/{name}/scan', self.clbk_laser, 10)
        self.cmd_vel_pub = self.node.create_publisher(Twist, f'/{name}/cmd_vel', 10)
        self.odom_sub = self.node.create_subscription(Odometry, f'/{name}/odom', self.clbk_odom, 10)

    def clbk_laser(self, msg):
        self.lidar_front = msg.ranges[0]
        self.lidar_back = msg.ranges[180]
        if self.direction == 1:
            self.lidar_side = msg.ranges[45]
        else:
            self.lidar_side = msg.ranges[315]
    
    def clbk_odom(self, msg):
        if self.position is None:
            self.starting_position = msg.pose.pose.position
        self.position = msg.pose.pose.position
        #if self.starting_position_1 is not None:
        #    self.map_divide = abs(self.starting_position_0.y - self.starting_position_1.y) / 2

        


class wallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
              
        self.distance_from_wall = 0.6
        self.map_division = None

        self.pippi = Robot('tb3_0', self)
        self.fiona = Robot('tb3_1', self)        
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback) 
    
    def timer_callback(self):
        if self.pippi.position is not None and self.fiona.position is not None:
            vel_msg_pippi = self.wall_follower(self.pippi)
            vel_msg_fiona = self.wall_follower(self.fiona)
            self.pippi.cmd_vel_pub.publish(vel_msg_pippi)
            self.fiona.cmd_vel_pub.publish(vel_msg_fiona)
     
        
    def wall_follower(self, robot):
        vel_msg = Twist()

        if robot.state == "start":
            #if direction == 1:
                #self.get_logger().info("start")
            vel_msg.linear.x = 0.5
            vel_msg.angular.z = 0.0
            
            if self.pippi.starting_position is not None and self.fiona.starting_position is not None and self.map_division is None:
                self.map_division = (self.pippi.starting_position.y + self.fiona.starting_position.y) / 2
                self.get_logger().info("Division line: " + str(self.map_division))
            if robot.direction == 0:
                if robot.position.y < self.map_division:
                    robot.direction = 1
                    self.get_logger().info(str(robot.name) + str(robot.direction) + " Position: " + str(robot.position))
                else:
                    robot.direction = -1
                    self.get_logger().info(str(robot.name) + str(robot.direction) + " Position: " + str(robot.position))

            if robot.lidar_front <= self.distance_from_wall or robot.lidar_side <= self.distance_from_wall:
                robot.state = "wall_found"
       
        elif robot.state == "find_new_wall":
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = -0.5 * robot.direction
            if robot.lidar_prev_back < robot.lidar_back:
                if robot.lidar_prev_back == 100:
                    robot.lidar_prev_back = robot.lidar_back
                robot.state = "start"
            else:
                self.get_logger().info("Turning: " + str(robot.lidar_prev_back) + " This back: " + str(robot.lidar_back))
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = -0.5 * robot.direction
                robot.lidar_prev_back = robot.lidar_back



        
        elif robot.state == "wall_found":
            vel_msg.linear.x = 0.2
            vel_msg.angular.z = 0.0

            if self.map_division - 0.2 < robot.position.y < self.map_division + 0.2:
                robot.state = "find_new_wall"
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = -0.5 * robot.direction
                self.get_logger().info(robot.name + "is finding her new wall.")
            
            #if abs(robot.position.y - robot.starting_position.y) < 0.2 and robot.position.x < robot.starting_position.x:
            #   vel_msg.linear.x = 0.0
            #   vel_msg.angular.y = 0.0

            #wall to follow on left side
            elif (self.distance_from_wall - 0.1) <= robot.lidar_side <= (self.distance_from_wall + 0.1):
                vel_msg.linear.x = 0.5
                vel_msg.angular.z = 0.0

            #Wall in front
            elif robot.lidar_front <= self.distance_from_wall:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = -0.5 * robot.direction
                if robot.lidar_front <= self.distance_from_wall - 0.3:
                    vel_msg.linear.x = -0.3

            #Wall too close on left side
            elif robot.lidar_side < (self.distance_from_wall - 0.1):
                #if robot.direction == 1:
                    #self.get_logger().info("wall too close")
                vel_msg.linear.x = 0.4
                vel_msg.angular.z = -0.2 * robot.direction

            #Lost wall
            elif robot.lidar_side > self.distance_from_wall:
                vel_msg.linear.x = 0.1
                vel_msg.angular.z = 0.5 * robot.direction
            

        return vel_msg


def main(args=None):
    rclpy.init(args=args)

    wall_follower2 = wallFollower()

    rclpy.spin(wall_follower2)

    wall_follower2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()