import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg    import LaserScan
import sys, os

sys.path.append(os.path.abspath('../multi_robot_scoring/scoring_interfaces/srv'))
from scoring_interfaces.srv import SetMarkerPosition

class MarkerDetection(Node):
    def __init__(self):
        super().__init__('RobotLeaderNode')

        #-----------------------------------------------------------------------------------
        #Create a subscriber to the marker_map_pose and the marker_id topics of either
        #      tb3_0 or tb3_1. As callback functions use clbk_marker_map_pose and clbk_marker_id.

        self.marker_id_sub = self.create_subscription(Int64, 'tb3_0/marker_id', self.clbk_marker_id, 10)
        self.marker_pose_sub = self.create_subscription(Pose, 'tb3_0/marker_map_pose', self.clbk_marker_map_pose, 10)

        self.marker_id_sub = self.create_subscription(Int64, 'tb3_1/marker_id', self.clbk_marker_id, 10)
        self.marker_pose_sub = self.create_subscription(Pose, 'tb3_1/marker_map_pose', self.clbk_marker_map_pose, 10)
        
        
        self.cli = self.create_client(SetMarkerPosition, '/set_marker_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available')
        self.req = SetMarkerPosition.Request()

        self.prev_marker_id = -1
        self.marker_id = -1
        self.marker_position = Point()
        self.marker_list = []

        timer_period = 1.0  # seconds
        # Create timer function that gets executed once per second
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def clbk_marker_map_pose(self, msg):
        self.marker_position = msg.position


    def clbk_marker_id(self, msg):
        self.marker_id = msg.data
    
    
    def send_request(self, id, pos):
        self.req.marker_id = id
        self.req.marker_position = pos

        self.future = self.cli.call_async(self.req)
        #rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def timer_callback(self):
        
        if self.marker_id not in self.marker_list and self.marker_id != -1:
            self.get_logger().info(str(self.marker_id) + str(self.marker_list))
            self.get_logger().info("ID: " + str(self.marker_id))
            self.get_logger().info("Pos: " + str(self.marker_position))
            self.send_request(self.marker_id, self.marker_position)
            self.marker_list.append(self.marker_id)
        

        
def main(args=None):
    rclpy.init(args=args)

    marker_detection = MarkerDetection()

    rclpy.spin(marker_detection)

    marker_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()