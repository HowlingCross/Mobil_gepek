import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
import math

class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower_node')

        # publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)


        self.target_reached_publisher = self.create_publisher(Bool, '/waypoint_reached', 1)

        self.current = Odometry()
        self.next = PoseStamped()
        self.is_initialized = False #

        # Feliratkozok
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/target_waypoint',
            self.target_callback,
            1)
        
        self.timer = self.create_timer(0.1 , self.control_loop)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1)

    
    def odom_callback(self, msg: Odometry):
        self.current = msg

    def target_callback(self, msg: PoseStamped):
        self.next = msg
        self.is_initialized = True

    def control_loop(self):
        target_x = self.next.pose.position.x
        target_y = self.next.pose.position.y
        current_x = self.current.pose.pose.position.x
        current_y = self.current.pose.pose.position.y

        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        if distance > 0.1:
            linear_speed = 0.2
            self.publish_cmd_vel(linear_speed) ## addig megyunk mig meg nem kozelitjuk

        if distance < 0.1: 
            self.publish_cmd_vel(0.0) ## ha kozelebe vagyink ne menjunk tovabb
            self.send_reached_signal()
            self.is_initialized = False
    
    def publish_cmd_vel(self, linear_x):
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_x
        cmd_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_msg)
    
    def send_reached_signal(self):
        msg = Bool()
        msg.data = True
        self.target_reached_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    follower_node = WaypointFollower()
    rclpy.spin(follower_node)
    follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()