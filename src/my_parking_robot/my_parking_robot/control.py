#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from tf_transformations import euler_from_quaternion


class PIDControl(Node):
    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter('Kp', 1.2)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.2)
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value

        self.declare_parameter('max_speed', 0.16)
        self.max_speed = self.get_parameter('max_speed').value
        self.declare_parameter('max_angular_speed', 0.3)
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        self.path = Path()
        self.path_index = 0
        self.path_received = False
        self.odom = Odometry()

        self.integral = 0.0
        self.previous_error = 0.0
        self.dt = 0.1

        self.sub_path = self.create_subscription(Path, "/path", self.callback_path, 1)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 1)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 1)

        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info("PID path")

    def callback_path(self, msg: Path):
        if not msg.poses:
            return
        self.path = msg
        self.path_index = 0
        self.path_received = True
        self.get_logger().info(f"Következő póz {len(msg.poses)}")

    def callback_odom(self, msg: Odometry):
        self.odom = msg

    def timer_callback(self):
        if not self.path_received:
            return

        # Ha az útvonal véget ért
        if self.path_index >= len(self.path.poses):
            self.get_logger().info(" Path követése befejezve.")
            self.path_received = False
            cmd = Twist()
            self.pub_cmd.publish(cmd) 
            return

        # Következő pont a txt fájban
        target_pose = self.path.poses[self.path_index]
        dx = target_pose.pose.position.x - self.odom.pose.pose.position.x
        dy = target_pose.pose.position.y - self.odom.pose.pose.position.y
        dist = math.sqrt(dx**2 + dy**2)

        
        if dist < 0.1:
            self.path_index += 1
            self.get_logger().info(f"➡️ Következő pont: {self.path_index}/{len(self.path.poses)}")
            return

        target_yaw = math.atan2(dy, dx)
        q = [self.odom.pose.pose.orientation.x,
             self.odom.pose.pose.orientation.y,
             self.odom.pose.pose.orientation.z,
             self.odom.pose.pose.orientation.w]
        current_yaw = euler_from_quaternion(q)[2]
        angular_error = math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))

    
        self.integral += angular_error * self.dt
        derivative = (angular_error - self.previous_error) / self.dt
        angular_output = self.Kp * angular_error + self.Ki * self.integral + self.Kd * derivative
        angular_output = max(min(angular_output, self.max_angular_speed), -self.max_angular_speed)

        linear_output = min(self.max_speed, dist)

        cmd = Twist()
        cmd.linear.x = linear_output
        cmd.angular.z = angular_output
        self.pub_cmd.publish(cmd)

        self.previous_error = angular_error


def main(args=None):
    rclpy.init(args=args)
    node = PIDControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
