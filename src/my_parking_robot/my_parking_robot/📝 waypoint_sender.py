import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool 
import math
import os
from ament_index_python.packages import get_package_share_directory 

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender_node')
        
        # 1. PARAMÉTEREK ÉS ÁLLAPOT
        self.declare_parameter('text_file_name', 'path.txt')
        self.text_file_name = self.get_parameter('text_file_name').value

        self.all_waypoints = []
        self.current_waypoint_index = 0
        self.num_waypoints = 0

        # 2. PUBLIKÁLÓK / FELIRATKOZÓK
        self.waypoint_publisher = self.create_publisher(PoseStamped, "/target_waypoint", 1) 
        self.reached_subscription = self.create_subscription(
            Bool,
            '/waypoint_reached',
            self.reached_callback,
            1)
        
        # 3. ÚTPONTOK BETÖLTÉSE ÉS INDULÁS
        try:
            if not self.load_waypoints():
                return # Ha nem sikerült betölteni, ne indítsuk el a timert
        except:
            return

        # ELSŐ ÚTPONT KÜLDÉSÉNEK IDŐZÍTÉSE
        self.timer = self.create_timer(0.5, self.initial_publish)
                
    # Útpontok betöltése a fájlból (X, Y formátumban)
    def load_waypoints(self):
        try:
            pkg_share_dir = get_package_share_directory('my_parking_robot')
            full_path = os.path.join(pkg_share_dir, 'param', self.text_file_name)
            
            with open(full_path, 'r') as f:
                path_data = f.readlines()

            for line in path_data:
                # Csak X és Y-t olvasunk be
                values = [float(v.strip()) for v in line.split(',') if v.strip()]
                
                if len(values) >= 2:
                    x, y = values[0], values[1]
                    
                    msg_pose = PoseStamped()
                    msg_pose.header.frame_id = "map"
                    msg_pose.pose.position.x = x
                    msg_pose.pose.position.y = y
                    msg_pose.pose.position.z = 0.0      # Fixen 0.0
                    msg_pose.pose.orientation.w = 1.0   # Fixen előre néz (Yaw=0)
                    
                    self.all_waypoints.append(msg_pose)
            
            self.num_waypoints = len(self.all_waypoints)
            return self.num_waypoints > 0

        except:
            # A legegyszerűbb hibaeset kezelés
            return False

    # Az első útpont elküldése (timer callback)
    def initial_publish(self):
        self.publish_current_waypoint()
        self.timer.cancel() # Leállítja az indító időzítőt
        
    
    # Az aktuális útpont publikálása
    def publish_current_waypoint(self):
        if 0 <= self.current_waypoint_index < self.num_waypoints:
            current_pose_stamped = self.all_waypoints[self.current_waypoint_index]
            current_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            self.waypoint_publisher.publish(current_pose_stamped)
        
    # Visszajelzés fogadása és a következő útpont küldése
    def reached_callback(self, msg: Bool):
        if msg.data and self.current_waypoint_index < self.num_waypoints: 
            self.current_waypoint_index += 1
            self.publish_current_waypoint()


def main(args=None):
    rclpy.init(args=args)
    waypoint_sender = WaypointSender()
    rclpy.spin(waypoint_sender)
    waypoint_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()