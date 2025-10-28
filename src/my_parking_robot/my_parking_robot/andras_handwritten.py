#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose
import math

# --- KONSTANSOK ---
PI = math.pi
BETU_MERET = 3.0           # Nagyobb méret egyetlen betűhöz
START_X = 5.5              # Középre igazítás X tengelyen
START_Y = 5.5              # Középvonal Y

# Vezérlő paraméterek (P-kontroller)
LINEAR_SPEED = 2.5         # Alap sebesség
K_ANGULAR = 5.0            # Szög-korrekció (gyors és pontos fordulás)
DISTANCE_TOLERANCE = 0.1   # Célpont elérésének tűréshatára

class AndrasHandwrittenNode(Node):

    def __init__(self):
        super().__init__("handwritten_A_drawer")
        self.get_logger().info("Rajzolás: Írott nagy 'A' betű.")

        # Publisher / Subscriber / Service Clients
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        
        # Állapot kezelés
        self.current_pose = Pose()
        # Csak az "A" betűt rajzoljuk!
        self.betu_index = 0
        self.szakasz_index = 0
        self.teleport_sikeres = False
        self.is_moving = False
        
        # Célpont és rajzoló adatok
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.A_rajzolo = self._get_handwritten_A() # Csak az A betű adatai
        
        self.clear_background()
        self.init_turtle()

        # Időzítő (50 Hz)
        self.timer_ = self.create_timer(0.02, self.timer_callback)

    def pose_callback(self, msg: Pose):
        self.current_pose = msg

    # --- SZERVÍZ SEGÉD METÓDUSOK ---

    def clear_background(self):
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            pass
        pen_req = SetPen.Request(r=0, g=0, b=0, width=4, off=0) # Vastagabb toll
        self.pen_client.call_async(pen_req)

    def init_turtle(self):
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            pass
        
        # Kezdő teleportálás (az A betű kezdőpontja, alsó közép)
        start_y_corrected = START_Y - BETU_MERET / 2
        teleport_req = TeleportAbsolute.Request(x=START_X, y=start_y_corrected, theta=PI/2) 
        future = self.teleport_client.call_async(teleport_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
             self.teleport_sikeres = True
             self.get_logger().info('Teleportálás sikeres. Rajzolás indul.')
             self.set_next_goal()

    def set_pen_state(self, off, r=0, g=0, b=0):
        pen_req = SetPen.Request(r=r, g=g, b=b, width=4, off=off)
        self.pen_client.call_async(pen_req)

    # --- RAJZOLÓ KOORDINÁTÁK: Csak az 'A' ---

    def _get_handwritten_A(self):
        """ Az írott nagy 'A' betű rajzolásának pontjai. """
        M = BETU_MERET # Magasság
        W = BETU_MERET * 0.7 # Szélesség
        
        # Az "A" írott betű célpontjai:
        return [
            # 1. Alap ív fel
            (-W/2, -M/2, False),      # 1. Kezdőpont: Alul balra (Toll fel)
            (0.0, M/2, True),         # 2. Fel-középre (Toll le)
            # 2. Ív le
            (W/2, -M/2, True),        # 3. Le-jobbra
            # 3. Kötővonal / záró szár
            (W/2, -M/2, False)        # 4. Kész, Toll fel, ugrás a végpontra (itt vége)
        ]

    # --- ÁLLAPOT KEZELÉS ---

    def set_next_goal(self):
        """ Megadja a következő célpontot. """
        
        if self.szakasz_index >= len(self.A_rajzolo):
            return True # Rajzolás befejezve
        
        rel_x, rel_y, pen_down = self.A_rajzolo[self.szakasz_index]
        
        # A cél abszolút koordinátái (egyetlen betű a képernyő közepén)
        self.goal_x = START_X + rel_x
        self.goal_y = START_Y + rel_y
        
        self.set_pen_state(off=0 if pen_down else 1)
        self.is_moving = True
        self.szakasz_index += 1
        return False # Még nem fejeztük be


    # --- TIMER FŐ LÉPÉS ÉS KONTROLLER LOGIKA ---

    def timer_callback(self):
        if not self.teleport_sikeres or not self.is_moving:
            return

        cmd = Twist()
        
        dx = self.goal_x - self.current_pose.x
        dy = self.goal_y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < DISTANCE_TOLERANCE:
            # Célpont elérése
            self.cmd_vel_publisher_.publish(Twist()) # Állj meg
            
            if self.set_next_goal():
                # Rajzolás teljesen befejezve
                self.is_moving = False
                self.timer_.cancel()
                self.get_logger().info("Az írott nagy 'A' betű kirajzolva. Node leállítva.")
            return
            
        # --- P-KONTROLLER LOGIKA ---
        
        desired_theta = math.atan2(dy, dx)
        angle_diff = desired_theta - self.current_pose.theta
        
        # Normalizálás
        if angle_diff > PI:
            angle_diff -= 2 * PI
        if angle_diff < -PI:
            angle_diff += 2 * PI
            
        cmd.angular.z = K_ANGULAR * angle_diff
        cmd.linear.x = LINEAR_SPEED * math.tanh(distance) 
        
        self.cmd_vel_publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AndrasHandwrittenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()