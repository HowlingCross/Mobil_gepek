#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose
import math

# --- KONSTANSOK AZ 11X11-ES TÉRBEN ---
BETU_MERET = 1.25  # Betűk mérete (kb. 1.25 egység magasak lesznek)
KOZ = 0.25         # Távolság a betűk között
START_X = 1.0      # Kezdő X koordináta a bal oldalról (11x11-hez igazítva)
START_Y = 5.5      # Y középvonal

# Általános sebesség beállítások
LINEAR_SPEED = 1.5
ANGULAR_SPEED = 1.0

class AndrasHandwrittenNode(Node):

    def __init__(self):
        super().__init__("andras_handwritten_drawer")
        self.get_logger().info("ANDRAS írott rajzoló node elindult.")

        # Publisher / Subscriber / Service Clients
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        
        # Állapot kezelés
        self.current_pose = Pose()
        self.betuk = ["A", "N", "D", "R", "A", "S"]
        self.betu_index = 0
        self.szakasz_szamlalo = 0  # Timer futások száma a jelenlegi szakaszban
        self.teleport_sikeres = False
        
        # Kezdeti beállítások
        self.clear_background()
        self.init_turtle()

        # Időzítő (Timer) a sebességparancsok folyamatos küldéséhez (20 Hz)
        self.timer_ = self.create_timer(0.05, self.timer_callback)

    def pose_callback(self, msg: Pose):
        """A teknősbéka aktuális pozíciójának frissítése."""
        self.current_pose = msg

    # --- SZERVÍZ SEGÉD METÓDUSOK ---

    def clear_background(self):
        """Beállítja a tollat és törli a hátteret."""
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetPen service nem elérhető, várunk...')
        pen_req = SetPen.Request(r=0, g=0, b=0, width=3, off=0) # Fekete toll, lent
        self.pen_client.call_async(pen_req)

    def init_turtle(self):
        """A teknősbéka kezdő pozícióba teleportálása."""
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service nem elérhető, várunk...')
        
        # Teleportáljon az Y középvonalhoz, balra fordulva (jobb a görbékhez)
        teleport_req = TeleportAbsolute.Request(x=START_X, y=START_Y - BETU_MERET / 2, theta=math.pi/2)
        future = self.teleport_client.call_async(teleport_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
             self.teleport_sikeres = True
             self.get_logger().info('Kezdő teleportálás sikeres.')

    def set_pen_state(self, off):
        """ Toll felemelése (off=1) vagy leengedése (off=0) """
        pen_req = SetPen.Request(r=0, g=0, b=0, width=3, off=off)
        self.pen_client.call_async(pen_req)

    # --- RAJZOLÁSI LOGIKA ---

    def draw_betu(self, betu: str, cmd: Twist):
        """ 
        A betű állapotgépe. Minden betű külön fázisokra oszlik.
        A sebességparancsokat adja vissza.
        """
        
        if betu == "A":
            if self.szakasz_szamlalo < 30:  # 1. Ív fel (A hurok alsó része)
                cmd.linear.x = LINEAR_SPEED
                cmd.angular.z = ANGULAR_SPEED * 1.5
            elif self.szakasz_szamlalo < 60: # 2. Egyenes le
                cmd.linear.x = LINEAR_SPEED
                cmd.angular.z = -ANGULAR_SPEED * 0.5
            elif self.szakasz_szamlalo < 90: # 3. Záró 'szár' a következő betűhöz
                cmd.linear.x = LINEAR_SPEED
                cmd.angular.z = -ANGULAR_SPEED * 0.8
            else:
                return True # Befejezve
            
        elif betu == "N":
            if self.szakasz_szamlalo < 30:  # 1. Ív fel
                cmd.linear.x = LINEAR_SPEED
                cmd.angular.z = ANGULAR_SPEED * 1.5
            elif self.szakasz_szamlalo < 60: # 2. Ív le
                cmd.linear.x = LINEAR_SPEED
                cmd.angular.z = -ANGULAR_SPEED * 1.0
            elif self.szakasz_szamlalo < 90: # 3. Ív fel a következő betűhöz
                cmd.linear.x = LINEAR_SPEED
                cmd.angular.z = ANGULAR_SPEED * 1.0
            else:
                return True
                
        elif betu == "D":
            if self.szakasz_szamlalo < 60:  # 1. Hosszú 'C' ív (a D hasa)
                cmd.linear.x = LINEAR_SPEED * 0.7
                cmd.angular.z = ANGULAR_SPEED * 2.5
            elif self.szakasz_szamlalo < 90: # 2. Egyenes szár
                cmd.linear.x = LINEAR_SPEED * 1.2
                cmd.angular.z = 0.0
            elif self.szakasz_szamlalo < 120: # 3. Záró ív
                cmd.linear.x = LINEAR_SPEED * 0.8
                cmd.angular.z = -ANGULAR_SPEED * 1.0
            else:
                return True
                
        elif betu == "R":
            if self.szakasz_szamlalo < 30:  # 1. Függőleges vonal
                cmd.linear.x = LINEAR_SPEED * 1.5
                cmd.angular.z = 0.0
            elif self.szakasz_szamlalo < 60: # 2. Félkörív
                cmd.linear.x = LINEAR_SPEED * 0.5
                cmd.angular.z = ANGULAR_SPEED * 2.0
            elif self.szakasz_szamlalo < 90: # 3. Ferde szár
                cmd.linear.x = LINEAR_SPEED * 1.2
                cmd.angular.z = -ANGULAR_SPEED * 0.5
            else:
                return True

        elif betu == "S":
            if self.szakasz_szamlalo < 40:  # 1. Első ív
                cmd.linear.x = LINEAR_SPEED * 0.8
                cmd.angular.z = ANGULAR_SPEED * 1.8
            elif self.szakasz_szamlalo < 80: # 2. Második ív
                cmd.linear.x = LINEAR_SPEED * 0.8
                cmd.angular.z = -ANGULAR_SPEED * 1.8
            elif self.szakasz_szamlalo < 110: # 3. Záró kötővonal
                cmd.linear.x = LINEAR_SPEED * 1.5
                cmd.angular.z = ANGULAR_SPEED * 0.2
            else:
                return True
                
        return False # Még nem fejeztük be a rajzolást
        
    # --- TIMER FŐ LÉPÉS ---

    def timer_callback(self):
        if not self.teleport_sikeres:
            return

        cmd = Twist()

        if self.betu_index >= len(self.betuk):
            # Rajzolás befejezve
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_publisher_.publish