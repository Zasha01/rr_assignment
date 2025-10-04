import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

def sector_min(ranges, angle_min, angle_increment, start_deg, end_deg):
    start = int((math.radians(start_deg) - angle_min) / angle_increment)
    end = int((math.radians(end_deg) - angle_min) / angle_increment)
    i0, i1 = sorted([max(0, start), min(len(ranges)-1, end)])
    vals = [r for r in ranges[i0:i1+1] if math.isfinite(r)]
    return min(vals) if vals else float('inf')

class Guest(Node):
    def __init__(self):
        super().__init__('guest_controller')
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.on_tick)
        self.last_scan = None
        self.kp = 1.4
        self.kf = 0.8
        self.d_desired = 0.5
        self.safety = 0.6
        self.v_mean = 0.18
        self.A = 0.10
        self.freq = 0.25
        self.t0 = time.time()

    def on_scan(self, msg):
        self.last_scan = msg

    def on_tick(self):
        if self.last_scan is None:
            return
        s = self.last_scan
        d_right = sector_min(s.ranges, s.angle_min, s.angle_increment, -100, -20)
        d_front = sector_min(s.ranges, s.angle_min, s.angle_increment, -20, 20)

        cmd = Twist()
        e = self.d_desired - d_right
        front_term = max(0.0, self.safety - d_front)
        cmd.angular.z = self.kp * e + self.kf * front_term

        t = time.time() - self.t0
        v = self.v_mean + self.A * math.sin(2.0 * math.pi * self.freq * t)
        v = max(0.05, v)          # never fully stop during the wave
        if d_front < 0.35:
            v = 0.0
        cmd.linear.x = v
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(Guest())
    rclpy.shutdown()
