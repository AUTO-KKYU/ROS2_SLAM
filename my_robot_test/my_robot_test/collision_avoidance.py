#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

import queue
import time
from math import inf

class PIDController:
    def __init__(self, kP, kI, kD, kS):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kS = kS
        self.err_int = 0
        self.err_dif = 0
        self.err_prev = 0
        self.err_hist = queue.Queue(self.kS)
        self.t_prev = 0

    def control(self, err, t):
        dt = t - self.t_prev
        if dt > 0.0:
            self.err_hist.put(err)
            self.err_int += err
            if self.err_hist.full():
                self.err_int -= self.err_hist.get()
            self.err_dif = err - self.err_prev
            u = (self.kP * err) + (self.kI * self.err_int * dt) + (self.kD * self.err_dif / dt)
            self.err_prev = err
            self.t_prev = t
            return u

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )
        self.robot_scan_sub = self.create_subscription(LaserScan, '/scan', self.robot_laserscan_callback, qos_profile_sensor_data)
        self.robot_ctrl_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.robot_controller_callback)
        self.laserscan = []
        self.ctrl_msg = Twist()
        self.start_time = self.get_clock().now()
        self.pid_lat = PIDController(0.2, 0.01, 0.2, 10)
        self.pid_lon = PIDController(0.1, 0.001, 0.05, 10)
        self.scan_available = False

    def robot_laserscan_callback(self, msg):
        self.laserscan = msg.ranges
        for ls in range(len(self.laserscan)):
            if self.laserscan[ls] == 0:
                self.laserscan[ls] = inf
        self.scan_available = True

    def robot_controller_callback(self):
        DELAY = 4.0
        if self.get_clock().now() - self.start_time > Duration(seconds=DELAY) and self.scan_available:
            left_scan_min = min(self.laserscan[0:30])
            right_scan_min = min(self.laserscan[330:360])
            tstamp = time.time()
            if right_scan_min - left_scan_min > 0.5:
                if left_scan_min >= 0.5:
                    LIN_VEL = self.pid_lon.control(min(3.5, self.laserscan[0]), tstamp)
                else:
                    LIN_VEL = 0.0
                ANG_VEL = -self.pid_lat.control(left_scan_min, tstamp)
            elif left_scan_min - right_scan_min > 0.5:
                if right_scan_min >= 0.5:
                    LIN_VEL = self.pid_lon.control(min(3.5, self.laserscan[0]), tstamp)
                else:
                    LIN_VEL = 0.0
                ANG_VEL = self.pid_lat.control(right_scan_min, tstamp)
            else:
                if left_scan_min <= 0.5 or right_scan_min <= 0.5:
                    LIN_VEL = 0.0
                    if right_scan_min <= left_scan_min:
                        ANG_VEL = self.pid_lat.control(right_scan_min, tstamp)
                    else:
                        ANG_VEL = -self.pid_lat.control(left_scan_min, tstamp)
                else:
                    LIN_VEL = self.pid_lon.control(min(3.5, self.laserscan[0]), tstamp)
                    ANG_VEL = 0.0
            self.ctrl_msg.linear.x = LIN_VEL
            self.ctrl_msg.angular.z = ANG_VEL
            self.robot_ctrl_pub.publish(self.ctrl_msg)
            print('Distance to closest obstacle is {} m'.format(round(min(left_scan_min, right_scan_min), 4)))
        else:
            print('Initializing...')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

