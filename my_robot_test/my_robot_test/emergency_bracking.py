import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy 
from rclpy.duration import Duration 
from math import pi 

class RobotController(Node):
    def __init__(self):
        info = '\nMake the robot stop if obstacles are detected within a threshold distance.\n'
        print(info)
        super().__init__('robot_controller') 
        qos_profile = QoSProfile( 
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, 
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, 
        depth=10 
        )
        self.robot_scan_sub = self.create_subscription(LaserScan, '/scan', self.robot_laserscan_callback, qos_profile) 
        self.robot_scan_sub 
        self.robot_ctrl_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile) 
        timer_period = 0.001 
        self.timer = self.create_timer(timer_period, self.robot_controller_callback) 
        self.laserscan = [] 
        self.ctrl_msg = Twist() 
        self.start_time = self.get_clock().now() 

    def robot_laserscan_callback(self, msg):
        self.laserscan = msg.ranges

    def robot_controller_callback(self):
        THRESH = 1.0 # Distance threshold to perform emergency braking (m)
        LIN_VEL = 1.0 # Linear velocity (m/s)
        ANG_VEL = 0.0 # Angular velocity (rad/s)
        DELAY = 4.0 # Time delay (s)
        if self.get_clock().now() - self.start_time > Duration(seconds=DELAY):
            if self.laserscan[0] >= THRESH: 
                self.ctrl_msg.linear.x = LIN_VEL # Set linear velocity
                self.ctrl_msg.angular.z = ANG_VEL # Set angular velocity
                self.robot_ctrl_pub.publish(self.ctrl_msg) 
                print('Obstacle detected at {} m'.format(self.laserscan[0]))
            else:
                self.ctrl_msg.linear.x = 0.0 # Set linear velocity
                self.ctrl_msg.angular.z = 0.0 # Set angular velocity
                self.robot_ctrl_pub.publish(self.ctrl_msg) 
                print('Emergency braking performed!')
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
