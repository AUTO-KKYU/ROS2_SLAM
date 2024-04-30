#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import numpy as np
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from ament_index_python.packages import get_package_share_directory
import yaml 
import os 

class RobotNavigator(Node):
    def __init__(self, goals_file):
        super().__init__('robot_navigator')
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        
        # 설정 파일 로드
        self.load_goals(goals_file)
        
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.subscription 


    def load_goals(self, file_path):
        with open(file_path, 'r') as file:
            self.goals = yaml.safe_load(file)

    def pose_callback(self, msg):
        current_pose = msg.pose.pose
        orientation = current_pose.orientation      
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = euler_from_quaternion(quaternion)
        print(f"Current pose in Euler angles: {self.convert_degree(euler)} degrees")

    def to_quaternion(self, roll, pitch, yaw):
        return quaternion_from_euler(roll * np.pi / 180, pitch * np.pi / 180, yaw * np.pi / 180)

    def convert_degree(self, radian_input):
        return np.array(radian_input) * 180. / np.pi

    def set_goal_pose(self, x, y, z, roll, pitch, yaw):
        quaternion = self.to_quaternion(roll, pitch, yaw)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]
        return goal_pose

    def navigate_to_pose(self, goal_id):
        goal_data = self.goals[goal_id]
        x, y, z = goal_data['x'], goal_data['y'], goal_data['z']
        roll, pitch, yaw = goal_data['roll'], goal_data['pitch'], goal_data['yaw']
        goal_pose = self.set_goal_pose(x, y, z, roll, pitch, yaw)

        self.nav.goToPose(goal_pose)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            if feedback and feedback.distance_remaining % 5 == 0:
                print(f'Distance remaining: {feedback.distance_remaining:.2f} meters.')
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=10.0):
                    self.nav.cancelTask()

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')

def main():
    rclpy.init()
    robot_navigator = RobotNavigator('/home/kkyu/my_mobile/src/my_robot_test/config/nav2.yaml')
    try:
        robot_navigator.navigate_to_pose('goal1')  # 여기서 'goal1'은 YAML 파일에 정의된 목표의 키
        rclpy.spin(robot_navigator)
    except KeyboardInterrupt:
        pass
    finally:
        robot_navigator.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()