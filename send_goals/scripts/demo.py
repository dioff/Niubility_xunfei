#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import rospy
import actionlib
import random 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
import cv2

waypoints = {

    'uphill-1': (0.93, -0.46, -0.70, 0.71),

    'F1': (0.95, -3.00, -0.70, 0.71),

    'F-LU': (1.57, -4.80, -0.7, 0.71),
    # 'F-LL': (1.34, -5.17, -0.05, 1.00),
    # 'F-RU': (0.11, -5.03, -0.81, 0.59), 
    # 'F-RR': (0.42, -5.17, -1.00, 0.04), 

    'F3': (0.92, -2.30, 0.95, 0.30),  
    'F2': (0.92, -2.30, 0.33, 0.95),
    'uphill-2': (0.92, -2.30, 0.71, 0.70),
    'downhill': (0.94, -0.36, 0.71, 0.70),
    # 'turn': (0.94, -0.36, 0.00, 1.00),

    'E1': (2.80, -0.56, -0.12, 0.99),
    'E2': (2.80, -0.56, -0.42, 0.90),
    'E3': (2.80, -0.56, -0.69, 0.72),
    # 'E4': (2.80, -0.56, -0.92, 0.38),

    'D1': (2.79, -3.68, 0.26, 0.97),
    'D2': (2.79, -3.68, -0.5, 0.86),
    'D3': (2.79, -3.68, -0.70, 0.71),
    # 'D4': (2.79, -3.68, -0.85, 0.53),

    # 'C1': (5.13, -1.97, 0.71, 0.70),
    # 'C2': (5.13, -1.97, 0.85, 0.53),
    # 'C3': (5.13, -1.97, 0.94, 0.33),
    # 'C4': (5.13, -1.97, 1.0, 0.02), 

    'B1': (4.98, -3.49, -0.71, 0.71),
    'B2': (4.98, -3.49, -0.87, 0.49),
    'B3': (4.98, -3.49, -0.95, 0.32),
    'B4': (4.98, -3.49, 1.00, 0.04),

    'turning': (2.61, 0.07, 0.70, 0),

    'home': (-0.10, 0.05, 0.70, 0.00)
}

def goal_pose(pose):
    goal_pose = MoveBaseGoal()

    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0]
    goal_pose.target_pose.pose.position.y = pose[1]
    goal_pose.target_pose.pose.orientation.z = pose[2]
    goal_pose.target_pose.pose.orientation.w = pose[3]

    return goal_pose

class liu_control:
    def __init__(self) :
        self.cap = cv2.VideoCapture("/dev/vide0")
        ret, frame = self.cap.read()

    def save_img(self, index, num, num_wait):
        for i in range(num_wait):
            ret, frame = self.cap.read()
            frame = cv2.flip(frame, 1)
        cv2.imwrite("/home/ucar/catkin_test_ws/src/image1" + str(index) + "_" + str(num) + ".jpg", frame)

    def cap_release(self):
        self.cap.release()

if __name__ == '__main__':
    try:
        rospy.init_node("liu_demo")
        rospy.loginfo("开始测试")
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        liu = liu_control()
        coun1 = 0
        coun2 = 0
        coun3 = 0
        coun4 = 0
        coun5 = 0
        while True:
            for waypoint_name, pose in waypoints.items():
                goal = goal_pose(pose)
                client.send_goal(goal)
                client.wait_for_result()
                if waypoint_name in ['E1', 'E2', 'E3', 'E4']:
                    liu.save_img(1, coun1, 10)
                    coun1 += 1

                elif waypoint_name in ['D1', 'D2', 'D3', 'D4']:
                    liu.save_img(2, coun2, 10)
                    coun2 += 1

                elif waypoint_name in ['C1', 'C2', 'C3', 'C4']:
                    liu.save_img(3, coun3, 10)
                    coun3 += 1
                
                elif waypoint_name in ['B1', 'B2', 'B3']:
                    liu.save_img(4, coun4, 10)
                    coun4 += 1
                
                elif waypoint_name in ['F1', 'F-LU', 'F-LL', 'F-RU', 'F-RR', 'F3', 'F2']:
                    liu.save_img(5, coun5, 10)
                    coun5 += 1
                elif waypoint_name in ['B4', 'C4']:
                    liu.yolo_start()
                # elif waypoint_name == 23:
                    rospy.loginfo("回家等待yolo识别结果")
                
            liu.cap_release()
            
            rospy.spin()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Liu interrupted.")
    finally:
        liu.cap_release()
        liu.cmd_vel_pub.publish(Twist())