#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import random 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int8
import cv2
import os
import time

waypoints = {

    # 'uphill-1': (0.93, -0.46, -0.70, 0.71),

    # 'F1': (0.94, -3.25, -0.70, 0.71),

    # # 'F-LU': (1.74, -4.95, -0.66, 0.74),
    # 'F-LL': (1.42, -5.35, -0.07, 1.00),
    # # 'F-RU': (0.07, -5.03, -0.70, 0.71), 
    # # 'F-RR': (0.28, -5.40, -1.00, 0.03), 

    # 'F3': (0.92, -2.30, 0.95, 0.30),  
    # 'F2': (0.92, -2.30, 0.33, 0.95),
    # 'uphill-2': (0.92, -2.30, 0.71, 0.70),
    # 'downhill': (0.93, -0.36, 0.71, 0.70),
    # 'turn': (0.94, -0.36, 0.00, 1.00),

    # 'E1': (2.80, -0.56, -0.12, 0.99),
    # 'E2': (2.80, -0.56, -0.46, 0.89),
    # 'E3': (2.80, -0.56, -0.90, 0.44),
    # 'E4': (2.80, -0.56, -0.92, 0.38),

    'D1': (2.79, -3.68, 0.26, 0.97),
    # 'D2': (2.79, -3.68, -0.5, 0.86),
    # 'D3': (2.79, -3.68, -0.70, 0.71),
    # 'D4': (2.79, -3.68, -0.85, 0.53),

    'C1': (5.13, -1.97, 0.71, 0.70),
    # 'C2': (5.13, -1.97, 0.85, 0.53),
    # 'C3': (5.13, -1.97, 0.94, 0.33),
    # 'C4': (5.13, -1.97, 1.0, 0.02), 

    'B1': (4.98, -3.49, -0.71, 0.71),
    # 'B2': (4.98, -3.49, -0.87, 0.49),
    # 'B3': (4.98, -3.49, -0.95, 0.32),
    # 'B4': (4.98, -3.49, 1.00, 0.04),

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

if __name__ == '__main__':
    try:
        rospy.init_node("liu_contorl")
        rospy.loginfo("开始导航")
        # 创建MoveBaseAction client
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # 等待MoveBaseAction启动        
        client.wait_for_server()

        for waypoint_name, pose in waypoints.items():
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Liu interrupted.")

        
