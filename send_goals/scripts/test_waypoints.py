#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import random 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist





waypoints = {

    'uphill-1': [(0.93, -0.46, -0.70, 0.71), None],

    'F1': [(0.95, -3.00, -0.70, 0.71), None],

    'F-LU': [(1.57, -4.80, -0.7, 0.71), None],
    # 'F-LL': (1.34, -5.17, -0.05, 1.00),
    # 'F-RU': (0.11, -5.03, -0.81, 0.59), 
    # 'F-RR': (0.42, -5.17, -1.00, 0.04), 

    'F3': [(0.92, -2.30, 0.95, 0.30), None],
    'F2': [(0.92, -2.30, 0.33, 0.95), None],
    'uphill-2': [(0.92, -2.30, 0.71, 0.70), None],
    'downhill': [(0.94, -0.36, 0.71, 0.70), None],

    'E1': [(2.80, -0.56, -0.12, 0.99), (2.80, -1.00, -0.12, 0.99)],
    'E3': [(2.80, -0.56, -0.69, 0.72), None],

    'D1': [(2.79, -3.68, 0.26, 0.97), None],
    'D2': [(2.79, -3.68, -0.5, 0.86), None],
    'D3': [(2.79, -3.68, -0.70, 0.71), None],
    
    # 'C1': (5.13, -1.97, 0.71, 0.70),
    # 'C2': (5.13, -1.97, 0.85, 0.53),
    # 'C3': (5.13, -1.97, 0.94, 0.33),
    # 'C4': (5.13, -1.97, 1.0, 0.02), 

    'B1': [(4.98, -3.49, -0.71, 0.71), None],
    'B2': [(4.98, -3.49, -0.87, 0.49), None],
    'B3': [(4.98, -3.49, -0.95, 0.32), None],

    'turning': [(2.61, 0.07, 0.70, 0), None],
    'home': [(-0.10, 0.05, 0.70, 0.00), None]
}



def goal_pose(pose):
    goal_pose = MoveBaseGoal()

    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0]
    goal_pose.target_pose.pose.position.y = pose[1]
    goal_pose.target_pose.pose.orientation.z = pose[2]
    goal_pose.target_pose.pose.orientation.w = pose[3]

    return goal_pose

def use_backup_waypoint(waypoint_name):
    if waypoint_name in waypoints and waypoints[waypoint_name][1] is not None:
        return waypoints[waypoint_name][1]
    return None

if __name__ == '__main__':
    try:
        rospy.init_node("liu_control")
        rospy.loginfo("开始导航")
        # 创建MoveBaseAction client
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # 等待MoveBaseAction启动        
        client.wait_for_server()

        prev_waypoint_name = None

        while not rospy.is_shutdown():
            for waypoint_name, pose_pair in waypoints.items():
                main_pose, backup_pose = pose_pair
                goal = goal_pose(main_pose)
                client.send_goal(goal)
                client.wait_for_result()

                # Check if the previous waypoint was not reached successfully
                if prev_waypoint_name is not None and client.get_state() != actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("上一个坐标 %s 不可达，将使用备用坐标", prev_waypoint_name)
                    backup_goal = goal_pose(backup_pose)
                    client.send_goal(backup_goal)
                    client.wait_for_result()

                prev_waypoint_name = waypoint_name

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Liu interrupted.")
    finally:
        client.cancel_goal()
