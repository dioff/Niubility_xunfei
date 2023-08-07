#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


def laser_callback(msg):
    # global move_forward
    target_distance = 0.4  # 目标距离
    linear_velocity = 0.5  # 线速度

    # 获取左侧的距离值
    # left_distance = msg.ranges[-180]
    right_distance = msg.ranges[180]
           # 处理距离值为inf的情况
    # if math.isinf(left_distance):
    #     left_distance = float('inf')
    rospy.loginfo("right distance: %.2f", right_distance)

    if right_distance <= target_distance or math.isinf(right_distance):
        cmd_vel = Twist()
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = 0.0
        cmd_vel_pub.publish(cmd_vel)
        rospy.loginfo("stop = %.2f", cmd_vel.linear.y)
    elif right_distance > target_distance:
        cmd_vel = Twist()
        # 距离大于目标距离，向左移动
        cmd_vel.linear.y = -0.2
        cmd_vel.angular.z = 0.0
        cmd_vel_pub.publish(cmd_vel)
        rospy.loginfo("go right = %.2f", cmd_vel.linear.y)


if __name__ == '__main__':
    rospy.init_node('radar_distance_control')
    laser_sub = rospy.Subscriber('/scan', LaserScan, laser_callback, queue_size=10)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.spin()
