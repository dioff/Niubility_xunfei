#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class OdomEKF:
    def __init__(self):
        rospy.init_node('odom_ekf', anonymous=False)
        self.ekf_pub = rospy.Publisher('odom_ekf', Odometry, queue_size=10)
        rospy.Subscriber('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.ekf_callback)
        
    def ekf_callback(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = 'base_link'
        odom.pose = msg.pose

        odom.pose.covariance = msg.pose.covariance
        
        self.ekf_pub.publish(odom)

if __name__ == '__main__':
    try:
        odom_ekf = OdomEKF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
