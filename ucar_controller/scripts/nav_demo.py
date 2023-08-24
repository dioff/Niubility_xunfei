#i/usr/bin/env python
# _*_ coding: UTF-8 _*_
import rospy
import random
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



class PatrolNav():


    def _init_ (self):
        rospy.init_node('patrol_nav_node',anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.rest_time = rospy.get_param("~rest_time",5)
        self.keep_patrol = rospy.get_param("~keep_patrol",False)
        self.random_patrol = rospy.get_param("~random_patrol",False)
        self.patrol_type = rospy.get_param("~patrol_type",0)
        self.patrol_loop = rospy.get_param("~patrol_loop",2)
        self.patrol_time = rospy.get_param("~patrol_time",5)
        self.locations = dict()
        self.locations['one'] = Pose(Point11,-1.590,0.000),Quaternion(0.000,0.000,0.006,0.999))
        self.locations['two'] = Pose(Point(13.8,12.33,0.000),Quaternion(0.000,0.000,0.99,0.038))
        self.locations['three'] = Pose(Point(5.53,12.48,0.000),Quaternion(0.000,0.000,0.704,0.7097))
        self.locations['four'] = Pose(Point(2.559,9.689,0.000),Quaternion(0.000,0.000,0.999,0.012))
        self.locations['five'] = Pose(Point(6.886,1.490,0.000),Quaternion(0.000,0.000,0.823,0.567))
        self.locations['six'] = Pose(Point(1.765,2.084,0.000),Quaternion(0.000,0.000,-0.005,0.999))
        goal_states = ['PENDING','ACTIVE','PREEMPTED','SUCCEEDED','ABORTED',
                        'REJECTED', 'PREEMPTING','RECALLING','RECALLED','LOST']

        self.move_base = actionlib.SimpleActionClient("move_base",MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(30))
        rospy.loginfo("Connected to move base server")

        n_successes = 0
        target_num = 0
        location = ""
        start_time = rospy.Time.now()
        locations_cnt = len(self.locations)
        sequeue = ['one','two','three','four','five','six']
        rospy.loginfo("Starting positive navigation ")
        
        while not rospy.is_shutdown():
            location = sequeue[target_num]
            rospy.loginfo("target_num-value:"+str(target_num))

            rospy.loginfo("GOing to: " + str(location))
            self.send_goal(location)

            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.logerr("ERROR:Timed out achieving goal")
            else:
                state = self.move_base.get_state()

            target_num +=1
            if target_num > 5:
                target_num = 0
    def send_goal(self,locate):
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = self.locations[locate]
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base.send_goal(self.goal)
    
    def shutdown(self):
        rospy.logwarn("Stopping the patrol...")

if _name_ == '_main_':
    try:
        PatrolNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("Patrol navigation exception finished.")
