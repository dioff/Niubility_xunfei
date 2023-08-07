#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def talker():
    
    # 发布者 pub 订阅 ucar_voice_topic
    pub = rospy.Publisher('ucar_voice_topic', String, queue_size=10)
    # 新建 talker 节点
    rospy.init_node('publisher_demo', anonymous=True)

    # 发布消息
    rate = rospy.Rate(0.03)
    while not rospy.is_shutdown():
        hello_str = "A区是小麦植株 B区是黄瓜植株 C区是水稻植株 D区是玉米植株 黄瓜的数量为6 玉米的数量为5 西瓜的数量为4"
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
