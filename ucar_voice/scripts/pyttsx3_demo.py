#!/usr/bin/env python


import pyttsx3
import io
import sys
import rospy
from std_msgs.msg import String


def voice_sound(text):
 
    engine = pyttsx3.init()
    
    # 获取语音包
    voices = engine.getProperty('voices')
    # for voice in voices:
    #     print ('id = {}\tname = {}'.format(voice.id, voice.name))
    
    # 设置使用的语音包
    # engine.setProperty('voice', 'zh') #开启支持中文
    engine.setProperty('voice', 'zh', voices[0].id) # 女声？
    
    # 改变语速  范围为0-200   默认值为200
    rate = engine.getProperty('rate')  #获取当前语速
    engine.setProperty('rate', rate-100)
    
    # 设置音量  范围为0.0-1.0  默认值为1.0
    engine.setProperty('volume', 0.7)
    
    # 预设要朗读的文本数据
    line = "你好，世界！" #要播报的内容
    engine.say(text)
    
    # 朗读
    engine.runAndWait()


# 回调函数定义
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    voice_sound(data.data)
    
def ucar_voice():

    # 包名叫 ucar_voice
    # 节点名叫 pyttsx3_demo
    # 话题名叫 ucar_voice_topic
    rospy.init_node('pyttsx3_demo', anonymous=True)
    rospy.Subscriber("ucar_voice_topic", String, callback)

    # 进入循环，等待接收消息
    rospy.spin()

if __name__ == '__main__':
    try:
        ucar_voice()
    except rospy.ROSInterruptException:
        pass
