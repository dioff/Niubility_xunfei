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

def analyze_result():

    result_file_path = '/home/ucar/catkin_test_ws/src/image/result.txt'
    # 定义植物和水果的映射关系
    plant_type_map = {
        'plant_cucumber': '黄瓜植株',
        'plant_corn': '玉米植株',
        'plant_wheat': '小麦植株',
        'plant_rice': '水稻植株',
    }
    fruit_type_map = {
        'fruit_corn': '玉米果实',
        'fruit_watermelon': '西瓜果实',
        'fruit_cucumber': '黄瓜果实',
    }

    # 初始化统计结果的字典
    room_plants = {}
    fruit_counts = {}

    # 读取result.txt文件并进行统计
    with open(result_file_path, 'r') as file:
        lines = file.readlines()

    # 处理每一行的内容，统计植物类型
    for i, line in enumerate(lines):
        # 判断当前行是否为空，如果为空，则随机选择一个未知植株
        if not line.strip():
            room_plants[f'Room_{i+1}'] = '未知植株'
        else:
            plant_types = line.strip().split(' ')
            plant_counts = {plant_type: plant_types.count(plant_type) for plant_type in plant_types}
            max_plant = max(plant_counts, key=plant_counts.get)
            room_plants[f'Room_{i+1}'] = plant_type_map.get(max_plant, '未知植株')

    # 统计水果类型
    fruits = lines[3].strip().split(' ')
    for fruit in fruits:
        fruit_counts[fruit] = fruit_counts.get(fruit, 0) + 1

    # 找出最多的水果种类和数量
    max_fruit = max(fruit_counts, key=fruit_counts.get)
    max_fruit_count = fruit_counts[max_fruit]

    # 随机选择一个没有出现过的植株作为未知果实的替代
    used_plants = set(room_plants.values())
    unknown_plants = list(set(plant_type_map.values()) - used_plants)
    for i, room in enumerate(room_plants):
        if room_plants[room] == '未知植株':
            if unknown_plants:
                room_plants[room] = random.choice(unknown_plants)
                unknown_plants.remove(room_plants[room])

    # 构造输出字符串                ，D区域种植的作物为{}
    result_string = "任务完成E区域种植的作物为{}D区域种植的作物为{}B区域种植的作物为{}F区域存放的果实为{}数量为{}个".format(
        room_plants['Room_1'],
        room_plants['Room_2'],
        room_plants['Room_3'],
        # room_plants['Room_4'],
        fruit_type_map.get(max_fruit, '未知果实'),
        max_fruit_count
    )

    return result_string


def mp3_modify():

    result_file_path = '/home/ucar/catkin_test_ws/src/image/result.txt'
    # 定义植物和水果的映射关系
    plant_type_map = {
        'plant_cucumber': '黄瓜植株',
        'plant_corn': '玉米植株',
        'plant_wheat': '小麦植株',
        'plant_rice': '水稻植株',
    }
    fruit_type_map = {
        'fruit_corn': '玉米果实',
        'fruit_watermelon': '西瓜果实',
        'fruit_cucumber': '黄瓜果实',
    }

    # 初始化统计结果的字典
    room_plants = {}
    fruit_counts = {}

    # 读取result.txt文件并进行统计
    with open(result_file_path, 'r') as file:
        lines = file.readlines()

    # 处理每一行的内容，统计植物类型
    for i, line in enumerate(lines):
        # 判断当前行是否为空，如果为空，则随机选择一个未知植株
        if not line.strip():
            room_plants[f'Room_{i+1}'] = '未知植株'
        else:
            plant_types = line.strip().split(' ')
            plant_counts = {plant_type: plant_types.count(plant_type) for plant_type in plant_types}
            max_plant = max(plant_counts, key=plant_counts.get)
            room_plants[f'Room_{i+1}'] = plant_type_map.get(max_plant, '未知植株')

    # 统计水果类型
    fruits = lines[3].strip().split(' ')
    for fruit in fruits:
        fruit_counts[fruit] = fruit_counts.get(fruit, 0) + 1

    # 找出最多的水果种类和数量
    max_fruit = max(fruit_counts, key=fruit_counts.get)
    max_fruit_count = fruit_counts[max_fruit]

    # 随机选择一个没有出现过的植株作为未知果实的替代
    used_plants = set(room_plants.values())
    unknown_plants = list(set(plant_type_map.values()) - used_plants)
    for i, room in enumerate(room_plants):
        if room_plants[room] == '未知植株':
            if unknown_plants:
                room_plants[room] = random.choice(unknown_plants)
                unknown_plants.remove(room_plants[room])

    # 构造输出字符串                ，D区域种植的作物为{}
    result_string = "任务完成E区域种植的作物为{}D区域种植的作物为{}E区域种植的作物为{}F区域存放的果实为{}数量为{}个".format(
        room_plants['Room_1'],
        room_plants['Room_2'],
        room_plants['Room_3'],
        # room_plants['Room_4'],
        fruit_type_map.get(max_fruit, '未知果实'),
        max_fruit_count
    )

    # 写一个列表
    mp3_results = [
        [['任务完成.mp3'], 0], 
        [['B_小麦.mp3', 'B_水稻.mp3', 'B_玉米.mp3', 'B_黄瓜.mp3'], 0], 
        [['C_小麦.mp3', 'C_水稻.mp3', 'C_玉米.mp3', 'C_黄瓜.mp3'], 0], 
        [['D_小麦.mp3', 'D_水稻.mp3', 'D_玉米.mp3', 'D_黄瓜.mp3'], 0], 
        [['E_小麦.mp3', 'E_水稻.mp3', 'E_玉米.mp3', 'E_黄瓜.mp3'], 0], 
        [['F_玉米.mp3', 'F_西瓜.mp3', 'F_黄瓜.mp3'], 0], 
        [['数量为1个.mp3', '数量为2个.mp3', '数量为3个.mp3', '数量为4个.mp3', '数量为5个.mp3'], 0], 
    ]

    plant_mapping = {
        "小麦植株": 0,
        "水稻植株": 1,
        "玉米植株": 2,
        "黄瓜植株": 3,
    }

    fruit_mapping = {
        "玉米果实": 0,
        "西瓜果实": 1,
        "黄瓜果实": 2,
    }

    number_mapping = {
        1: 0,
        2: 1,
        3: 2,
        4: 3,
        5: 4,
    }
    
    # 修改结果的示例
    mp3_results[0][1] = 0   # 任务完成（无需修改）
    mp3_results[1][1] = plant_mapping[room_plants['Room_1']]   # B
    mp3_results[2][1] = plant_mapping[room_plants['Room_2']]   # C
    mp3_results[3][1] = plant_mapping[room_plants['Room_3']]   # D
    mp3_results[4][1] = plant_mapping[room_plants['Room_4']]   # E
    mp3_results[5][1] = fruit_mapping[fruit_type_map.get(max_fruit, '未知果实')]   # F
    mp3_results[6][1] = number_mapping[max_fruit_count]   # 数量，注意，0 号文件是 1 个，1 号 文件是 2 个，如果怕错可以加个字典映射

    # 7 次播放
    for result in mp3_results:
        audios = result[0]
        number = result[1]
        if number == plant_mapping[room_plants['Room_3']]:   # 需要根据情况进行修改
            continue
        os.system(f'ffplay -nodisp -autoexit /home/ucar/Music/mp3/{audios[number]}')

    return result_string


class liu_control:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.yolo_start_pub = rospy.Publisher("/yolo_start_flag", Int8, queue_size=1)
        self.yolo_over_sub = rospy.Subscriber("/yolo_over_flag", Int8 , self.callback_yolo_over)
        self.tts_pub = rospy.Publisher('/ucar_voice_topic', String, queue_size=10)
        self.liu_star_sub = rospy.Subscriber("/liu_ucar_star", Int8, self.callback_liu_star)

        self.cap = cv2.VideoCapture("/dev/video0")
        ret, frame = self.cap.read()
        self.yolo_over_flag = 0
        self.liu_start_flag = 0

    def save_img(self, index, num, num_wait):
        for i in range(num_wait):
            ret, frame = self.cap.read()
            frame = cv2.flip(frame,1)   ##图像左右颠倒
        cv2.imwrite("/home/ucar/catkin_test_ws/src/image/" + str(index) + "_" + str(num) + ".jpg", frame)
    
    def cap_release(self):
        self.cap.release()

    def yolo_start(self):
        self.yolo_start_pub.publish(1)

    def callback_liu_star(self, msg):
        self.liu_start_flag = msg.data

    def callback_yolo_over(self, msg):
        self.yolo_over_flag = msg.data

    def report_result(self,msg):
        self.tts_pub.publish(msg)

    def get_liu_start_flag(self):
        return self.liu_start_flag

if __name__ == '__main__':
    try:
        rospy.init_node("liu_contorl")
        rospy.loginfo("开始导航")
        # 创建MoveBaseAction client
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # 等待MoveBaseAction启动        
        client.wait_for_server()
        liu = liu_control()
        while not liu.get_liu_start_flag():  # 等待liu_start_flag为1
            rospy.loginfo("等待语音启动，%d", liu.get_liu_start_flag())
            time.sleep(0.1)  # 为了不占用过多CPU资源，加入短暂延时
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

                # elif waypoint_name in ['C1', 'C2', 'C3', 'C4']:
                #     liu.save_img(3, coun3, 10)
                #     coun3 += 1
                
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
                # while True:
                #     rospy.loginfo("回家等待yolo识别结果，在while循环内")
                #     if liu.yolo_over_flag:
                #         rospy.loginfo("开始等待yolo执行结束")
                #         time.sleep(0.1)
                #         break
                # rospy.loginfo("开始分析数据")        
                # tts_result = mp3_modify()
                # rospy.loginfo(tts_result)
                # # liu.report_result(tts_result)
            liu.cap_release()
            break

        while True:
            rospy.loginfo("回家等待yolo识别结果，在while循环内")
            if liu.yolo_over_flag:
                rospy.loginfo("开始等待yolo执行结束")
                time.sleep(0.1)
                break
        rospy.loginfo("开始分析数据")        
        tts_result = mp3_modify()
        rospy.loginfo(tts_result)
        # liu.report_result(tts_result)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Liu interrupted.")
    finally:
        liu.cap_release()
        liu.cmd_vel_pub.publish(Twist())
        
