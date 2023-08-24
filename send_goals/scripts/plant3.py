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
    'F0': (0.94, -2.78, -0.70, 0.71),
    'F1': (0.39, -3.32, -0.45, 0.89),
    'F4': (1.26, -3.66, -0.93, 0.36),

    # 'F-LU': (1.78, -4.95, -0.66, 0.74),
    'F-LL': (1.36, -5.35, -0.07, 1.00),
    # 'F-RU': (0.07, -5.03, -0.70, 0.71), 
    # 'F-RR': (0.28, -5.40, -1.00, 0.03), 
    'F3': (0.92, -2.30, 0.91, 0.40),  
    'F2': (0.92, -2.30, 0.33, 0.95),
    'uphill-2': (0.92, -2.30, 0.71, 0.70),
    'downhill': (0.93, -0.36, 0.71, 0.70),
    # 'turn': (0.94, -0.36, 0.00, 1.00),

    # 'E1': (2.83, -1.11, 0.22, 0.97),
    'E2': (3.17, -1.16, -0.97, 0.24),
    'E3': (3.12, -1.25, 0.95, 0.33),

    'D1': (2.77, -3.79, -0.97, 0.23),
    'D2': (2.76, -3.81, -0.12, 0.99),

    'C1': (4.66, -1.24, -0.90, 0.44),
    'C2': (4.87, -1.25, 0.97, 0.23),

    'B1': (5.01, -1.52, 0.99, 0.12),
    'B2': (5.01, -1.52, -0.96, 0.30),
    'B4': (5.01, -1.52, -0.96, 0.30),

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

# 二维 txt 转二级列表
def file_to_list(file_name):
    with open(file_name, 'r') as file:
        lines = file.readlines()
    # 创建二级列表
    data = []
    for line in lines:
        # 分割每一行，并将其添加到列表中
        data.append(line.split())
    return data

# 列表解析，统计列表中每个元素的数量，返回格式是字典
from collections import Counter
def count_elements(lst):
    return dict(Counter(lst))

# 将列表中只有 'nan' 键的字典，替换为示例中没出现过的，随机在 5 种键中抽一个，字典元素的数量设为 1
def replace_nan(list_of_dicts, keys):
    # 将 keys 转换为集合以方便操作
    keys = set(keys)
    # 创建一个 set 来存储已经在 list_of_dicts 中出现过的键
    used_keys = set()
    for dictionary in list_of_dicts:
        used_keys.update(dictionary.keys())
    # 可用键是 keys 中没有在 used_keys 中出现过的键
    available_keys = keys - used_keys
    # 将 available_keys 转换为列表以方便随机选择
    available_keys = list(available_keys)
    for dictionary in list_of_dicts:
        if list(dictionary.keys()) == ['nan']:
            if available_keys:  # 如果还有可用的键
                new_key = random.choice(available_keys)
                available_keys.remove(new_key)
                dictionary[new_key] = dictionary.pop('nan')
    return list_of_dicts

def replace_nan_in_dicts(dict_list, keys):
    for i, d in enumerate(dict_list):
        # 如果字典只有一个键且这个键为'nan'
        if list(d.keys()) == ['nan']:
            # 从没有出现过的键中随机选取一个
            not_in_dict_keys = [k for k in keys if k not in [list(dic.keys())[0] for dic in dict_list]]
            if not_in_dict_keys:  # 判断是否有未出现过的键
                new_key = random.choice(not_in_dict_keys)
                # 替换字典
                dict_list[i] = {new_key: 1}
    return dict_list

# 如果字典中只有 'nan' 键的元素，就删除这个元素，并替换为 keys 列表中随便一个元素为新的键，键的值为1，然后return新的字典
def replace_nan_in_dicts_fruit(dictionary, keys):
    if len(dictionary) == 1 and 'nan' in dictionary:
        # 从 keys 列表中随机选择一个键
        new_key = random.choice(keys)
        # 用新的键和值替换字典中的 'nan' 键
        dictionary[new_key] = 1
        del dictionary['nan']
    return dictionary

# 将列表中元素有 'nan' 的字典，删除掉 'nan' 元素
def remove_nan(list_of_dicts):
    for dictionary in list_of_dicts:
        # 如果字典中包含 'nan'，则删除它
        if 'nan' in dictionary:
            del dictionary['nan']
    return list_of_dicts

# 将列表中元素数量大于1的字典，只保留元素的值最大的
def keep_max_value(list_of_dicts):
    for idx, dictionary in enumerate(list_of_dicts):
        # 如果字典中的元素数量大于1，我们只保留值最大的元素
        if len(dictionary) > 1:
            max_key = max(dictionary, key=dictionary.get)  # 找到值最大的键
            max_value = dictionary[max_key]  # 获取最大的值
            list_of_dicts[idx] = {max_key: max_value}  # 创建一个只包含最大值的新字典
    return list_of_dicts

def remove_nan_fruit(dictionary):
    # 如果字典中包含 'nan'，则删除它
    if 'nan' in dictionary:
        del dictionary['nan']
    return dictionary

# 统计列表中每个字典的键总共有多少种
def dict_to_list(list_of_dicts):
    ls = []
    for dictionary in list_of_dicts:
        for key, value in dictionary.items():
            ls.append(key)
    return ls

def get_keys(dictionary):
    return list(dictionary.keys())

def get_max_value_key(input_dict):
    max_key = max(input_dict, key=input_dict.get)
    max_value = input_dict[max_key]
    return max_key, max_value

def modify_values(dictionary):
    # 如果字典中存在 'fruit_watermelon' 和 'fruit_cucumber'，并且他们的值相等
    if 'fruit_watermelon' in dictionary and 'fruit_cucumber' in dictionary and dictionary['fruit_watermelon'] == dictionary['fruit_cucumber']:
        # 在 'fruit_watermelon' 的值上随机加 1 或 2
        dictionary['fruit_cucumber'] += random.randint(1, 3)
    # 如果修改后的 'fruit_watermelon' 和 'fruit_cucumber' 的值依然相等
    if 'fruit_watermelon' in dictionary and 'fruit_corn' in dictionary and dictionary['fruit_watermelon'] == dictionary['fruit_corn']:
        # 在 'fruit_cucumber' 的值上随机加 1、2 或 3
        dictionary['fruit_corn'] += random.randint(1, 2)
    # 如果修改后的 'fruit_watermelon' 和 'fruit_cucumber' 的值依然相等
    if 'fruit_corn' in dictionary and 'fruit_cucumber' in dictionary and dictionary['fruit_corn'] == dictionary['fruit_cucumber']:
        # 在 'fruit_cucumber' 的值上加 1
        dictionary['fruit_cucumber'] += 1
    return dictionary

# 输入是由字典组成的列表，如果列表中有空字典，就在 keys 列表中随机选一个键放进去，值为1，最后返回列表
def empty_plant(dict_list, keys):
    for i, dictionary in enumerate(dict_list):
        if not dictionary:  # 检查字典是否为空
            random_key = random.choice(keys)  # 随机选择一个键
            dict_list[i] = {random_key: 1}  # 将随机选择的键和值1添加到空字典中
    return dict_list

# 如果输入的字典是空字典，就在 keys 列表中随机选一个键放进去，值为1，最后返回字典
def empty_fruit(input_dict, keys):
    if not input_dict:  # 检查字典是否为空
        random_key = random.choice(keys)  # 从 keys 列表中随机选择一个键
        input_dict[random_key] = 3  # 将随机选择的键和值 1 添加到字典中
    return input_dict

# 判断
def determine(matrix):

    # 植株，总共有 5 种：'nan', 'plant_corn', 'plant_cucumber', 'plant_rice', 'plant_wheat'
    # 第一：字典只有 nan 的随便猜一个
    # 第二：有 nan 有其他的，删掉 nan
    # 第三：有多个植株的，保留最大的
    # 第四：空行判断
    matrix_plants = matrix[0:4]     # 注意，切片是包含左边不包含右边
    keys = ['plant_corn', 'plant_cucumber', 'plant_rice', 'plant_wheat']
    matrix_plants = replace_nan_in_dicts(matrix_plants, keys)
    matrix_plants = remove_nan(matrix_plants)
    matrix_plants = keep_max_value(matrix_plants)
    matrix_plants = empty_plant(matrix_plants, keys)

    # 果实，总共有 4 种，'nan', 'fruit_corn', 'fruit_cucumber', 'fruit_watermelon'
    # 第一：字典只有 nan 的随便猜一个
    # 第二：删除 nan
    # 第三：最大判断
    # 西瓜和玉米数量相同，玉米+1-2
    # 西瓜和黄瓜数量相同，黄瓜+1-3
    # 玉米和黄瓜数量相同，黄瓜+1
    matrix_frult = matrix[4]
    keys_fruit = ['fruit_corn', 'fruit_cucumber', 'fruit_watermelon']
    matrix_frult = replace_nan_in_dicts_fruit(matrix_frult, keys_fruit)
    matrix_frult = remove_nan_fruit(matrix_frult)
    matrix_frult = modify_values(matrix_frult)
    matrix_frult = empty_fruit(matrix_frult, keys_fruit)

    # 输出最后的组合
    matrix_plants_frult = dict_to_list(matrix_plants)
    key, value = get_max_value_key(matrix_frult)
    matrix_plants_frult.append(key)
    matrix_plants_frult.append(value)

    return matrix_plants_frult

# 按字典替换列表
def replace_by_dict(input_list, replacement_dict):
    return list(map(replacement_dict.get, input_list))

def list_rename(lst):
    replacement_dict = {
        'plant_corn': '玉米',
        'plant_cucumber': '黄瓜',
        'plant_rice': '水稻',
        'plant_wheat': '小麦'
    }
    lst = replace_by_dict(lst, replacement_dict)

    # 重点！！！！！！！！！！！！！！！！！！！！！！！！！！
    # name_areas 是区域的顺序，会按顺序播报，要修改就改这个，其他一点不要动
    name_areas = ['E_', 'D_', 'C_', 'B_']
    lst = [str(a) + str(b) for a, b in zip(name_areas, lst)]

    return lst

def element_rename(key):
    replacement_dict = {
        'fruit_corn': '玉米',
        'fruit_cucumber': '黄瓜',
        'fruit_watermelon': '西瓜',
    }
    return "F_" + str(replacement_dict[key])

def element_rename_number(key):
    replacement_dict = {
        1: '数量为1个',
        2: '数量为2个',
        3: '数量为3个',
        4: '数量为4个',
        5: '数量为5个',
        6: '数量为6个',
        7: '数量为7个',
        8: '数量为8个',
        9: '数量为9个',
    }
    return str(replacement_dict[key])

def mp3_modify(lst):    
    # 正式播报
    for i in lst:
        os.system(f'ffplay -nodisp -autoexit /home/ucar/Music/mp3/{i}.mp3')
        # os.system(f'ffplay -nodisp -autoexit mp3/{i}.mp3')

def player():
    # 读取 result
    result_file_path = '/home/ucar/catkin_test_ws/src/image/result.txt'
    # result_file_path = r'C:\Users\Dao\Works\泰州学院\比赛与项目\讯飞机器人比赛\代码\语音\$最新\test\result.txt'
    result_list = file_to_list(result_file_path)
    
    # 解析 result
    result_count = []
    for ls in result_list:
        result_count.append(count_elements(ls))
    
    # 判断 result
    result_determine = determine(result_count)

    # 重命名
    result_mp3_plant = list_rename(result_determine[0:4])
    result_mp3_frult = element_rename(result_determine[4])
    result_mp3_number = element_rename_number(result_determine[5])

    # 合成最终列表
    result_mp3 = ['任务完成']
    result_mp3 = result_mp3 + result_mp3_plant
    result_mp3.append(result_mp3_frult)
    result_mp3.append(result_mp3_number)

    # 运行测试
    mp3_modify(result_mp3)
    print(result_mp3)

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
                if waypoint_name in ['E1', 'E2', 'E3']:
                    liu.save_img(1, coun1, 10)
                    coun1 += 1

                elif waypoint_name in ['D1', 'D2', 'D3']:
                    liu.save_img(2, coun2, 10)
                    coun2 += 1

                elif waypoint_name in ['C1', 'C2']:
                    liu.save_img(3, coun3, 10)
                    coun3 += 1
                
                elif waypoint_name in ['B1', 'B2', 'B3']:
                    liu.save_img(4, coun4, 10)
                    coun4 += 1
                
                elif waypoint_name in ['F1', 'F-LU', 'F-LL', 'F-RU', 'F-RR', 'F2', 'F3', 'F4']:
                    liu.save_img(5, coun5, 10)
                    coun5 += 1

                elif waypoint_name in ['B4']:
                    liu.yolo_start()
                    rospy.loginfo("回家等待yolo识别结果")

            liu.cap_release()
            break

        while True:
            rospy.loginfo("回家等待yolo识别结果，在while循环内")
            if liu.yolo_over_flag:
                rospy.loginfo("开始等待yolo执行结束")
                time.sleep(0.1)
                break
        player()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Liu interrupted.")
    finally:
        liu.cap_release()
        liu.cmd_vel_pub.publish(Twist())
        
