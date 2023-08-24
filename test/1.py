import random
import os
import numpy as np

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
        if line.strip() == 'nan':
            room_plants[f'Room_{i+1}'] = '未知植株'
        else:
            # plant_types = line.strip().split(' ')
            # plant_counts = {plant_type: plant_types.count(plant_type) for plant_type in plant_types}
            # max_plant = max(plant_counts, key=plant_counts.get)
            # room_plants[f'Room_{i+1}'] = plant_type_map.get(max_plant, '未知植株')
            plant_types = [plant for plant in line.strip().split(' ') if plant.startswith('plant_')]
            if plant_types:  # 如果存在植物类型，则选择出现次数最多的植物作为当前房间的植物
                plant_counts = {plant_type: plant_types.count(plant_type) for plant_type in plant_types}
                max_plant = max(plant_counts, key=plant_counts.get)
                room_plants[f'Room_{i+1}'] = plant_type_map.get(max_plant, '未知植株')
            else:  # 否则，随机选择一个未知植株
                room_plants[f'Room_{i+1}'] = '未知植株'

    # # 统计水果类型
    # fruits = lines[3].strip().split(' ')
    # fruit_counts = {fruit: fruits.count(fruit) for fruit in set(fruits)}
    # # 判断是否有未知果实
    # if 'nan' in fruit_counts:
    #     nan_count = fruit_counts['nan']
    #     del fruit_counts['nan']
    #     # 检查是否全部为nan，如果是，则未知果实为nan，否则根据水果数量来判断
    #     if nan_count == len(fruits):
    #         unknown_fruit = 'nan'
    #     else:
    #         if 'fruit_corn' in fruit_counts and 'fruit_watermelon' in fruit_counts:
    #             unknown_fruit = 'fruit_corn'
    #         elif 'fruit_cucumber' in fruit_counts and 'fruit_watermelon' in fruit_counts:
    #             unknown_fruit = 'fruit_cucumber'
    # # 找出最多的水果种类和数量
    # max_fruit = max(fruit_counts, key=fruit_counts.get)
    # max_fruit_count = fruit_counts[max_fruit]
# 统计水果类型
    fruits = lines[3].strip().split(' ')
    fruit_counts = {fruit: fruits.count(fruit) for fruit in set(fruits)}
    # 判断是否有未知果实
    if 'nan' in fruit_counts:
        nan_count = fruit_counts['nan']
        del fruit_counts['nan']
        # 检查是否全部为nan，如果是，则未知果实为nan，否则根据水果数量来判断
        if nan_count == len(fruits):
            unknown_fruit = 'nan'
        else:
            max_fruit_count = max(fruit_counts.values())
            max_fruits = [fruit for fruit, count in fruit_counts.items() if count == max_fruit_count]

            if len(max_fruits) == 1:
                unknown_fruit = max_fruits[0]
            else:
                if 'fruit_watermelon' in max_fruits and 'fruit_cucumber' in max_fruits:
                    unknown_fruit = 'fruit_cucumber'
                    fruit_counts['fruit_cucumber'] += random.randint(1, 3)
                elif 'fruit_watermelon' in max_fruits and 'fruit_corn' in max_fruits:
                    unknown_fruit = 'fruit_corn'
                    fruit_counts['fruit_corn'] += random.randint(1, 2)
                elif 'fruit_cucumber' in max_fruits and 'fruit_corn' in max_fruits:
                    unknown_fruit = 'fruit_cucumber'
                    fruit_counts['fruit_cucumber'] += 1



    # 随机选择一个没有出现过的植株作为未知植株的替代
    used_plants = set(room_plants.values())
    unknown_plants = list(set(plant_type_map.values()) - used_plants)
    for i, room in enumerate(room_plants):
        if room_plants[room] == '未知植株':
            if unknown_plants:
                room_plants[room] = random.choice(unknown_plants)
                unknown_plants.remove(room_plants[room])
    
    # 处理未知果实的情况
    if unknown_fruit:
        for i, room in enumerate(room_plants):
            if room_plants[room] == '未知植株':
                room_plants[room] = unknown_fruit

    # # 构造输出字符串                ，D区域种植的作物为{}
    # result_string = "任务完成E区域种植的作物为{}C区域种植的作物为{}B区域种植的作物为{}F区域存放的果实为{}数量为{}个".format(
    #     room_plants['Room_1'],
    #     room_plants['Room_2'],
    #     room_plants['Room_3'],
    #     # room_plants['Room_4'],
    #     fruit_type_map.get(max_fruit, '未知果实'),
    #     max_fruit_count
    # )
    # 构造输出字符串
    result_string = "任务完成E区域种植的作物为{}C区域种植的作物为{}B区域种植的作物为{}F区域存放的果实为{}数量为{}个".format(
        room_plants['Room_1'],
        room_plants['Room_2'],
        room_plants['Room_3'],
        fruit_type_map.get(unknown_fruit, fruit_type_map[max(fruit_counts, key=fruit_counts.get)]),
        fruit_counts.get(unknown_fruit, max(fruit_counts.values()))
    )
    # return result_string



    # # 写一个列表
    mp3_results = [
        [['任务完成.mp3'], 0], 
        [['E_小麦.mp3', 'E_水稻.mp3', 'E_玉米.mp3', 'E_黄瓜.mp3'], 0],
        [['D_小麦.mp3', 'D_水稻.mp3', 'D_玉米.mp3', 'D_黄瓜.mp3'], 0], 
        [['C_小麦.mp3', 'C_水稻.mp3', 'C_玉米.mp3', 'C_黄瓜.mp3'], 0],
        [['B_小麦.mp3', 'B_水稻.mp3', 'B_玉米.mp3', 'B_黄瓜.mp3'], 0], 
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
        "fruit_corn": 0,
        "fruit_watermelon": 1,
        "黄瓜果实": 2,
    }

    number_mapping = {
        1: 0,
        2: 1,
        3: 2,
        4: 3,
        5: 4,
    }
    
    # # 修改结果的示例

    mp3_results[0][1] = 0   # 任务完成（无需修改）
    mp3_results[1][1] = plant_mapping[room_plants['Room_1']]   # E
    mp3_results[2][1] = plant_mapping[room_plants['Room_4']]   # D
    mp3_results[3][1] = plant_mapping[room_plants['Room_2']]   # C
    mp3_results[4][1] = plant_mapping[room_plants['Room_3']]   # B
    mp3_results[5][1] = fruit_mapping[fruit_type_map.get(unknown_fruit, fruit_type_map[max(fruit_counts, key=fruit_counts.get)])]   # F
    mp3_results[6][1] = number_mapping[fruit_counts.get(unknown_fruit, max(fruit_counts.values()))]   # 数量，注意，0 号文件是 1 个，1 号 文件是 2 个，如果怕错可以加个字典映射
    
    # # 7 次播放
    for result in mp3_results:
        audios = result[0]
        number = result[1]
        if number == plant_mapping[room_plants['Room_4']]:   # 需要根据情况进行修改
            continue
        os.system(f'ffplay -nodisp -autoexit /home/ucar/Music/mp3/{audios[number]}')

    return result_string    



print(mp3_modify())
mp3_modify()
