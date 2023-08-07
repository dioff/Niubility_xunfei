import random

def analyze_result():


    result_file_path = '/home/ucar/catkin_test_ws/src/test/result.txt'
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
    result_string = "任务完成B区域种植的作物为{}, C区域种植的作物为{}, E区域种植的作物为{}, F区域存放的果实为{}, 数量为{}个".format(
        room_plants['Room_1'],
        room_plants['Room_2'],
        room_plants['Room_3'],
        # room_plants['Room_4'],
        fruit_type_map.get(max_fruit, '未知果实'),
        max_fruit_count
    )

    return result_string

print(analyze_result())