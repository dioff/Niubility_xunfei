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

    # 构造输出字符串                ，D区域种植的作物为{}
    result_string = "任务完成E区域种植的作物为{}C区域种植的作物为{}B区域种植的作物为{}F区域存放的果实为{}数量为{}个".format(
        room_plants['Room_1'],
        room_plants['Room_2'],
        room_plants['Room_3'],
        # room_plants['Room_4'],
        fruit_type_map.get(max_fruit, '未知果实'),
        max_fruit_count
    )

    return result_string