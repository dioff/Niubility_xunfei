# 设备操作手册 v1.0

## 一、安装依赖

- 利用startup_scripts进行文件创建与udev服务重启，以及在环境.bashrc中添加了ROS的环境source

  ```bash
  cd ./ucar_ws/src/startup_scripts/
  sudo chmod 777 ./initdev*.sh
  sudo ./initdev_mini.sh # 对应晓mini版本，晓版本勿执行这句
  sudo reboot # 可选，最好完成一次重启
  ```

- 安装下位机与ROS通讯依赖

```bash
    sudo apt update
    sudo apt install ros-melodic-serial libeigen3-dev
    sudo pip3 uninstall em
    sudo pip3 install empy -i https://pypi.tuna.tsinghua.edu.cn/simple	
```

- 安装Gmapping建图依赖

```bash
sudo apt install ros-melodic-gmapping		
```

- 安装导航所有依赖

```bash
sudo apt-get install ros-melodic-navigation*
```

- 安装imu所需依赖

```bash
sudo apt install ros-melodic-serial
```

- 安装Teb所需依赖

```bash
sudo apt install ros-melodic-teb-local-planner
```



## 二、编译

```bash
cd ~/catkin_test_ws
catkin_make
```



## 三、运行

```bash
roslaunch ucar_nav liu_nav_test.launch   # 启动底盘导航
roslaunch xf_mic_asr_offline voice_control.launch  # 语音启动程序
rosrun ucar_yolo darknet_ucar.py   # 图像识别程序
rosrun send_goals send_goals.py  # 启动总调度
```

语音唤醒词为小易同学，在唤醒完以后说：”兄弟出发“即可完成比赛所有内容