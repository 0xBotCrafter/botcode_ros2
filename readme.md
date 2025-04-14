# 【Bot手册-ROS2】源码(点亮Star不迷路~)
## 1 介绍
本仓库是【Bot手册-ROS2】的学习源码仓库，用于ROS2学习时的参考和代码复用拷贝。

【Bot手册-ROS2】链接：https://zhuanlan.zhihu.com/p/1895206170618230592

## 2 使用方法
### 2.1 克隆仓库
```shell
git clone https://github.com/0xBotCrafter/botcode_ros2.git
```
### 2.2 安装依赖
```shell
sudo apt update && sudo apt upgrade

sudo apt install ros-$ROS_DISTRO-tf-transformations
sudo apt install ros-$ROS_DISTRO-joint-state-publisher
sudo apt install ros-$ROS_DISTRO-robot-state-publisher
sudo apt install ros-$ROS_DISTRO-xacro
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs
sudo apt install ros-$ROS_DISTRO-gazebo-ros2-control
sudo apt install ros-$ROS_DISTRO-navigation2
sudo apt install ros-$ROS_DISTRO-nav2-bringup
sudo apt install ros-$ROS_DISTRO-pluginlib -y
```
### 2.3 编译
```shell
cd ./botcode_ros2
colcon build
```

### 2.2 运行演示
```shell
source ./install/setup.bash
ros2 launch chapt6_rob_descrip gazebo_sim.launch.py
ros2 launch chapt7_nav2_cpp navigation2.launch.py
ros2 launch chapt7_nav2_py autopatrol.launch.py
```
## 3 作者
0xBotCrafter: zhouzge@foxmail.com