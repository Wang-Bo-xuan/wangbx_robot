# wangbx_robot 
## 一. 项目介绍  
该项目是基于Ubuntu 16.04 LTS/ROS kinetic实现的自主移动机器人算法仿真平台，将覆盖全自主移动底盘，多关节机械臂等机器人平台相关功能。该平台紧依赖于Gazebo、SLAM框架及MoveIT框架，最好同时配有qt、opencv2/3及pcl、eigen等。  
## 二. 使用说明  
1. 启用仿真器  
> roslaunch wangbx_robot_config gazebo.launch  
2. 启动建图  
> roslaunch wangbx_robot_config mapping.launch  
3. 启动遥控(二选一,其中qt_teleop为GUI版本，为触控屏设计)  
> roslaunch wangbx_robot_config teleop.launch  
> roslaunch wangbx_robot_config qt_teleop.launch
4. 其他功能扔处于开发阶段，包括但不限于栅格地图构建、粒子滤波定位等  
## 三. 其他  
本项目完全开源，欢迎各位道友加入并贡献自己的一份力量，有交流需求欢迎联系  
Tel:13715047518  
Wechat:Wang080207030511  
QQ:779139218  
