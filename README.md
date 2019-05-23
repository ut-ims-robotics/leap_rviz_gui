# VRUI_RVIZ - Virtual reality user interface in RVIZ using Leap Motion, OSVR and UR5

Vrui_rviz is a virutal reality user interface in RViz that uses Leap Motion (LM) contrller, OSVR headset and UR5 robot to provide a way to have hands-free control of robot in virtual reality

## Installing

## Using

1.```sudo leapd ```

2.```roslaunch vrui_rviz <launch file>.launch```, where <launch file> stands for launch file with certain configuraton

3.```roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=ROBOT_IP ```, where ROBOT_IP stands for robot's IP address

4.```roslaunch ur5_custom_config ur5_on_table_moveit_planning_execution.launch```


## Table of launch files

| Launch file  | Configuration |
| ------------- | ------------- |
| cam_without_osvr.launch  | controlling robot with LM and webcam, meant for testing  |
| leap_cam_osvr.launch  | contrilling physical robot in RViz using UR5, LM, webcam and OSVR  |
| leap_osvr.launch  | contrilling physical robot in RViz using UR5, LM and OSVR  |
| robot_without_osvr.launch  | contrilling physical robot in RViz using UR5 and LM  |
| without_osvr.launch  | controlling robot model in RViz using LM  |



