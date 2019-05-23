# VRUI_RVIZ - Virtual reality user interface in RVIZ using Leap Motion

how to install

how to use

```
sudo leapd 
```
roslaunch vrui_rviz <käivitusskripti nimi>.launch, kus <käivitusskripti nimi> tuleb asendada sobiva käivitusskriptiga, mis on leitavad tabelis 5.2
```
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=ROBOT_IP, kus ROBOT_IP tuleb asendada kasutatava roboti IP-aadressiga 
```
roslaunch ur5_custom_config ur5_on_table_moveit_planning_execution.launch
```
