# ur_force_control
use UR5 and optoforce hex-e to realize admittance control
 


# package description
* *etherdap_ros* is the ros driver package of optoforce sensor

* *force_control* is the package to perform force control. Now hybird position/force control and admittance control is tested.

* *gravity_compensate* is the package to perform gravity_compensate. Use least squares to identify parameters of end effector and eliminate the influence of end effector because of gravity

* *ur_control_test* is the package of test someting like yaml_file write or poe method to get robot pose

* *ur_modern_driver* is the ros driver package of ur robot

* *ur5_table* and *ur5_table_moveit* is the config of ur_moveit package

* *wrench_filter* is the packgae to perform sensor data processing,mainly use mean filter to lower the noise

# Usage
* mkdir -p catkin_ws/src

* git clone this repository to src

* catkin_make

* open a terminal and run `roslaunch wrench_filter wrench.launch`

* open a terminal and run  `roslaunch ur5_table_moveit ur5_moveit_planning_execution.launch`
* open a termianl and run `rosrun gravity_compensate gravity_identify_node`(if you want to recalibrate the end_effector)
* open a terminal and run  `roslaunch gravity_compensate  gravity_compensate.launch`

* open a terminal and run `rosrun force_control admittance_control_node`


# Development environment
ubuntu 16.04

# Reference
https://zhuanlan.zhihu.com/p/150587985
 

