


xhost +local:docker
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.100
roslaunch ur5_smartlab_moveit_config ur5_smartlab_planning_execution.launch
roslaunch ur5_smartlab_gazebo ur5.launch
rosrun ur5_control demo_control.py

roslaunch moveit_setup_assistant setup_assistant.launch
