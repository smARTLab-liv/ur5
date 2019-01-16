#### Dependencies

clone to rospackages
```

```
git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git
git clone -b kinetic-devel https://github.com/ros-industrial/robotiq.git
# remove gazebo packages
# rosdep install robotiq_modbus_tcp

```


```
pip install pathlib
pip install matplotlib

``` 

xhost +local:docker
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.100
roslaunch ur5_smartlab_moveit_config ur5_smartlab_planning_execution.launch
roslaunch ur5_smartlab_gazebo ur5.launch
rosrun ur5_control demo_control.py

roslaunch moveit_setup_assistant setup_assistant.launch



gripper


rosdep install robotiq_modbus_tcp
sudo apt-get install ros-kinetic-soem



rosrun robotiq_3f_gripper_control Robotiq3FGripperTcpNode.py 192.168.1.11