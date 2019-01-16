#!/usr/bin/env python

import sys
import rospy
import moveit_commander

from UR5Controller import UR5Controller

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('print_state_node', anonymous=True)

arm_controller = UR5Controller()
try:
    arm_controller.pick_and_place()
    print(arm_controller.get_current_pose())

except:
    arm_controller.stop()

moveit_commander.roscpp_shutdown()
