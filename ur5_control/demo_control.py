#!/usr/bin/env python

import sys
import rospy
import moveit_commander

from UR5Controller import UR5Controller
from CameraController import CameraController

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

arm_controller = UR5Controller()
# arm_controller.move_joints(arm_controller.joint_poses['frontal'], plan_only=False)

# camera_controller = CameraController('stiff/01/')
try:
    # camera_controller.capture_and_display()

    k = 1
    for i, j in arm_controller.grid_tracer(plan_only=False):
        # camera_controller.capture_display_and_save(k)
        k += 1

    arm_controller.stop()
    # camera_controller.release()
except:
    arm_controller.stop()
    # camera_controller.release()

moveit_commander.roscpp_shutdown()
