#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from math import pi

from UR5Controller import UR5Controller
from CameraController import CameraController
import time

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pick_and_place_demo', anonymous=True)

arm_controller = UR5Controller()
# arm_controller.move_joints(arm_controller.joint_poses['frontal'], plan_only=False)

camera_controller = CameraController('stiff/01/')
try:
    camera_controller.capture_and_display()

    k = 1
    pa = ((0.05, 0.5, 0.3), (1 * pi / 4, pi / 2, 0))
    pb = ((0.4, 0.15, 0.3), (3 * pi / 4, pi / 2, 0))
    m = ((0.3, 0.4, 0.6), (2 * pi / 4, pi / 2, 0))

    while(not camera_controller.detect_touch()):
        time.sleep(0.1)

    print('end!')



    # for i, j in arm_controller.pick_and_place(pa, pb, m, plan_only=False):
        # camera_controller.capture_display_and_save(k)
        # k += 1
    # arm_controller.stop()
    # camera_controller.release()
except:
    arm_controller.stop()
    # camera_controller.release()

moveit_commander.roscpp_shutdown()
