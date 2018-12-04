#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from math import pi
import tf

# moveit_commander.roscpp_initialize(sys.argv)
# rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur5_control', anonymous=True)

robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander('manipulator')


# orientation = tf.transformations.quaternion_from_euler(pi, (pi), 0, 'sxyz')
#
# plan = group.plan([2.2,0,-1.57,0,0,0])
# group.execute(plan)
# print('done.')
# print(orientation)

# pose_goal.orientation.w = orientation[0]
# pose_goal.orientation.x = orientation[1]
# pose_goal.orientation.y = orientation[2]
# pose_goal.orientation.z = orientation[3]
#


def move(group, pose):
    group.set_pose_target(pose)
    plan = group.plan()

    if plan.joint_trajectory.header.frame_id != '':
        plan = group.go(wait=True)
        group.stop()
    else:
        print('Failed to get a plan.')

    group.clear_pose_targets()


joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
          'ee_fixed_joint']
ee_link = group.get_end_effector_link()
joint_goal = group.get_current_pose(ee_link)
joints = group.get_current_joint_values()
print('current pose: ', joint_goal.pose)
# print(-pi / 6)

# Set the reference frame for pose targets
reference_frame = "/base_link"

# Set the ur5_arm reference frame accordingly
group.set_pose_reference_frame(reference_frame)

# Allow replanning to increase the odds of a solution
group.allow_replanning(True)

# Allow some leeway in position (meters) and orientation (radians)
group.set_goal_position_tolerance(0.01)
group.set_goal_orientation_tolerance(0.1)
group.set_planning_time(0.1)
group.set_max_acceleration_scaling_factor(.5)
group.set_max_velocity_scaling_factor(.5)


c1 = moveit_msgs.msg.JointConstraint('shoulder_pan_joint', 0, pi/10, pi/10, 1)
c2 = moveit_msgs.msg.JointConstraint('shoulder_lift_joint', 0, pi/2, pi/10, 2)
c3 = moveit_msgs.msg.JointConstraint('elbow_joint', 0, 999, 2, 10)

constraints = moveit_msgs.msg.Constraints()
constraints.name = 'ur5_constraints'
constraints.joint_constraints.append(c1)
constraints.joint_constraints.append(c2)
constraints.joint_constraints.append(c3)


#
group.set_path_constraints(constraints)
print('n constraint: ', len(group.get_path_constraints().joint_constraints))
print('current joints: ', zip(joints, group.get_current_joint_values()))

pose_goal = geometry_msgs.msg.Pose()
# pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(1, 0, 0))
pose_goal.position = geometry_msgs.msg.Point(0.75, 0.19, 0.05)

move(group, pose_goal)

# move(group, pose_goal)
# print(group.get_interface_description())
# group.set_planner_id('xxxx')


# group.set_pose_target(pose_goal)
# plan1 = group.plan()
# print(dir(plan1))
# print(plan1)
# print(type(plan1))

# for y in range(0, 2):
#     print(0.1*y)
#     pose_goal = geometry_msgs.msg.Pose()
#     # pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, pi / 2, 0))
#
#     pose_goal.position.x = 0.45
#     pose_goal.position.y = 0.3
#     pose_goal.position.z = 0.01
#     #
#     move(group, pose_goal)
# #
