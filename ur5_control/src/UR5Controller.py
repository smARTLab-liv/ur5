from moveit_msgs.msg import DisplayTrajectory
from rospy import Publisher, sleep
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from math import pi
from geometry_msgs.msg import Quaternion, Point, Pose
from tf.transformations import quaternion_from_euler
import copy
import moveit_msgs

import roslib
import time

roslib.load_manifest('robotiq_3f_gripper_control')
from robotiq_3f_gripper_control.msg import _Robotiq3FGripper_robot_output  as outputMsg


def P(pos, ori):
    return Pose(Point(*pos), Quaternion(*quaternion_from_euler(*ori)))


class UR5Controller:
    joint_poses = {
        'all_zeros': [0, 0, 0, 0, 0, 0],  # all zeros
        'frontal': [-pi / 4, -pi / 2, -pi / 2, -pi / 2, pi / 2, -pi / 4],
    }

    # cartesian_poses = {
    #     'start_scanning':
    # }

    def __init__(self):

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("manipulator")
        self.display_trajectory_publisher = Publisher("/move_group/display_planned_path", DisplayTrajectory,
                                                      queue_size=0)
        self.group.allow_replanning(True)

        # Set the reference frame for pose targets
        reference_frame = "/base_link"

        # Set the ur5_arm reference frame accordingly
        self.group.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.group.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.group.set_goal_position_tolerance(0.001)
        self.group.set_goal_orientation_tolerance(0.001)
        self.group.set_planning_time(0.1)
        self.group.set_max_acceleration_scaling_factor(.25)
        self.group.set_max_velocity_scaling_factor(.25)

        print('angles ', pi / 2, pi / 4, (pi / 2) + (pi / 4), (pi / 2) - (pi / 4))
        # c1 = moveit_msgs.msg.JointConstraint('shoulder_pan_joint', pi/2, pi/4 , pi/4, 1.0)
        # c2 = moveit_msgs.msg.JointConstraint('shoulder_lift_joint', 0, pi / 2, pi / 10, 2)
        # c3 = moveit_msgs.msg.JointConstraint('elbow_joint', pi / 2.5, pi / 4, pi / 4, 10)

        constraints = moveit_msgs.msg.Constraints()
        constraints.name = 'ur5_constraints'
        # constraints.joint_constraints.append(c1)
        # constraints.joint_constraints.append(c2)
        # constraints.joint_constraints.append(c3)

        #
        self.group.set_path_constraints(constraints)
        # joints = self.group.get_current_joint_values()
        # print('n constraint: ', len(self.group.get_path_constraints().joint_constraints))
        # print('current joints: ', zip(joints, self.group.get_current_joint_values()))

        ## Gripper
        self.current_command = outputMsg.Robotiq3FGripper_robot_output()
        self.gripper_publisher = Publisher('Robotiq3FGripperRobotOutput', outputMsg.Robotiq3FGripper_robot_output,
                                           queue_size=0)

        sleep(1)

    def move_to_target(self, plan_only):
        plan = self.group.plan()
        if not plan_only:
            if plan.joint_trajectory.header.frame_id != '':
                self.group.go(wait=True)
                self.group.stop()
            else:
                print('Failed to get a plan.')

    def move_to_pose(self, pose, plan_only=True):
        self.group.set_pose_target(pose)
        self.move_to_target(plan_only)
        self.group.clear_pose_targets()

    def move_joints(self, joints, plan_only=True):
        self.group.set_joint_value_target(joints)
        self.move_to_target(plan_only)
        self.group.clear_pose_targets()

    def get_current_pose(self):
        return self.group.get_current_pose().pose

    def get_current_joints(self):
        return self.group.get_current_joint_values()

    def gripper_send_command(self, command):
        self.gripper_publisher.publish(command)
        sleep(0.1)

    def gripper_reset(self):
        command = outputMsg.Robotiq3FGripper_robot_output()
        command.rACT = 0
        self.gripper_send_command(command)
        self.current_command = command

    def gripper_activate(self):
        command = outputMsg.Robotiq3FGripper_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150
        self.gripper_send_command(command)
        self.current_command = command

    def gripper_open(self):
        self.current_command.rPRA = 0
        self.gripper_send_command(self.current_command)

    def gripper_close(self):
        self.current_command.rPRA = 255
        self.gripper_send_command(self.current_command)

    def gripper_set(self, angle):
        self.current_command.rPRA = 0 if angle < 0 else (255 if angle > 255 else angle)
        self.gripper_send_command(self.current_command)

    def gripper_set_pinch(self):
        self.current_command.rMOD = 1
        self.gripper_send_command(self.current_command)

    def move_cartesian(self, waypoints):
        current_pose = [self.get_current_pose()]
        plan, fraction = self.group.compute_cartesian_path(current_pose + waypoints, 0.01, 0.0, True)

        if 1 - fraction < 0.2:
            # rospy.loginfo("Path computed successfully. Moving the arm.")
            num_pts = len(plan.joint_trajectory.points)
            # rospy.loginfo("\n# intermediate waypoints = " + str(num_pts))
            # print(num_pts)
            self.group.execute(plan)

            # for wp in plan.joint_trajectory.points
            # self.arm.execute(plan)
            # rospy.loginfo("Path execution complete.")
        else:
            print("Path planning failed")

    def grid_tracer(self, plan_only, l1=0.13, l2=0.15, n_steps=14, center=None):
        l1_step = (l1 / n_steps)
        l2_step = (l2 / n_steps)
        base_x = (center and center['x']) or 0
        base_y = (center and center['y']) or 0.35
        base_z = (center and center['z']) or 0.076
        base_z_up = base_z + 0.04
        down_position = P((base_x, base_y, base_z), (0, 1.5707, 0))
        up_position = copy.deepcopy(down_position)
        up_position.position.z = down_position.position.z + 0.01

        half = n_steps / 2

        for i in range(-half, half + 1):
            down_position.position.y = base_y + (l1_step * i)
            up_position.position.y = down_position.position.y

            for j in range(-half, half + 1):
                try:
                    down_position.position.x = base_x + (l2_step * j)

                    # print(self.get_current_joints())
                    down_position.position.z = base_z_up
                    self.move_to_pose(down_position, plan_only=plan_only)

                    down_position.position.z = base_z
                    self.move_to_pose(down_position, plan_only=plan_only)

                    yield i, j

                    down_position.position.z = base_z_up
                    self.move_to_pose(down_position, plan_only=plan_only)
                except:
                    print('..')
                    self.stop()
                    raise

    def pick_and_place(self, pose_a, pose_b, m, plan_only=True, block_size=0.11, n_rows=3, n_reps=2):
        pose_a = P(*pose_a)
        pose_b = P(*pose_b)
        pose_m = P(*m)

        self.move_to_pose(pose_m, plan_only=plan_only)

        print('Gripper activate..')
        self.gripper_reset()
        # sleep(5)
        self.gripper_activate()
        sleep(15)
        self.gripper_set_pinch()
        sleep(2)
        self.gripper_open()
        sleep(1)

        print('Gripper ready.')

        for k in range(n_reps):
            even = k % 2 == 0
            pa = copy.deepcopy(pose_a if even else pose_b)
            pa_z = pa.position.z
            pb = copy.deepcopy(pose_b if even else pose_a)
            pb_z = pb.position.z


            for i in range(n_rows):
                # move to picking area
                pa.position.z = pa_z + block_size / 2 + (n_rows - 1 - i) * block_size
                self.move_to_pose(pa, plan_only=plan_only)

                pa.position.z = pa_z + (n_rows - 1 - i) * block_size
                self.move_to_pose(pa, plan_only=plan_only)
                sleep(0.5)

                # pick
                self.gripper_set(90)
                sleep(2)

                pa.position.z = pa_z + block_size / 2 + (n_rows - 1 - i) * block_size
                self.move_to_pose(pa, plan_only=plan_only)

                # move to placing area
                pb.position.z = pb_z + block_size / 2 + i * block_size
                self.move_to_pose(pb, plan_only=plan_only)

                pb.position.z = pb_z + i * block_size
                self.move_to_pose(pb, plan_only=plan_only)
                sleep(0.5)

                # place
                self.gripper_set(60)
                sleep(2)
                #3
                pb.position.z = pb_z + block_size / 2 + i * block_size
                self.move_to_pose(pb, plan_only=plan_only)

            self.move_to_pose(pose_m, plan_only=plan_only)


            # print(i)

    def stop(self):
        self.group.stop()
