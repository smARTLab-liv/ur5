from moveit_msgs.msg import DisplayTrajectory
from rospy import Publisher
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from math import pi
from geometry_msgs.msg import Quaternion, Point, Pose
from tf.transformations import quaternion_from_euler
import copy
import moveit_msgs


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

        print(pi/2, pi/4, (pi/2)+(pi/4), (pi/2)-(pi/4))
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

    def grid_tracer(self, plan_only, l1=0.13, l2=0.15, n_steps=14):
        l1_step = (l1 / n_steps)
        l2_step = (l2 / n_steps)
        base_y = 0.35
        base_x = 0
        base_z = 0.076
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

    def stop(self):
        self.group.stop()
