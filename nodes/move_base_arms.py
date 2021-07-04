#!/usr/bin/env python

from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
from math import radians
import tf_conversions as tf2
import sys
import copy
import rospy
import moveit_commander
import trajectory_msgs.msg

class MovePR2(object):
    '''
    Class for moving PR2
    '''

    def __init__(self):
        '''
        Initialise class
        '''

        # Initialise nodes and create publishers/subscribers as a class attribute
        rospy.init_node('move_pr2', anonymous=True)
        rospy.Subscriber('/base_scan', LaserScan, self.get_laser_data)
        self.vel_pub = rospy.Publisher('/base_controller/command', Twist, queue_size=1)

        # Initialise moveit and create class attributes to define group joints
        moveit_commander.roscpp_initialize(sys.argv)
        self.pr2 = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.left_arm = moveit_commander.MoveGroupCommander('left_arm')
        self.right_arm = moveit_commander.MoveGroupCommander('right_arm')
        self.torso = moveit_commander.MoveGroupCommander('torso')
        self.head = moveit_commander.MoveGroupCommander('head')
        self.left_gripper = moveit_commander.MoveGroupCommander('left_gripper')
        self.left_arm.set_pose_reference_frame('base_footprint')
        # self.right_gripper = moveit_commander.MoveGroupCommander('right_gripper')
        print('\n#############################################################\n')
        print('Left arm frame of reference:', self.left_arm.get_planning_frame())
        # get_end_effector_link()
        print('\nAvailable groups for this robot:', self.pr2.get_group_names())

        # Initialise all required variables as a class attribute
        self.ranges = [] # Used to obtain distances from the base laser scan
        self.data = [] # Used to obtain point cloud values from wide stereo camera

        while not rospy.get_rostime():
            # A no-op. Waits until first message has
            # been received by clock
            print('waiting for first message...')
            rospy.sleep(0.1)

    def get_laser_data(self, laser_msg):
        self.ranges = laser_msg.ranges

    def move_base(self, vel):
        twist = Twist()
        twist.linear.x = vel[0]
        twist.linear.y = vel[1]
        twist.linear.z = vel[2]
        twist.angular.x = vel[3]
        twist.angular.y = vel[4]
        twist.angular.z = vel[5]
        self.vel_pub.publish(twist)
        rospy.sleep(0.01)

    def move_to_home(self):
        to_rad = lambda tup_deg : tuple(radians(i) for i in tup_deg)

        # Home position for left, right, torso and head joints
        home_positions = (to_rad((5, -9, 8, -85, -180, -85, -177)), to_rad((-6, -10, 0, -87, -174, -81, -168)), to_rad((0, 20)), 0.03)
        left_arm_joint_goal = self.left_arm.get_current_joint_values()
        right_arm_joint_goal = self.right_arm.get_current_joint_values()
        left_gripper_goal = self.left_gripper.get_current_joint_values()
        head_goal = self.head.get_current_joint_values()
        torso_goal = self.torso.get_current_joint_values()
        torso_goal[0] = home_positions[-1]
        left_gripper_goal[0] = radians(16)
        left_gripper_goal[1] = radians(16)
        left_gripper_goal[4] = radians(16)
        left_gripper_goal[5] = radians(16)
        print(left_gripper_goal)


        for i in range(7):
            left_arm_joint_goal[i] = home_positions[0][i]
            right_arm_joint_goal[i] = home_positions[1][i]
            if i < 2:
                head_goal[i] = home_positions[2][i]

        # self.head.go(head_goal)
        self.left_gripper.go(left_gripper_goal)
        # self.torso.go(torso_goal)
        # self.left_arm.go(left_arm_joint_goal)
        # self.right_arm.go(right_arm_joint_goal)
        # self.head.stop()
        self.left_gripper.stop()
        # self.torso.stop()
        # self.left_arm.stop()
        # self.right_arm.stop()

    def move_to_pose(self, left_arm = True, goal_pose = None):
        if left_arm:
            arm = self.left_arm
        else:
            arm = self.right_arm

        arm.go(goal_pose, wait=True)
        arm.stop()
        arm.clear_pose_targets()


if __name__ == '__main__':
    pr2 = MovePR2()
    pr2.move_to_home()
    # while not pr2.ranges:
    #     print(pr2.ranges)
    # while min(pr2.ranges) >= 0.5:
    #     pr2.move_base([0.75, 0, 0, 0, 0, 0])
    #     print(min(pr2.ranges))
    # pr2.move_base([0, 0, 0, 0, 0, 0])
    # print(min(pr2.ranges))

    # banana_pose = Pose()
    # quaternion = tf2.transformations.quaternion_from_euler(0, 90, 0)
    # banana_pose.orientation.x = quaternion[0]
    # banana_pose.orientation.y = quaternion[1]
    # banana_pose.orientation.z = quaternion[2]
    # banana_pose.orientation.w = quaternion[3]
    # banana_pose.position.x = 0.7374
    # banana_pose.position.y = 0.04
    # banana_pose.position.z = 0.7734 + 0.15 # Need to figure out the offset of the gripper
    # pr2.move_to_pose(goal_pose = banana_pose)
