#!/usr/bin/env python

'''
Usage:
 For developing and testing dead reckoning algorithms

Author:
 Louis Fernandez - 12555850@student.uts.edu.au

Useful ROS docs and other resources
 http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html
 https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html
 http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
 http://docs.ros.org/en/api/rosgraph_msgs/html/msg/Clock.html
 http://wiki.ros.org/rospy/Overview/Time
 http://wiki.ros.org/Clock#Using_Simulation_Time_from_the_.2BAC8-clock_Topic

Notes:
 Current code only works for the robot moving straight and turning CW
 Current code only tracking relative position from 'start'

to do (in order):
 Fix output of calculate_pose for when robot is rotating. It
  needs to be negative since we are turning CW
 Add a handle for backwards and CCW turn movements?????????
 Transform position from IMU frame to world frame and test
   the robot position is updated in the x-y plane accordingly.
   Absolute robot position needs to be tracked. Not just local
   position from starting position. May need to add another
   variable which tracks absolute x-y-yaw robot position and
   create a tf transform which broadcasts the imu frame into
   the world frame
 Test code using teleop keyboard

'''

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
import tf_conversions


class MoveBase():
    '''
    Class to move pr2 a certain distance using odom
    msg and then comparing the odom msg with
    calculated pose from imu
    '''

    def __init__(self):
        '''
        Initialise variables to store poses, create
        a new node and subscribe to relevant topics
        '''
        self.odom_pose = {'x': 0, 'y': 0, 'yaw': 0}  # Save pose from odom
        self.imu_pose = []  # Save pose calculated from imu
        self.delta_time = 0  # Use to store time taken for PR2 to move
        self.flag = False  # When set to True, imu output are stored in a variable
        self.sim_time = 0  # Simulation time
        rospy.init_node('move_base_with_odom_imu')
        self.vel_pub = rospy.Publisher('base_controller/command', Twist, queue_size=5)
        self.odom_imu = rospy.Publisher('odom_imu_ref', Odometry, queue_size=5)
        rospy.Subscriber('torso_lift_imu/data', Imu, self.get_imu_data)
        rospy.Subscriber('base_odometry/odom', Odometry, self.odom_callback)
        rospy.Subscriber('clock', Clock, self.clock_callback)

    def clock_callback(self, clock_msg):
        '''
        Store simulation time
        '''
        self.sim_time = clock_msg.clock.to_sec()

    def get_imu_data(self, imu_msg):
        '''
        Callback function to obtain imu data. Store imu data in a list
        so that the average acceleration during the movement can be obtained
        and double integrated to calculate position
        '''
        if self.flag:
            self.imu_pose.append([imu_msg.angular_velocity,
                                 imu_msg.linear_acceleration])

    def odom_callback(self, odom_msg):
        '''
        Stores the robot pose based on odom output
        '''
        # Obtain position and orientation in radians
        pos = odom_msg.pose.pose.position
        [__, __, yaw] = \
            tf_conversions.transformations.euler_from_quaternion([
                                  odom_msg.pose.pose.orientation.x,
                                  odom_msg.pose.pose.orientation.y,
                                  odom_msg.pose.pose.orientation.z,
                                  odom_msg.pose.pose.orientation.w])
        self.odom_pose['x'] = pos.x
        self.odom_pose['y'] = pos.y
        self.odom_pose['yaw'] = yaw

    def calculate_pose(self, direc='straight'):
        '''
        Calculates the pose of the robot based on imu output. Calculted pose
        depends on if the robot's direction movement. Returns the calculated
        pose, average velocity and time taken to travel
        '''

        if direc == 'straight':
            # If robot drove straight, obtain all the lin acc in x dir,
            # average it out and double integrate for position
            lin_acc = [round(p[1].x, 4) for p in self.imu_pose]
            lin_acc_avg = sum(lin_acc)/len(lin_acc)
            v = lin_acc_avg*self.delta_time
            p = v*self.delta_time
            self.imu_pose = []  # Reset variable for recalculation
            o = Odometry()
            o.pose.pose.position.x = p
            o.pose.pose.position.y = 0
            o.pose.pose.position.z = 0
            o.pose.pose.orientation.x = 0
            o.pose.pose.orientation.y = 0
            o.pose.pose.orientation.z = 0
            o.pose.pose.orientation.w = 1
            self.odom_imu.publish(o)
            return p, v, self.delta_time

        elif direc == 'left':
            # If robot drove left, obtain all the lin acc in y dir,
            # average it out and double integrate for position
            lin_acc = [round(p[1].y, 4) for p in self.imu_pose]
            lin_acc_avg = sum(lin_acc)/len(lin_acc)
            v = lin_acc_avg*self.delta_time
            p = v*self.delta_time
            self.imu_pose = []  # Reset variable for recalculation
            o = Odometry()
            o.pose.pose.position.x = 0
            o.pose.pose.position.y = p
            o.pose.pose.position.z = 0
            o.pose.pose.orientation.x = 0
            o.pose.pose.orientation.y = 0
            o.pose.pose.orientation.z = 0
            o.pose.pose.orientation.w = 1
            self.odom_imu.publish(o)
            return p, v, self.delta_time

        else:
            # Robot turns, do the same as above but calculate angular position
            print('to do')
            ang_vel = [round(p[0].z, 4) for p in self.imu_pose]
            ang_vel_avg = sum(ang_vel)/len(ang_vel)
            orientation = self.delta_time*ang_vel_avg
            self.imu_pose = []  # Reset variable for recalculation
            o = Odometry()
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, orientation)
            o.pose.pose.position.x = 0
            o.pose.pose.position.y = 0
            o.pose.pose.position.z = 0
            o.pose.pose.orientation.x = q[0]
            o.pose.pose.orientation.y = q[1]
            o.pose.pose.orientation.z = q[2]
            o.pose.pose.orientation.w = q[3]
            self.odom_imu.publish(o)
            return orientation, ang_vel_avg, self.delta_time

    def move_base(self, dist, lx=0, ly=0, az=0, direc='straight'):
        '''
        Moves the PR2 by publishing to the /base_controller/command
        topic. The time taken to move the PR2 is also recorded. Takes
        in linear x and y velocities, as well as the z angular velocity.
        Also takes in a string variable, direc, to determine which axis
        to measure the distance from and a user defined distance
        '''
        vel = Twist()
        vel.linear.x = lx
        vel.linear.y =ly
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = az

        if direc == 'straight':  # Move straight
            while not rospy.get_rostime():
                # A no-op. Waits until first message has
                # been received by clock
                pass
            rospy.sleep(0.001)
            ref_disp = self.odom_pose['x']
            self.flag = True  # Start recording imu outputs
            reference_time = self.sim_time
            # print('Time at which robot moved:', reference_time)
            while abs(self.odom_pose['x'] - ref_disp) < dist:
                # Move PR2 a given distance
                self.vel_pub.publish(vel)
                rospy.sleep(0.001)
            self.flag = False  # Start recording imu outputs
            vel.linear.x = 0
            self.vel_pub.publish(vel)
            self.delta_time = self.sim_time - reference_time

        elif direc == 'left':  # Move straight
            while not rospy.get_rostime():
                # A no-op. Waits until first message has
                # been received by clock
                pass
            rospy.sleep(0.001)
            ref_disp = self.odom_pose['y']
            self.flag = True  # Start recording imu outputs
            reference_time = self.sim_time
            # print('Time at which robot moved:', reference_time)
            while abs(self.odom_pose['y'] - ref_disp) < dist:
                # Move PR2 a given distance
                self.vel_pub.publish(vel)
                rospy.sleep(0.001)
            self.flag = False  # Start recording imu outputs
            vel.linear.y = 0
            self.vel_pub.publish(vel)
            self.delta_time = self.sim_time - reference_time

        else:  # Turn
            ref_yaw = self.odom_pose['yaw']

            if 0 <= ref_yaw <= 4.71:
                # If in 3rd and 2nd quadrant, measure
                # the angle difference to check for 90
                # degree turn
                while not rospy.get_rostime():
                    pass
                rospy.sleep(0.001)
                self.flag = True
                reference_time = rospy.get_time()
                while abs(self.odom_pose['yaw'] - ref_yaw) < 1.57:
                    # Turn PR2 90 degrees
                    self.vel_pub.publish(vel)
                    rospy.sleep(0.001)
                self.flag = False
                vel.angular.z = 0
                self.vel_pub.publish(vel)
                self.delta_time = reference_time - rospy.get_time()

            elif ref_yaw < 1.57:
                # Handle for first quadrant
                pass

            elif ref_yaw > 4.71:
                # Handle for fourth quadrant
                pass

            else:
                print('Missed a handle for a case')
                import sys; sys.exit()
