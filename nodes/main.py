#!/usr/bin/env python

'''
Usage:
 1) Launch pr2 robot in gazebo
 2) rosrun pr2_tutorial main.py OR ./main.py
 But make sure you sudo +x chmod main.py the
 script (Changes it to an executable)
Author:
 Louis Fernandez - 12555850@student.uts.edu.au
'''

from move_base_pr2 import MoveBase
import time


def main():
    '''
    main function that moves the pr2 and difference between
    calculated pose and odom pose
    '''
    move_straight()
    time.sleep(1)
    move_left()
    time.sleep(1)
    turn_90()


def move_left():
    '''
    Move robot left
    '''
    # User defined values
    lin_vel = 0.5
    direc = 'left'
    dist = 2

    pr2 = MoveBase()
    pr2.move_base(ly=lin_vel, dist=dist, direc=direc)
    measured_pose = pr2.odom_pose['y']
    [calcd_pose, calcd_vel, d_time] = pr2.calculate_pose(direc='left')
    error_pose = abs((calcd_pose-measured_pose)/measured_pose)
    print('odom pose:', measured_pose)
    print('calculated pose:', calcd_pose)
    print('Pose error %% is', error_pose*100)
    print('Actual velocity:', lin_vel)
    print('calculated velocity:', calcd_vel)
    print('Velocity error %% is', ((lin_vel-calcd_vel)/lin_vel)*100)
    print('Time taken to move:', d_time)
    print('')


def turn_90():
    '''
    turn robot 90 degrees
    '''
    # User defined values
    ang_vel = -0.2
    direc = 'turn'

    pr2 = MoveBase()
    pr2.move_base(az=ang_vel, dist=0, direc=direc)
    measured_pose = pr2.odom_pose['yaw']
    [calcd_pose, calcd_vel, d_time] = pr2.calculate_pose(direc=direc)
    error_pose = abs((calcd_pose-measured_pose)/measured_pose)
    print('odom pose:', measured_pose)
    print('calculated pose:', calcd_pose)
    print('Pose error %% is', error_pose*100)
    print('Actual velocity:', ang_vel)
    print('calculated velocity:', calcd_vel)
    print('Velocity error %% is', ((ang_vel-calcd_vel)/ang_vel)*100)
    print('Time taken to move:', d_time)
    print('')


def move_straight():
    '''
    Move robot straight
    '''
    # User defined values
    lin_vel = 1
    dist = 4

    pr2 = MoveBase()
    pr2.move_base(lx=lin_vel, dist=dist)
    measured_pose = pr2.odom_pose['x']
    [calcd_pose, calcd_vel, d_time] = pr2.calculate_pose()
    error_pose = abs((calcd_pose-measured_pose)/measured_pose)
    print('odom pose:', measured_pose)
    print('calculated pose:', calcd_pose)
    print('Pose error %% is', error_pose*100)
    print('Actual velocity:', lin_vel)
    print('calculated velocity:', calcd_vel)
    print('Velocity error %% is', ((lin_vel-calcd_vel)/lin_vel)*100)
    print('Time taken to move:', d_time)
    print('')


if __name__ == '__main__':
    main()
