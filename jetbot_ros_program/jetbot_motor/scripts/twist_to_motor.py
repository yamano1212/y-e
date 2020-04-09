#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import Twist
from jetbot import Robot

v_r = 0
v_l = 0

def twist_cb(data):
    # Twistを取得
    v = data.linear.x#(m/s)
    omega = data.angular.z#(rad/s)

    global v_r
    global v_l
    # Twistから速度へ変換
    v_r = (omega/2 + v)
    v_l = (-omega/2 + v)

def listener():
    rospy.init_node('twist_to_motor')
    rospy.Subscriber('dtw_robot/diff_drive_controller/cmd_vel', Twist, twist_cb)
    rate = rospy.Rate(10)
    robot = Robot()

    global init_flag
    init_flag = 1

    while not rospy.is_shutdown():
        robot.set_motors(v_l, v_r)
        print(v_l)
        rate.sleep()


if __name__ == '__main__':
    listener()