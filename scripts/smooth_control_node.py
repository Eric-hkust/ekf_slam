#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
import numpy as np
from trajectory import SmoothCtrlPredict
#from decimal import *
import os
import copy
import math
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray

car_pose = []
A_star = []
trajectory_predict = []
control_predict = []
flag = 0

def Save_Car_pose(car_data):
    global car_pose
    car_pose = [car_data.pose[0].position.x,car_data.pose[0].position.y,car_data.pose[0].position.z]

def Save_A_star(A_list):
    global car_pose
    global A_star
    global trajectory_predict
    global control_predict
    global flag
    A_star = []
    temp = copy.deepcopy(A_list.data)
    if len(car_pose) == 0 or len(temp) == 0:
        return 
    for i in range(int(len(temp)/2)):
        A_star.append((temp[2*i],temp[2*i+1]))
    SC_controller = SmoothCtrlPredict(tuple(car_pose),A_star,dt = 0.05)
    trajectory_predict,control_predict = SC_controller.predict(True)
    flag = 0

def Control_Publisher():
    global control_predict
    global flag
    if len(control_predict) == 0 or flag >= len(control_predict):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        return msg
    msg = Twist()
    msg.linear.x = (control_predict[flag][0])
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = (control_predict[flag][1])
    flag += 1
    return msg

if __name__ == '__main__':
    rospy.init_node('SC_node', anonymous=False)
    rate = rospy.Rate(20)
    rospy.Subscriber('/Environment', ModelStates, Save_Car_pose)
    rospy.Subscriber('A_star', Float32MultiArray, Save_A_star)
    control_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    
    while not rospy.is_shutdown():
        msg = Control_Publisher()
        rospy.loginfo(msg)
        control_publisher.publish(msg)

        rate.sleep()
    rospy.spin()
