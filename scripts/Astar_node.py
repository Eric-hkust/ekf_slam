#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose
import time
import Astar
from typing import Dict, List, Iterator, Tuple
import numpy as np
from decimal import *
import os
from gazebo_msgs.msg import ModelStates
import copy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import math

car_pose = []
obstacles = []
graph = Astar.AStar((6.0, 6.0, 0.05))
end_points = [[5.5, 5.5], [0.5, 0.5]]
graph.set_end_point([5.5,5.5])
def Save_Environment(env_data):
    global car_pose
    global obstacles
    car_pose = []
    car_x = env_data.pose[0].position.x
    car_y = env_data.pose[0].position.y
    car_pose = [car_x, car_y]
    obstacles = []
    for i in range(1,len(env_data.pose)):
        obstacles.append([env_data.pose[i].position.x,env_data.pose[i].position.y])

def Update_A_star(car,obst):
    if np.linalg.norm(np.asarray(car[:2]) - np.asarray(end_points[1])) < 0.2:
        graph.set_end_point(end_points[0])
    elif np.linalg.norm(np.asarray(car[:2]) - np.asarray(end_points[0])) < 0.2:
        graph.set_end_point(end_points[1])
    graph.set_start_point(car)
    #print("change")
    graph.clear()
    for i in range(len(obst)):   
        temp_ob = obst[i]
        temp_ob.append(0.08)
        graph.add_cir(temp_ob)
    graph.update()
    path = graph.get_residual_path()
    print("residual path: ", path)
    path.pop(0)
    return path

def Astar_list(path):
    A_list = Float32MultiArray()
    thing = MultiArrayDimension()
    thing.label = "a_star"
    thing.size = 1
    thing.stride = 1
    A_list.layout.dim.append(thing)

    for i in range(len(path)):
        A_list.data.append(path[i][0])
        A_list.data.append(path[i][1])
    return A_list

if __name__ == '__main__':
    rospy.init_node('Astar_node', anonymous=False)
    rate = rospy.Rate(10)
    rospy.Subscriber('Environment', ModelStates, Save_Environment)
    Astar_publisher = rospy.Publisher('A_star', Float32MultiArray, queue_size = 10)
    
    
    while not rospy.is_shutdown():
        if len(car_pose) == 0 or len(obstacles) == 0:
            continue
        temp_car = copy.deepcopy(car_pose[0:2])
        temp_obstacles = copy.deepcopy(obstacles) 
        path = Update_A_star(temp_car, temp_obstacles)
        msg = Astar_list(path)
        # rospy.loginfo(msg)
        Astar_publisher.publish(msg)
        rate.sleep()
    rospy.spin()
