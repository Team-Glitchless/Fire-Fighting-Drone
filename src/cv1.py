#!/usr/bin/env python
import controler
import rospy
from time import sleep
import numpy as np
import math


def return_oringin(iris):
    iris.set_waypoint(iris.curr_x, iris.curr_y, 10)
    iris.set_pose()
    iris.set_waypoint(0,0, 10)
    iris.set_pose()
    iris.land(0.01)


class trajectory:
    def __init__(self):
        self.waypoints = []
        self.coff = []
        self.index = []


if __name__ == '__main__':
    try:
        iris_controller = controler.Flight_controller()
    except rospy.ROSInterruptException:
        pass
    alt = 3.0
    iris_controller.toggle_arm(True)
    iris_controller.takeoff(alt)
    iris_controller.set_offboard_mode()
    
    #iris_controller.set_waypoint(-3.5,2, alt)
    #iris_controller.set_pose()
    '''print('Current yaw  : '+str(iris_controller.curr_yaw))
    print('set yaw = 0')
    iris_controller.set_orientation(iris_controller.curr_roll, iris_controller.curr_pitch, 0)
    iris_controller.set_pose()
    sleep(3)
    print('set yaw = 1')
    iris_controller.set_orientation(iris_controller.curr_roll, iris_controller.curr_pitch, math.pi/2)
    iris_controller.set_pose()
    print(iris_controller.curr_yaw)
    sleep(5)'''
    iris_controller.move_to(5, 0, 3)
    iris_controller.set_orientation(iris_controller.curr_roll, iris_controller.curr_pitch, 0)
    
    iris_controller.set_pose()
    print(iris_controller.curr_yaw)
    iris_controller.move_to(5, 0, 3)
    print(iris_controller.curr_yaw)
    sleep(10)
    #return_oringin(iris_controller)
    