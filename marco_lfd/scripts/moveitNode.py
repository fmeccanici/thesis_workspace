#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import keyboard

class moveitNode():
    
    def __init__(self):
        rospy.init_node('moveit_node', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "marco_arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

    def displayTrajectory(self):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        
