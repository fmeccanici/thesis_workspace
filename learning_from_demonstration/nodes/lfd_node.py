#!/usr/bin/env python3.5

# import ros related packages
import rospy 

# import my own classes
from learning_from_demonstration.learning_from_demonstration import learningFromDemonstration

class lfdNode():
    def __init__(self):
        # initialize ros related
        rospy.init_node('lfd_node')
        rospy.Subscriber('learning_from_demonstration/')

        # initialize other classes
        lfd = learningFromDemonstration()

    