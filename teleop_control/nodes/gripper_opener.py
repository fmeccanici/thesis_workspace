#!/usr/bin/env python

import rospy, time, tf

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelConfiguration

# random comment for committing
class gripperOpener():
    def __init__(self):
        rospy.init_node("gripper_opener")

        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self._gripper_callback)

        self.joint_state_pub = rospy.Publisher("/joint_states_gripper_open", JointState, queue_size=10)

    def _gripper_callback(self, data):

        joint_state_gripper_opened = JointState

        joint_state_gripper_opened = data

        states = list(joint_state_gripper_opened.position)
        states[7] += 1
        states = tuple(states)

        joint_state_gripper_opened.position = states
        self.joint_state_pub.publish(joint_state_gripper_opened)

    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    try:
        node = gripperOpener()
        node.run()
    except Exception as e: 
        rospy.loginfo(e)
        pass