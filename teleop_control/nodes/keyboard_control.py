#!/usr/bin/env python2.7

import rospy, time, tf

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from pynput.keyboard import Key, Listener
import threading, pynput

class teleopControl():
    def __init__(self):
        rospy.init_node("teleop_control")
        self.frame_id = 'base_footprint'

        # get ros parameters
        self._getParameters()

        self.end_effector_goal_sub = rospy.Subscriber("/whole_body_kinematic_controller/arm_tool_link_goal_dummy", PoseStamped, self._end_effector_goal_callback)
        self._end_effector_pose_sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self._end_effector_pose_callback)
        
        self.end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)


    def _end_effector_pose_callback(self, data):
        self.current_slave_pose = data.pose


    def _end_effector_goal_callback(self, data):
        ee_pose = PoseStamped()

        # rospy.loginfo('check')
        if self.part_to_publish == "position":
            
            ee_pose.pose.position.x = data.pose.position.x
            ee_pose.pose.position.y = data.pose.position.y
            ee_pose.pose.position.z = data.pose.position.z
            ee_pose.pose.orientation.x = self.current_slave_pose.orientation.x
            ee_pose.pose.orientation.y = self.current_slave_pose.orientation.y
            ee_pose.pose.orientation.z = self.current_slave_pose.orientation.z
            ee_pose.pose.orientation.w = self.current_slave_pose.orientation.w

            ee_pose.header.stamp = rospy.Time.now()
            self.end_effector_goal_pub.publish(ee_pose)
            
        elif self.part_to_publish == "both":
            
            ee_pose.pose.position.x = data.pose.position.x
            ee_pose.pose.position.y = data.pose.position.y
            ee_pose.pose.position.z = data.pose.position.z

            ee_pose.pose.orientation.x = data.pose.orientation.x
            ee_pose.pose.orientation.y = data.pose.orientation.y
            ee_pose.pose.orientation.z = data.pose.orientation.z
            ee_pose.pose.orientation.w = data.pose.orientation.w

            ee_pose.header.stamp = rospy.Time.now()
            self.end_effector_goal_pub.publish(ee_pose)

        elif self.part_to_publish == "none":
            pass

        

    def _getParameters(self):
        self.part_to_publish = rospy.get_param('~part_to_publish')            

    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    try:
        node = teleopControl()
        node.run()
    except Exception as e: 
        rospy.loginfo(e)
        pass