#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from slave_control.msg import ControlState

class armGoalNode():
    def __init__(self, position=True, orientation=False):
        rospy.init_node('only_position_node')
        
        if orientation == False and position == True:
            self.only_position_sub = rospy.Subscriber("/whole_body_kinematic_controller/arm_tool_link_goal_dummy", geometry_msgs.msg.PoseStamped, self.only_position_callback)
        elif orientation == True and position == True:
            self.position_and_orientation_sub = rospy.Subscriber("/whole_body_kinematic_controller/arm_tool_link_goal_dummy", geometry_msgs.msg.PoseStamped, self.position_and_orientation_callback)
        
        self.slave_control_state_sub = rospy.Subscriber("slave_control_state", ControlState, self._slave_control_state_callback)
        self.end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", geometry_msgs.msg.PoseStamped, queue_size=10)

    def _slave_control_state_callback(self,data):
        self.current_slave_pose = data.slave_pose.pose

    def only_position_callback(self, data):
        data.pose.orientation.x = self.current_slave_pose.orientation.x
        data.pose.orientation.y = self.current_slave_pose.orientation.y
        data.pose.orientation.z = self.current_slave_pose.orientation.z
        data.pose.orientation.w = self.current_slave_pose.orientation.z

        self.end_effector_goal_pub.publish(data)
        rospy.loginfo("publishing only position")

    def position_and_orientation_callback(self, data):

        self.end_effector_goal_pub.publish(data)
        rospy.loginfo("publishing position and orientation")

if __name__ == "__main__":
    only_position_node = armGoalNode(True, False)
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        r.sleep()