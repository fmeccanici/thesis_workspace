#!/usr/bin/env python3.5

###### Node that converts the end_effector_pose topic to a service #######
import rospy
from geometry_msgs.msg import PoseStamped, Pose


class KeyboardControl():
    def __init__(self):
        rospy.init_node('teach_pendant')

        self._end_effector_pose_sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self._end_effector_pose_callback)
        self._get_end_effector_service = rospy.Service('get_end_effector_pose', PoseStamped, self._get_end_effector_pose)

    def _get_end_effector_pose(self, req):
        self.teaching_mode = req.teaching_mode.data

        if self.teaching_mode == 0:
            self.EEtrajectory = []

        # reset white buttons to prevent the append code to run when 
        # we switch to teaching mode when we forgot to press the button
        self.white_button_toggle_previous = 0
        self.white_button_toggle = 0


        rospy.loginfo(("Set teaching mode to {}").format(self.teaching_mode) )

        resp = GetEndEffectorPoseResponse()
        resp.

        return resp

    def _end_effector_pose_callback(self, data):
        self.ee_pose = data.pose
            

if __name__ == "__main__":
    node = KeyboardControl()
    try:
        node.run()
    except Exception as e: 
        print(e)