#!/usr/bin/env python2.7

###### Node that converts the end_effector_pose topic to a service #######
import rospy, tf
from geometry_msgs.msg import PoseStamped, Pose
from teach_pendant.srv import GetEEPose, GetEEPoseResponse

class KeyboardControl():
    def __init__(self):
        rospy.init_node('teach_pendant')
        self._get_end_effector_service = rospy.Service('get_end_effector_pose', GetEEPose, self._get_end_effector_pose)
        self.listener = tf.TransformListener()

    def _get_end_effector_pose(self, req):
        self.listener.waitForTransform('/base_footprint', '/arm_tool_link', rospy.Time(), rospy.Duration(4.0))
        (trans,rot) = self.listener.lookupTransform('/base_footprint', '/arm_tool_link', rospy.Time(0))

        resp = GetEEPoseResponse()
        resp.pose = Pose()
        resp.pose.position.x = trans[0]
        resp.pose.position.y = trans[1]
        resp.pose.position.z = trans[2]
        resp.pose.orientation.x = rot[0]
        resp.pose.orientation.y = rot[1]
        resp.pose.orientation.z = rot[2]
        resp.pose.orientation.w = rot[3]

        return resp
            
    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.listener.waitForTransform('/base_footprint', '/arm_tool_link', rospy.Time(), rospy.Duration(4.0))
            (trans,rot) = self.listener.lookupTransform('/base_footprint', '/arm_tool_link', rospy.Time(0))

            r.sleep()

if __name__ == "__main__":
    node = KeyboardControl()
    try:
        node.run()
    except Exception as e: 
        print(e)