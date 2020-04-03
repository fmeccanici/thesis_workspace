#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from aruco_msgs.msg import MarkerArray
from pyquaternion import Quaternion
import numpy as np

class EEwrtMarkerPublisher():
    def __init__(self):
        rospy.init_node('ee_wrt_marker_publisher')
        self.marker_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self._marker_detection_callback)
        self.end_effector_pose_sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self._end_effector_pose_callback)

        self.ee_wrt_marker_pub = rospy.Publisher("/end_effector_pose_wrt_marker", PoseStamped, queue_size=10)

        self.object_marker_pose = Pose()

    def _marker_detection_callback(self, data):
        # print("marker pose = " + str(self.marker_pose_static.position))
        for marker in data.markers:
            if marker.id == 582:
                self.object_marker_pose = marker.pose.pose
            else: continue

    def _end_effector_pose_callback(self,data):
        self.ee_wrt_marker_pub.publish( self.ee_pose_wrt_object(data))

    def ee_pose_wrt_object(self, ee_pose):
        ee_pos_wrt_base = ([ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z])
        ee_orient_wrt_base = ([ee_pose.pose.orientation.w, ee_pose.pose.orientation.x, ee_pose.pose.orientation.y, ee_pose.pose.orientation.z])
        marker_pos_wrt_base = ([self.object_marker_pose.position.x, self.object_marker_pose.position.y, self.object_marker_pose.position.z])

        # express r_ee in marker frame instead of base_footprint
        p = list(np.subtract(ee_pos_wrt_base, marker_pos_wrt_base))
        r_ee_wrt_marker = p

        new_pose = PoseStamped()
        new_pose.pose.position.x = r_ee_wrt_marker[0] 
        new_pose.pose.position.y = r_ee_wrt_marker[1] 
        new_pose.pose.position.z = r_ee_wrt_marker[2] 
        new_pose.pose.orientation.x = ee_orient_wrt_base[0]
        new_pose.pose.orientation.y = ee_orient_wrt_base[1]
        new_pose.pose.orientation.z = ee_orient_wrt_base[2]
        new_pose.pose.orientation.w = ee_orient_wrt_base[3]
        new_pose.header.stamp = rospy.Time.now()
        new_pose.header.frame_id = 'base_footprint'


        return new_pose
        
    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    node = EEwrtMarkerPublisher()
    try:
        node.run()
    except Exception as e: rospy.loginfo(e)
