#!/usr/bin/env python3.5

import rospy
from visualization_msgs.msg import MarkerArray, Marker

from trajectory_visualizer.msg import TrajectoryVisualization

from geometry_msgs.msg import Point, Pose, Vector3, PoseArray
from std_msgs.msg import Header, ColorRGBA

from learning_from_demonstration.trajectory_parser import trajectoryParser
class trajectoryVisualizer():
    def __init__(self):
        self.frame_id = "base_footprint"

    def positions2pointMessage(self, positions):
        point = Point(x=positions[0], 
                        y = positions[1],
                        z = positions[2])
        return point

    def trajectory2markerArray(self, traj, r, g, b):
        marker_array = []
        for i,positions in enumerate(traj):
            point = self.positions2pointMessage(positions)
            pose = Pose(position=point)

            marker = Marker(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.frame_id),
                                            pose=pose,
                                            type=Marker.SPHERE,
                                            scale=Vector3(0.02,0.02,0.02),
                                            id=i,
                                            color=ColorRGBA(r=r, g=g, b=b, a=1),
                                            action=Marker.ADD)
            marker_array.append(marker)

        return marker_array

    def VisMsgToTraj(self, vis_msg):
        traj = []
        for pose in vis_msg.pose_array.poses:
            traj.append([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w ])
        
        return traj

    def trajToVisMsg(self, traj, r, g, b, frame_id="base_footprint"):
        self.frame_id = frame_id
        pose_array = PoseArray()
        visualization_msg = TrajectoryVisualization()

        for pose in traj:
            pose_ = Pose()

            pose_.position.x = pose[0]
            pose_.position.y = pose[1]
            pose_.position.z = pose[2]

            pose_.orientation.x = pose[3]
            pose_.orientation.y = pose[4]
            pose_.orientation.z = pose[5]
            pose_.orientation.w = pose[6]

            pose_array.poses.append(pose_)

            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = frame_id

        visualization_msg.pose_array = pose_array
        visualization_msg.r = r
        visualization_msg.g = g
        visualization_msg.b = b

        return visualization_msg