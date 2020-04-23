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
        self.marker_ids = []

    def store_marker_id(self, id):
        self.marker_ids.append(id)

    def positions2pointMessage(self, positions):
        point = Point(x=positions[0], 
                        y = positions[1],
                        z = positions[2])
        return point

    def trajectory2markerArray(self, traj, r, g, b, action=Marker.ADD):
        marker_array = []
        for i,positions in enumerate(traj):
            # if the marker id already exist we need to create new ids
            # to prevent from overwriting
            if i in self.marker_ids:
                i += self.marker_ids[-1]
            point = self.positions2pointMessage(positions)
            pose = Pose(position=point)

            marker = Marker(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.frame_id),
                                            pose=pose,
                                            type=Marker.SPHERE,
                                            scale=Vector3(0.02,0.02,0.02),
                                            id=i,
                                            color=ColorRGBA(r=r, g=g, b=b, a=1),
                                            action=action)
            self.store_marker_id(i)
            marker_array.append(marker)

        return marker_array


    def VisMsgToTraj(self, vis_msg):
        traj = []
        for pose in vis_msg.pose_array:
            traj.append([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w ])
        
        return traj

    def trajToVisMsg(self, traj, r, g, b, frame_id="base_footprint"):
        self.frame_id = frame_id
        pose_array = PoseArray()
        visualization_msg = TrajectoryVisualization()

        # visualization_msg.pose_array = [x[0:7] for x in traj]
        visualization_msg.r = r
        visualization_msg.g = g
        visualization_msg.b = b
        # rospy.loginfo(visualization_msg)
        for data in traj:
            
            pose_ = Pose()

            pose_.position.x = data[0]
            pose_.position.y = data[1]
            pose_.position.z = data[2]

            pose_.orientation.x = data[3]
            pose_.orientation.y = data[4]
            pose_.orientation.z = data[5]
            pose_.orientation.w = data[6]

            pose_array.poses.append(pose_)

            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = frame_id
        # rospy.loginfo(pose_array.poses)

        visualization_msg.pose_array = pose_array.poses
        visualization_msg.r = r
        visualization_msg.g = g
        visualization_msg.b = b

        return visualization_msg