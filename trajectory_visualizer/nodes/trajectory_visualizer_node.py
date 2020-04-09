#!/usr/bin/env python

import rospy
from trajectory_visualizer_python.trajectory_visualizer_python import trajectoryVisualizer
from trajectory_visualizer.msg import TrajectoryVisualization

from geometry_msgs.msg import Point, Pose, Vector3, PoseArray
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA

class trajectoryVisualizerNode():
    def __init__(self):
        rospy.init_node('trajectory_visualizer')
        self.vis  = trajectoryVisualizer()
        self.traj_pub = rospy.Publisher('trajectory_visualizer/markers', MarkerArray, queue_size=10)
        self.traj_sub = rospy.Subscriber('trajectory_visualizer/trajectory', TrajectoryVisualization, self._traj_callback)


        # self.traj_pred_pub = rospy.Publisher('trajectory_visualizer/markers_pred', MarkerArray, queue_size=10)
        # self.traj_ref_pub = rospy.Publisher('trajectory_visualizer/markers_ref', MarkerArray, queue_size=10)

        # self.traj_sub_pred = rospy.Subscriber('trajectory_visualizer/trajectory_predicted', TrajectoryVisualization, self._traj_pred_callback)
        # self.traj_sub_refined = rospy.Subscriber('trajectory_visualizer/trajectory_refined', TrajectoryVisualization, self._traj_ref_callback)

        self.frame_id = 'base_footprint'

    # def VisMsgToTraj(self, vis_msg):
    #     traj = []
    #     for pose in vis_msg.pose_array.poses:
    #         traj.append([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w ])
        
    #     return traj

    def _traj_callback(self, data):
        marker_array = MarkerArray()

        traj = self.vis.VisMsgToTraj(data)
        if traj[0][0] == 0.0:
            point = self.vis.positions2pointMessage([0.0, 0.0, 0.0])
            pose = Pose(position=point)
            marker = Marker(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.frame_id),
                                            pose=pose,
                                            type=Marker.SPHERE,
                                            scale=Vector3(0.02,0.02,0.02),
                                            id=0,
                                            color=ColorRGBA(r=0, g=0, b=0, a=0),
                                            action=Marker.DELETEALL)
            marker_array = [marker]
        else:
            marker_array = self.vis.trajectory2markerArray(traj, data.r, data.g, data.b)
        
        self.traj_pub.publish(marker_array)


    # def _traj_pred_callback(self, data):
    #     marker_array = MarkerArray()

    #     traj = self.VisMsgToTraj(data)
    #     if traj[0][0] == 0.0:
    #         point = self.positions2pointMessage([0.0, 0.0, 0.0])
    #         pose = Pose(position=point)
    #         marker = Marker(header=Header(stamp=rospy.Time.now(),
    #                                         frame_id=self.frame_id),
    #                                         pose=pose,
    #                                         type=Marker.SPHERE,
    #                                         scale=Vector3(0.02,0.02,0.02),
    #                                         id=0,
    #                                         color=ColorRGBA(r=0, g=0, b=0, a=0),
    #                                         action=Marker.DELETEALL)
    #         marker_array = [marker]
    #     else:
    #         marker_array = self.trajectory2markerArray(traj, data.r, data.g, data.b)
        
    #     self.traj_pred_pub.publish(marker_array)

    # def _traj_ref_callback(self, data):
    #     marker_array = MarkerArray()

    #     traj = self.VisMsgToTraj(data)
    #     if traj[0][0] == 0.0:
    #         point = self.positions2pointMessage([0.0, 0.0, 0.0])
    #         pose = Pose(position=point)
    #         marker = Marker(header=Header(stamp=rospy.Time.now(),
    #                                         frame_id=self.frame_id),
    #                                         pose=pose,
    #                                         type=Marker.SPHERE,
    #                                         scale=Vector3(0.02,0.02,0.02),
    #                                         id=0,
    #                                         color=ColorRGBA(r=0, g=0, b=0, a=0),
    #                                         action=Marker.DELETEALL)
    #         marker_array = [marker]
    #     else:
    #         marker_array = self.trajectory2markerArray(traj, data.r, data.g, data.b)

    #     self.traj_ref_pub.publish(marker_array)

    # def positions2pointMessage(self, positions):
    #     point = Point(x=positions[0], 
    #                     y = positions[1],
    #                     z = positions[2])
    #     return point

    # def trajectory2markerArray(self, traj, r, g, b):
    #     marker_array = []
    #     for i,positions in enumerate(traj):
    #         point = self.positions2pointMessage(positions)
    #         pose = Pose(position=point)

    #         marker = Marker(header=Header(stamp=rospy.Time.now(),
    #                                         frame_id=self.frame_id),
    #                                         pose=pose,
    #                                         type=Marker.SPHERE,
    #                                         scale=Vector3(0.02,0.02,0.02),
    #                                         id=i,
    #                                         color=ColorRGBA(r=r, g=g, b=b, a=1),
    #                                         action=Marker.ADD)
    #         marker_array.append(marker)

    #     return marker_array

    def publishMarkerArray(self, marker_array):
        self.traj_pub.publish(MarkerArray(markers=marker_array))

    def run(self):
        r = rospy.Rate(30)

        while not rospy.is_shutdown():
            r.sleep()



if __name__ == "__main__":
    trajectory_visualizer = trajectoryVisualizerNode()
    trajectory_visualizer.run()