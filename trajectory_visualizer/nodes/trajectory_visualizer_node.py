#!/usr/bin/env python

import rospy
from trajectory_visualizer_python.trajectory_visualizer_python import trajectoryVisualizer
from trajectory_visualizer.msg import TrajectoryVisualization
from learning_from_demonstration.msg import prompTraj
from trajectory_visualizer.srv import VisualizeTrajectory, VisualizeTrajectoryResponse, ClearTrajectories, ClearTrajectoriesResponse

from geometry_msgs.msg import Point, Pose, Vector3, PoseArray
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA
from std_msgs.msg import Bool
from std_srvs.srv import Empty

import numpy as np

class trajectoryVisualizerNode():
    def __init__(self):
        rospy.init_node('trajectory_visualizer')
        self.vis  = trajectoryVisualizer()
        self.traj_pub = rospy.Publisher('trajectory_visualizer/markers', MarkerArray, queue_size=10)
        self.traj_sub = rospy.Subscriber('trajectory_visualizer/trajectory', TrajectoryVisualization, self._traj_callback)
        self._visualize = rospy.Service('visualize_trajectory', VisualizeTrajectory, self._visualize_trajectory)
        self._visualize = rospy.Service('clear_trajectories', ClearTrajectories, self._clear_trajectories)


        # self.traj_pred_pub = rospy.Publisher('trajectory_visualizer/markers_pred', MarkerArray, queue_size=10)
        # self.traj_ref_pub = rospy.Publisher('trajectory_visualizer/markers_ref', MarkerArray, queue_size=10)

        # self.traj_sub_pred = rospy.Subscriber('trajectory_visualizer/trajectory_predicted', TrajectoryVisualization, self._traj_pred_callback)
        # self.traj_sub_refined = rospy.Subscriber('trajectory_visualizer/trajectory_refined', TrajectoryVisualization, self._traj_ref_callback)

        self.frame_id = 'base_footprint'
    
    def prompTrajMessage_to_correct_format(self, traj_msg):
        trajectory = []
        object_position = [traj_msg.object_position.x, traj_msg.object_position.y, traj_msg.object_position.z] 

        for i,pose in enumerate(traj_msg.poses):
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            
            pos = [x, y, z]

            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w

            ori = [qx, qy, qz, qw]

            t = [traj_msg.times[i]]

            

            trajectory.append(pos + ori + object_position + t)
        
        return trajectory

    def _visualize_trajectory(self, req):
        rospy.loginfo("Visualizing trajectory")
        # traj = self.prompTrajMessage_to_correct_format(req.traj)
        traj = self.vis.VisMsgToTraj(req.traj_vis)

        marker_array = self.vis.trajectory2markerArray(traj, req.traj_vis.r, req.traj_vis.g, req.traj_vis.b)

        response = VisualizeTrajectoryResponse()
        suc = Bool()        
        
        try:
            self.traj_pub.publish(marker_array)
            suc.data = True
            response.success = suc
            return response        
        except:
            suc.data = False
            response.success = suc
            return response    

    def _clear_trajectories(self, req):
        empty_traj = np.zeros((1, 7))
        marker_array = self.vis.trajectory2markerArray(empty_traj, r=0, g=0, b=0, action=Marker.DELETEALL)

        response = ClearTrajectoriesResponse()
        suc = Bool()        

        try:
            self.traj_pub.publish(marker_array)
            suc.data = True
            response.success = suc
            return response   
        except:
            suc.data = False
            response.success = suc
            return response 

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