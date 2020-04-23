#!/usr/bin/env python

import rospy
from trajectory_visualizer_python.trajectory_visualizer_python import trajectoryVisualizer
from trajectory_visualizer.msg import TrajectoryVisualization
from promp_context_ros.msg import prompTraj
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

        self._visualize_service = rospy.Service('visualize_trajectory', VisualizeTrajectory, self._visualize_trajectory)
        self._clear_trajectories_service = rospy.Service('clear_trajectories', ClearTrajectories, self._clear_trajectories)

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
        action = Marker.ADD
        marker_array = self.vis.trajectory2markerArray(traj, req.traj_vis.r, req.traj_vis.g, req.traj_vis.b, action)

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
        
        # empty marker id vector
        self.vis.marker_ids = []
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

    def publishMarkerArray(self, marker_array):
        self.traj_pub.publish(MarkerArray(markers=marker_array))

    def run(self):
        r = rospy.Rate(30)

        while not rospy.is_shutdown():
            r.sleep()


if __name__ == "__main__":
    trajectory_visualizer = trajectoryVisualizerNode()
    trajectory_visualizer.run()