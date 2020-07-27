#!/usr/bin/env python

import rospy, os, tf
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA, Bool
from geometry_msgs.msg import Vector3, Pose, PoseStamped
import numpy as np
import copy
from pyquaternion import Quaternion

from gazebo_msgs.msg import LinkStates
from execution_failure_detection.srv import GetExecutionFailure, GetExecutionFailureResponse, SetExpectedObjectPosition, SetExpectedObjectPositionResponse
from execution_failure_detection.msg import ExecutionFailure
from geometry_msgs.msg import Point

class SceneDrawer(object):
    def __init__(self, model_path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/execution_failure_detection/models/'):
        rospy.init_node('scene_drawer')
        self.table_pose_counter = 0
        
        ##### true size ######
        self.table_size_x = 0.8
        self.table_size_y = 1.5
        self.table_size_z = 0.66
        
        ##### size adapted for collision ######
        # x = 0.8
        # y = 1.5
        # z = 0.7

        # sizes are wrt base_footprint frame
        self.object_size_x = 0.05
        self.object_size_y = 0.05
        self.object_size_z = 0.1

        self.frame_id = 'base_footprint'
        self.model_path = model_path
        self.models = {}

        self.model_pub = rospy.Publisher('execution_failure_detection/model_visualization', MarkerArray, queue_size=10)
        self.ee_sub = rospy.Subscriber('end_effector_pose', PoseStamped, self._endEffectorPoseCallback)
        self.link_states_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.linkStatesCallback)
        
        ########### TF ###############
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
    
    def _endEffectorPoseCallback(self, data):
        self.ee_pose = data.pose

    def linkStatesCallback(self, data):
        self.object_pose = data.pose[1]
        self.table_pose = data.pose[2]
                
        if self.table_pose_counter == 0:
            color = 'red'
            self.table_pose.position.x = self.table_pose.position.x + self.table_size_x/2
            self.table_pose.position.y = self.table_pose.position.y + self.table_size_y/2
            self.table_pose.position.z = self.table_pose.position.z + self.table_size_z/2

            self.addCube(self.table_size_x, self.table_size_y, self.table_size_z, self.table_pose, color, 1)
        
        color = 'blue'
        self.addCube(self.object_size_x, self.object_size_y, self.object_size_z, self.object_pose, color, 2)

    # set size of ellipsoid wrt base frame
    def setEllipsoidSize(self, ellipsoid_type='collision'):

        while True:    
            try:
                rospy.wait_for_message('end_effector_pose', PoseStamped)
                q_ee = Quaternion(copy.deepcopy([self.ee_pose.orientation.w, self.ee_pose.orientation.x, self.ee_pose.orientation.y, self.ee_pose.orientation.z]))

                if ellipsoid_type == 'collision':
                    size_wrt_ee = [0.4, 0.35, -0.04]
                    size_wrt_base = q_ee.inverse.rotate(size_wrt_ee)

                    self.collision_ellipsoid_size_x = size_wrt_base[0]
                    self.collision_ellipsoid_size_y = size_wrt_base[1]
                    self.collision_ellipsoid_size_z = size_wrt_base[2] # 0.1 is best

                elif ellipsoid_type == 'reaching':
                    # size_wrt_ee = [0.15, 0.2, 0.1]
                    # size_wrt_base = q_ee.inverse.rotate(size_wrt_ee)
                    size_wrt_base = [0.075, 0.2, 0.1]

                    self.reaching_ellipsoid_size_x = size_wrt_base[0]
                    self.reaching_ellipsoid_size_y = size_wrt_base[1]
                    self.reaching_ellipsoid_size_z = size_wrt_base[2] 

                elif ellipsoid_type == 'all':
                    # size_wrt_ee = [0.4, 0.2, 0.1]
                    # size_wrt_ee = q_ee.rotate(size_wrt_ee)
                    size_wrt_base = [0.4, 0.2, 0.1]
                    self.collision_ellipsoid_size_x = size_wrt_base[0]
                    self.collision_ellipsoid_size_y = size_wrt_base[1]
                    self.collision_ellipsoid_size_z = size_wrt_base[2] # 0.1 is best
                    
                    # size_wrt_ee = [-0.15, 0.1, 0.1]
                    # size_wrt_base = q_ee.inverse.rotate(size_wrt_ee)
                    # size_wrt_ee = q_ee.rotate(size_wrt_ee)
                    
                    size_wrt_base = [0.075, 0.2, 0.1]

                    self.reaching_ellipsoid_size_x = size_wrt_base[0]
                    self.reaching_ellipsoid_size_y = size_wrt_base[1]
                    self.reaching_ellipsoid_size_z = size_wrt_base[2] 
                break

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    # origin of ellipsoid wrt base_footprint frame
    def setEllipsoidOrigin(self, ellipsoid_type='collision'):
        q_ee = Quaternion(copy.deepcopy([self.ee_pose.orientation.w, self.ee_pose.orientation.x, self.ee_pose.orientation.y, self.ee_pose.orientation.z]))

        if ellipsoid_type == 'collision':
            self.collision_ellipsoid_origin = copy.deepcopy(self.ee_pose)
            r_ellipsoid_wrt_ee = [0.02, 0, -0.01]
            r_ellipsoid_wrt_base = q_ee.rotate(r_ellipsoid_wrt_ee)

            self.collision_ellipsoid_origin.position.x += r_ellipsoid_wrt_base[0]
            self.collision_ellipsoid_origin.position.y += r_ellipsoid_wrt_base[1]
            self.collision_ellipsoid_origin.position.z += r_ellipsoid_wrt_base[2]
        
        elif ellipsoid_type == 'reaching':
            self.reaching_ellipsoid_origin = copy.deepcopy(self.ee_pose)

            r_ellipsoid_wrt_ee = [0.11, 0, 0]
            r_ellipsoid_wrt_base = q_ee.rotate(r_ellipsoid_wrt_ee)

            self.reaching_ellipsoid_origin.position.x += r_ellipsoid_wrt_base[0]
            self.reaching_ellipsoid_origin.position.y += r_ellipsoid_wrt_base[1]
            self.reaching_ellipsoid_origin.position.z += r_ellipsoid_wrt_base[2]

        elif ellipsoid_type == 'all':
            self.collision_ellipsoid_origin = copy.deepcopy(self.ee_pose) 
            self.reaching_ellipsoid_origin = copy.deepcopy(self.ee_pose)

            # express origin wrt ee frame
            r_ellipsoid_wrt_ee = [0.02, 0, 0]
            
            # rotate with ee frame to get ellipsoid origin in base frame
            r_ellipsoid_wrt_base = q_ee.rotate(r_ellipsoid_wrt_ee)

            self.collision_ellipsoid_origin.position.x += r_ellipsoid_wrt_base[0]
            self.collision_ellipsoid_origin.position.y += r_ellipsoid_wrt_base[1]
            self.collision_ellipsoid_origin.position.z += r_ellipsoid_wrt_base[2]
            
            r_ellipsoid_wrt_ee = [0.11, 0, 0]
            r_ellipsoid_wrt_base = q_ee.rotate(r_ellipsoid_wrt_ee)

            self.reaching_ellipsoid_origin.position.x += r_ellipsoid_wrt_base[0]
            self.reaching_ellipsoid_origin.position.y += r_ellipsoid_wrt_base[1]
            self.reaching_ellipsoid_origin.position.z += r_ellipsoid_wrt_base[2]



    def broadcastFrames(self, ellipsoid_type='all'):

        if ellipsoid_type == 'all':
            # publish collision ellipsoid frame wrt base footprint --> used for visualization in RViz
            self.broadcaster.sendTransform((self.reaching_ellipsoid_origin.position.x, self.reaching_ellipsoid_origin.position.y, self.reaching_ellipsoid_origin.position.z),
                                            (self.reaching_ellipsoid_origin.orientation.x, self.reaching_ellipsoid_origin.orientation.y, self.reaching_ellipsoid_origin.orientation.z,
                                            self.reaching_ellipsoid_origin.orientation.w),
                                            rospy.Time.now(), "reaching_ellipsoid", self.frame_id)

            # publish collision ellipsoid frame wrt base footprint --> used for visualization in RViz
            self.broadcaster.sendTransform((self.collision_ellipsoid_origin.position.x, self.collision_ellipsoid_origin.position.y, self.collision_ellipsoid_origin.position.z),
                                            (self.collision_ellipsoid_origin.orientation.x, self.collision_ellipsoid_origin.orientation.y, self.collision_ellipsoid_origin.orientation.z,
                                            self.collision_ellipsoid_origin.orientation.w),
                                            rospy.Time.now(), "collision_ellipsoid", self.frame_id)
                
            # publish collision ellipsoid frame wrt base footprint --> used for visualization in RViz
            self.broadcaster.sendTransform((self.object_pose.position.x, self.object_pose.position.y, self.object_pose.position.z),
                                            (self.object_pose.orientation.x, self.object_pose.orientation.y, self.object_pose.orientation.z,
                                            self.object_pose.orientation.w),
                                            rospy.Time.now(), "object", self.frame_id)
            
            # publish collision ellipsoid frame wrt base footprint --> used for visualization in RViz
            self.broadcaster.sendTransform((self.table_pose.position.x, self.table_pose.position.y, self.table_pose.position.z),
                                            (self.table_pose.orientation.x, self.table_pose.orientation.y, self.table_pose.orientation.z,
                                            self.table_pose.orientation.w),
                                            rospy.Time.now(), "table", self.frame_id)
        
        elif ellipsoid_type == 'reaching':
            # publish collision ellipsoid frame wrt base footprint --> used for visualization in RViz
            self.broadcaster.sendTransform((self.reaching_ellipsoid_origin.position.x, self.reaching_ellipsoid_origin.position.y, self.reaching_ellipsoid_origin.position.z),
                                            (self.reaching_ellipsoid_origin.orientation.x, self.reaching_ellipsoid_origin.orientation.y, self.reaching_ellipsoid_origin.orientation.z,
                                            self.reaching_ellipsoid_origin.orientation.w),
                                            rospy.Time.now(), "reaching_ellipsoid", self.frame_id)
            
            # publish collision ellipsoid frame wrt base footprint --> used for visualization in RViz
            self.broadcaster.sendTransform((self.object_pose.position.x, self.object_pose.position.y, self.object_pose.position.z),
                                            (self.object_pose.orientation.x, self.object_pose.orientation.y, self.object_pose.orientation.z,
                                            self.object_pose.orientation.w),
                                            rospy.Time.now(), "object", self.frame_id)
            
            # publish collision ellipsoid frame wrt base footprint --> used for visualization in RViz
            self.broadcaster.sendTransform((self.table_pose.position.x, self.table_pose.position.y, self.table_pose.position.z),
                                            (self.table_pose.orientation.x, self.table_pose.orientation.y, self.table_pose.orientation.z,
                                            self.table_pose.orientation.w),
                                            rospy.Time.now(), "table", self.frame_id)
    def deleteEllipsoid(self, ellipsoid_type='reaching'):
        if ellipsoid_type == 'reaching':
            id = 1000
            pose = self.reaching_ellipsoid_origin
            scale = Vector3(self.reaching_ellipsoid_size_x, self.reaching_ellipsoid_size_y, self.reaching_ellipsoid_size_z)
        elif ellipsoid_type == 'collision':
            id = 999
            pose = self.collision_ellipsoid_origin
            scale = Vector3(self.collision_ellipsoid_size_x, self.collision_ellipsoid_size_y, self.collision_ellipsoid_size_z)

        color=ColorRGBA(r=0, g=1, b=0, a=0.5)

        cube = Marker(header=Header(stamp=rospy.Time.now(),
                                        frame_id=self.frame_id),
                                        pose=pose,
                                        type=Marker.SPHERE,
                                        color=color,
                                        id=id,
                                        scale=Vector3(self.reaching_ellipsoid_size_x, self.reaching_ellipsoid_size_y, self.reaching_ellipsoid_size_z),
                                        action=Marker.DELETE)

        self.models[id] = cube

    def addEllipsoid(self, ellipsoid_type='collision'):

        if ellipsoid_type == 'collision':
            id = 999

            color=ColorRGBA(r=0, g=1, b=0, a=0.5)

            cube = Marker(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.frame_id),
                                            pose=self.collision_ellipsoid_origin,
                                            type=Marker.SPHERE,
                                            color=color,
                                            id=id,
                                            scale=Vector3(self.collision_ellipsoid_size_x, self.collision_ellipsoid_size_y, self.collision_ellipsoid_size_z),
                                            action=Marker.ADD)

            self.models[id] = cube

            # self.deleteEllipsoid(ellipsoid_type='reaching')

        elif ellipsoid_type == 'reaching':
            id = 1000

            color=ColorRGBA(r=1, g=0, b=1, a=1)

            cube = Marker(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.frame_id),
                                            pose=self.reaching_ellipsoid_origin,
                                            type=Marker.SPHERE,
                                            color=color,
                                            id=id,
                                            scale=Vector3(self.reaching_ellipsoid_size_x, self.reaching_ellipsoid_size_y, self.reaching_ellipsoid_size_z),
                                            action=Marker.ADD)

            self.models[id] = cube
            # self.deleteEllipsoid(ellipsoid_type='collision')
        
        elif ellipsoid_type == 'all':
            id = 999
            
            color=ColorRGBA(r=0, g=1, b=0, a=0.5)

            cube = Marker(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.frame_id),
                                            pose=self.collision_ellipsoid_origin,
                                            type=Marker.SPHERE,
                                            color=color,
                                            id=id,
                                            scale=Vector3(self.collision_ellipsoid_size_x, self.collision_ellipsoid_size_y, self.collision_ellipsoid_size_z),
                                            action=Marker.ADD)

            self.models[id] = cube

            id = 1000

            color=ColorRGBA(r=1, g=0, b=1, a=1)

            cube = Marker(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.frame_id),
                                            pose=self.reaching_ellipsoid_origin,
                                            type=Marker.SPHERE,
                                            color=color,
                                            id=id,
                                            scale=Vector3(self.reaching_ellipsoid_size_x, self.reaching_ellipsoid_size_y, self.reaching_ellipsoid_size_z),
                                            action=Marker.ADD)

            self.models[id] = cube
            
    def addCube(self, x, y, z, pose, color, id):
        if color == 'red':
            color=ColorRGBA(r=1, g=0, b=0, a=0.5)
        elif color == 'blue':
            color=ColorRGBA(r=0, g=0, b=1, a=0.5)

        cube = Marker(header=Header(stamp=rospy.Time.now(),
                                        frame_id=self.frame_id),
                                        pose=pose,
                                        type=Marker.CUBE,
                                        color=color,
                                        id=id,
                                        scale=Vector3(x, y, z),
                                        action=Marker.ADD)
        
        self.models[id] = cube

    def visualizeModels(self):
        model_list = []
        for id in self.models:
            model_list.append(self.models[id])

        self.model_pub.publish(model_list)
    
    def run(self):
        r = rospy.Rate(30)
        # self.setEllipsoidSize(ellipsoid_type='all')
        self.setEllipsoidSize(ellipsoid_type='reaching')
        
        while not rospy.is_shutdown():
            # self.setEllipsoidOrigin(ellipsoid_type='all')
            self.setEllipsoidOrigin(ellipsoid_type='reaching')

            # self.addEllipsoid(ellipsoid_type='all')
            self.addEllipsoid(ellipsoid_type='reaching')

            self.visualizeModels()
            self.broadcastFrames(ellipsoid_type='reaching')
            
            r.sleep()

if __name__ == "__main__":
    scene_drawer = SceneDrawer()
    scene_drawer.run()