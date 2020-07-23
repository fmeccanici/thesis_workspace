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

class ExecutionFailureNode(object):
    def __init__(self, model_path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/execution_failure_detection/models/'):
        rospy.init_node('execution_failure_detection_node')

        self.table_pose_counter = 0
        self.object_pose_wrt_reaching_ellipsoid = Pose()
        self.object_pose_wrt_base_footprint = Pose()

        self.table_pose = Pose()
        
        # set object/table sizes 

        ##### true size ######
        self.table_size_x = 0.8
        self.table_size_y = 1.5
        self.table_size_z = 0.66

        ##### size adapted for collision ######
        # x = 0.8
        # y = 1.5
        # z = 0.7

        self.object_size_x = 0.05
        self.object_size_y = 0.05
        self.object_size_z = 0.1

        self.frame_id = 'base_footprint'

        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.ee_sub = rospy.Subscriber('end_effector_pose', PoseStamped, self._endEffectorPoseCallback)

        self.execution_failure_service = rospy.Service('get_execution_failure', GetExecutionFailure, self._getExecutionFailure)
        self.set_expected_object_position_service = rospy.Service('set_expected_object_position', SetExpectedObjectPosition, self._setExpectedObjectPosition)
        self.expected_object_pose = Pose()
        self.execution_failure_pub = rospy.Publisher('execution_failure', ExecutionFailure, queue_size=10)


    def _endEffectorPoseCallback(self, data):
        self.ee_pose = data.pose
        q_ee = Quaternion(copy.deepcopy([self.ee_pose.orientation.w, self.ee_pose.orientation.x, self.ee_pose.orientation.y, self.ee_pose.orientation.z]))
        
        try:
            # self.listener.waitForTransform("/object", "/reaching_ellipsoid",
            #                 rospy.Time.now(), rospy.Duration(1.0))
            (trans,rot) = self.listener.lookupTransform('/reaching_ellipsoid', '/object', rospy.Time(0))
            self.object_pose_wrt_reaching_ellipsoid.position.x = trans[0]
            self.object_pose_wrt_reaching_ellipsoid.position.y = trans[1]
            self.object_pose_wrt_reaching_ellipsoid.position.z = trans[2]

            
            # self.listener.waitForTransform("/object", "/base_footprint",
            #                 rospy.Time.now(), rospy.Duration(1.0))
            
            (trans,rot) = self.listener.lookupTransform('/object', '/base_footprint', rospy.Time(0))
            self.object_pose_wrt_base_footprint.position.x = trans[0]
            self.object_pose_wrt_base_footprint.position.y = trans[1]
            self.object_pose_wrt_base_footprint.position.z = trans[2]


            # publish collision ellipsoid frame wrt base footprint --> used for visualization in RViz
            self.broadcaster.sendTransform((self.object_pose_wrt_reaching_ellipsoid.position.x, self.object_pose_wrt_reaching_ellipsoid.position.y, self.object_pose_wrt_reaching_ellipsoid.position.z),
                                            (q_ee.inverse.x, q_ee.inverse.y, q_ee.inverse.z, q_ee.inverse.w),
                                            rospy.Time.now(), "object_wrt_reaching_ellipsoid", "reaching_ellipsoid")
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def _getExecutionFailure(self, req):
        resp = GetExecutionFailureResponse()
        object_reached = Bool()
        obstacle_hit = Bool()
        object_reached.data = self.isObjectReached()
        obstacle_hit.data = self.isObstacleHit()
        object_kicked_over = Bool(self.isObjectKickedOver())
        
        resp.object_reached = object_reached
        resp.obstacle_hit = obstacle_hit
        resp.object_kicked_over = object_kicked_over

        return resp

    def _setExpectedObjectPosition(self, req):
        self.expected_object_pose = req.expected_object_pose

        resp = SetExpectedObjectPositionResponse()

        return resp

    def isObstacleHit(self):
        # z = self.table_pose.position.z
        # seems to be an offset in the z direction 
        # needed to properly detect if it is hitting the table
        z = 0.59
        xmin, xmax = self.table_pose.position.x - self.table_size_x/2, self.table_pose.position.x + self.table_size_x/2
        ymin, ymax = self.table_pose.position.y - self.table_size_y/2, self.table_pose.position.y + self.table_size_y/2
        
        for x in np.arange(xmin, xmax, 0.1):
            for y in np.arange(ymin, ymax, 0.1):
                if self.isInsideEllipsoid(x, y, z, ellipsoid_type='collision'):
                    return True
        # if self.isInsideEllipsoid(xmax,ymax,z,'collision'): return True

        return False
    
    def isObjectKickedOver(self):
        # print(round(self.object_pose.position.x, 2))
        # print(round(self.expected_object_pose.position.x, 2))
        # print('\n')
        # print(round(self.object_pose.position.y, 2))
        # print(round(self.expected_object_pose.position.y, 2))
        # print('\n')

        
        
        if round(self.object_pose.position.x, 2) != round(self.expected_object_pose.position.x, 2) or round(self.object_pose.position.y, 2) != round(self.expected_object_pose.position.y, 2):
            return True
        
        return False

    def isObjectReached(self):
        x = self.object_pose_wrt_base_footprint.position.x - self.object_size_x/2
        ymin, ymax = -self.object_size_y/2, self.object_size_y/2
        zmin, zmax = -self.object_size_z/2, self.object_size_z/2

        # for y in np.arange(ymin, ymax, 0.01):
        #     for z in np.arange(zmin, zmax, 0.01):
        #         if self.isInsideEllipsoid(x, y, z, ellipsoid_type='reaching'):
        #             # r_eval_wrt_base = [x - self.collision_ellipsoid_origin.position.x, y - self.collision_ellipsoid_origin.position.y, z - self.collision_ellipsoid_origin.position.z]
        #             # print(r_eval_wrt_base)
        #             # print('\n')
        #             return True
        ymin = self.object_pose_wrt_base_footprint.position.y
        zmin = self.object_pose_wrt_base_footprint.position.z
        x = self.object_pose_wrt_base_footprint.position.x 

        if self.isInsideEllipsoid(x,ymin,zmin,ellipsoid_type='reaching'):
            return True

        return False    

    # set size of ellipsoid wrt base frame
    def getEllipsoidSize(self, ellipsoid_type='collision'):
        try:
            if ellipsoid_type == 'collision':
                size_wrt_base = [0.4, 0.2, 0.1]
                self.collision_ellipsoid_size_x = size_wrt_base[0]
                self.collision_ellipsoid_size_y = size_wrt_base[1]
                self.collision_ellipsoid_size_z = size_wrt_base[2] # 0.1 is best
                

            elif ellipsoid_type == 'reaching':
                size_wrt_base = [0.075, 0.2, 0.1]

                self.reaching_ellipsoid_size_x = size_wrt_base[0]
                self.reaching_ellipsoid_size_y = size_wrt_base[1]
                self.reaching_ellipsoid_size_z = size_wrt_base[2] 
            

            elif ellipsoid_type == 'all':
                size_wrt_base = [0.4, 0.2, 0.1]
                self.collision_ellipsoid_size_x = size_wrt_base[0]
                self.collision_ellipsoid_size_y = size_wrt_base[1]
                self.collision_ellipsoid_size_z = size_wrt_base[2] # 0.1 is best
                
                size_wrt_base = [0.075, 0.2, 0.1]

                self.reaching_ellipsoid_size_x = size_wrt_base[0]
                self.reaching_ellipsoid_size_y = size_wrt_base[1]
                self.reaching_ellipsoid_size_z = size_wrt_base[2] 
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def isInsideEllipsoid(self, x, y, z, ellipsoid_type='collision'):

        if ellipsoid_type == 'collision':

            f = (r_eval_wrt_ellipsoid[0] / self.collision_ellipsoid_size_x)**2 + (r_eval_wrt_ellipsoid[1] / self.collision_ellipsoid_size_y)**2 + (r_eval_wrt_ellipsoid[2] / self.collision_ellipsoid_size_z)**2 

        elif ellipsoid_type == 'reaching':

            # object_wrt_ellipsoid = [self.object_pose_wrt_reaching_ellipsoid.position.x + x, self.object_pose_wrt_reaching_ellipsoid.position.y + y, self.object_pose_wrt_reaching_ellipsoid.position.z + z]
            object_wrt_ellipsoid = [self.object_pose_wrt_reaching_ellipsoid.position.x , self.object_pose_wrt_reaching_ellipsoid.position.y , self.object_pose_wrt_reaching_ellipsoid.position.z]

            rospy.wait_for_message('end_effector_pose', PoseStamped)
            q_ee = Quaternion(copy.deepcopy([self.ee_pose.orientation.w, self.ee_pose.orientation.x, self.ee_pose.orientation.y, self.ee_pose.orientation.z]))
            size = q_ee.rotate([self.reaching_ellipsoid_size_x, self.reaching_ellipsoid_size_y, self.reaching_ellipsoid_size_z])

            # f = (object_wrt_ellipsoid[0] / self.reaching_ellipsoid_size_x)**2 + (object_wrt_ellipsoid[1] / self.reaching_ellipsoid_size_y)**2 + (object_wrt_ellipsoid[2] / self.reaching_ellipsoid_size_z)**2 
            f = (object_wrt_ellipsoid[0] / size[0])**2 + (object_wrt_ellipsoid[1] / size[1])**2 + (object_wrt_ellipsoid[2] / size[2])**2 


        return f < 1

    def run(self):
        r = rospy.Rate(30)
        self.getEllipsoidSize('reaching')

        while not rospy.is_shutdown():
            # execution_failure_msg = ExecutionFailure()
            # print('\n')
            print("object reached = " + str(self.isObjectReached()))
            # print("obstacle hit = " + str(self.isObstacleHit()))
            # print('\n')
            # execution_failure_msg.object_reached = Bool(self.isObjectReached())
            # execution_failure_msg.object_kicked_over = Bool(self.isObjectKickedOver())

            # execution_failure_msg.obstacle_hit = Bool(self.isObstacleHit())


            # self.execution_failure_pub.publish(execution_failure_msg)

            r.sleep()

if __name__ == "__main__":
    execution_failure_node = ExecutionFailureNode()
    execution_failure_node.run()