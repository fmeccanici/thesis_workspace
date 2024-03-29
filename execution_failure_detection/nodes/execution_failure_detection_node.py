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

import matplotlib.pyplot as plt

class ExecutionFailureNode(object):
    def __init__(self, model_path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/execution_failure_detection/models/'):
        rospy.init_node('execution_failure_detection_node')

        self.table_pose_counter = 0
        self.object_pose_wrt_reaching_ellipsoid = Pose()
        self.table_pose_wrt_collision_ellipsoid = Pose()
        self.table_pose_wrt_base_footprint = Pose()
        self.object_pose_wrt_base_footprint = Pose()
        self.collision_ellipsoid_wrt_base_footprint = Pose()
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
            (trans,rot) = self.listener.lookupTransform('/reaching_ellipsoid', '/object', rospy.Time(0))

            self.object_pose_wrt_reaching_ellipsoid.position.x = trans[0]
            self.object_pose_wrt_reaching_ellipsoid.position.y = trans[1]
            self.object_pose_wrt_reaching_ellipsoid.position.z = trans[2]

            (trans,rot) = self.listener.lookupTransform('/object', '/base_footprint', rospy.Time(0))
            self.object_pose_wrt_base_footprint.position.x = trans[0]
            self.object_pose_wrt_base_footprint.position.y = trans[1]
            self.object_pose_wrt_base_footprint.position.z = trans[2]

            # publish collision ellipsoid frame wrt base footprint --> used for visualization in RViz
            self.broadcaster.sendTransform((self.object_pose_wrt_reaching_ellipsoid.position.x, self.object_pose_wrt_reaching_ellipsoid.position.y, self.object_pose_wrt_reaching_ellipsoid.position.z),
                                            (q_ee.inverse.x, q_ee.inverse.y, q_ee.inverse.z, q_ee.inverse.w),
                                            rospy.Time.now(), "object_wrt_reaching_ellipsoid", "reaching_ellipsoid")

            (trans,rot) = self.listener.lookupTransform('/collision_ellipsoid', '/table', rospy.Time(0))
            self.table_pose_wrt_collision_ellipsoid.position.x = trans[0]
            self.table_pose_wrt_collision_ellipsoid.position.y = trans[1]
            self.table_pose_wrt_collision_ellipsoid.position.z = trans[2]
            
            # for some reason it is the other way around in tf, collision ellipsoid expressed in base footprint
            (trans,rot) = self.listener.lookupTransform('/base_footprint', '/collision_ellipsoid', rospy.Time(0))
            self.collision_ellipsoid_wrt_base_footprint.position.x = trans[0]
            self.collision_ellipsoid_wrt_base_footprint.position.y = trans[1]
            self.collision_ellipsoid_wrt_base_footprint.position.z = trans[2]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        # (trans,rot) = self.listener.lookupTransform('/base_footprint', '/table', rospy.Time(0))
        # self.table_pose_wrt_base_footprint.position.x = trans[0]
        # self.table_pose_wrt_base_footprint.position.y = trans[1]
        # self.table_pose_wrt_base_footprint.position.z = trans[2]

        # self.broadcaster.sendTransform((self.table_pose_wrt_collision_ellipsoid.position.x, self.table_pose_wrt_collision_ellipsoid.position.y, self.table_pose_wrt_collision_ellipsoid.position.z),
        #                             (q_ee.inverse.x, q_ee.inverse.y, q_ee.inverse.z, q_ee.inverse.w),
        #                             rospy.Time.now(), "table_wrt_collision_ellipsoid", "collision_ellipsoid")

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
        q_ee = Quaternion(copy.deepcopy([self.ee_pose.orientation.w, self.ee_pose.orientation.x, self.ee_pose.orientation.y, self.ee_pose.orientation.z]))

        vec_min_wrt_base_footprint = [-self.table_size_x/2, self.table_size_y/2, self.table_size_z/2]
        vec_max_wrt_base_footprint = [self.table_size_x/2, -self.table_size_y/2, self.table_size_z/2]
        x = np.linspace(vec_min_wrt_base_footprint[0], vec_max_wrt_base_footprint[0], 10)
        y = np.linspace(vec_min_wrt_base_footprint[1], vec_max_wrt_base_footprint[1], 10)
        z = vec_min_wrt_base_footprint[2]
        for i in range(len(x)):
            for j in range(len(y)):
                try:
                    (trans,rot) = self.listener.lookupTransform('collision_ellipsoid', "table_" + str(i) + "_" + str(j), rospy.Time(0))
                    self.broadcaster.sendTransform((trans[0], trans[1], trans[2]),
                                    (q_ee.inverse.x, q_ee.inverse.y, q_ee.inverse.z,
                                    q_ee.inverse.w),
                                    rospy.Time.now(), "table_eval", "collision_ellipsoid")
                    if self.isInsideEllipsoid(trans[0],trans[1],trans[2],'collision'): 
                        # input('continue?')  

                        return True

                except:
                    continue

        return False
    
    def isObjectKickedOver(self):
        # print(round(abs(self.object_pose_wrt_base_footprint.position.x) + self.object_size_x/2, 2 ))
        # print(round(self.expected_object_pose.position.x, 2))
        # print('\n')
        # print(abs(round(self.object_pose_wrt_base_footprint.position.y, 2)))
        # print(round(self.expected_object_pose.position.y, 2))
        # print('\n')
        
        # account for difference in pose of centre and pose of frontal plane for evakluation
        # round(abs(self.object_pose_wrt_base_footprint.position.x) + self.object_size_x/2, 2 )
        # thats this + self.object_size / 2
        
        
        if round(abs(self.object_pose_wrt_base_footprint.position.x) + self.object_size_x/2, 2 ) != round(self.expected_object_pose.position.x, 2) or abs(round(self.object_pose_wrt_base_footprint.position.y, 2)) != round(self.expected_object_pose.position.y, 2):
            return True
        # if abs(round(self.object_pose_wrt_base_footprint.position.x, 2)) != round(self.expected_object_pose.position.x, 2) or abs(round(self.object_pose_wrt_base_footprint.position.y, 2)) != round(self.expected_object_pose.position.y, 2):
        #     return True

        return False

    def isObjectReached(self):
        # used for calculations
        object_wrt_ellipsoid = [self.object_pose_wrt_reaching_ellipsoid.position.x, self.object_pose_wrt_reaching_ellipsoid.position.y , self.object_pose_wrt_reaching_ellipsoid.position.z]
        object_wrt_ellipsoid[0] = -object_wrt_ellipsoid[0]
        object_wrt_ellipsoid[1] = -object_wrt_ellipsoid[1]
        object_wrt_ellipsoid[2] = -object_wrt_ellipsoid[2]

        x = object_wrt_ellipsoid[0]
        y = object_wrt_ellipsoid[1]
        z = object_wrt_ellipsoid[2]

        if self.isInsideEllipsoid(x,y,z,ellipsoid_type='reaching'):
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
        rospy.wait_for_message('end_effector_pose', PoseStamped)
        q_ee = Quaternion(copy.deepcopy([self.ee_pose.orientation.w, self.ee_pose.orientation.x, self.ee_pose.orientation.y, self.ee_pose.orientation.z]))
            
        if ellipsoid_type == 'collision':
            a = self.collision_ellipsoid_size_x/2
            b = self.collision_ellipsoid_size_y/2
            c = self.collision_ellipsoid_size_z/2
        
            
            f = (x / a)**2 + (y / b)**2 + (z / c)**2 

        elif ellipsoid_type == 'reaching':

            a = self.reaching_ellipsoid_size_x/2
            b = self.reaching_ellipsoid_size_y/2
            c = self.reaching_ellipsoid_size_z/2

            """
            try:
                # bereik van functie
                x_plot = np.linspace(-a * np.sqrt(1 - (y/b)**2), a * np.sqrt(1 - (y/b)**2), 100)

                o = [self.object_pose_wrt_reaching_ellipsoid.position.x, self.object_pose_wrt_reaching_ellipsoid.position.y, self.object_pose_wrt_reaching_ellipsoid.position.z]
                # o = q_ee.rotate(object_wrt_ellipsoid)
                o_x = np.linspace(0, o[0], 1000)
                o_z = np.linspace(0, o[2], 1000)
            
                print("x = " + str(x))
                print("y = " + str(y))
                print("a = " + str(a))
                print("b = " + str(b))

                # # y = map(lambda z: a*np.sqrt(1 - (y/b)**2) + (z/c)**2, z) 
                z_1 = map(lambda x: c*np.sqrt(1 - (y/b)**2 - (x/a)**2), x_plot) 
                z_2 = map(lambda x: -c*np.sqrt(1 - (y/b)**2 - (x/a)**2), x_plot) 

                print(z_1)
                # print(o_x)
                # print(o_z)
                # plt.plot(z_1, x, '-b')
                # plt.plot(z_2, x, '-b')



                plt.plot(x_plot, z_1, '-b')
                plt.plot(x_plot, z_2, '-b')
                plt.grid()
                plt.ylim([-0.2,0.2])
                plt.xlim([-0.2,0.2])

                # print(z_1)
                # plt.plot(x2, z_1, '-b')
                plt.plot(o_x, o_z, '-r')

                plt.show()
                pass
            except Exception as e:
                print(e)
                pass
                """

        # ellipsoid equation
        f = (x / a)**2 + (y / b)**2 + (z / c)**2 

        return f < 1
        
    def run(self):
        r = rospy.Rate(30)
        self.getEllipsoidSize('all')

        while not rospy.is_shutdown():
            execution_failure_msg = ExecutionFailure()
            # print('\n')
            # print("object reached = " + str(self.isObjectReached()))
            # print("obstacle hit = " + str(self.isObstacleHit()))
            # print("object kicked over = " + str(self.isObjectKickedOver()))
            # print('\n')

            # make ros message
            execution_failure_msg.object_reached = Bool(self.isObjectReached())
            execution_failure_msg.object_kicked_over = Bool(self.isObjectKickedOver())
            execution_failure_msg.obstacle_hit = Bool(self.isObstacleHit())

            # publish ros message
            self.execution_failure_pub.publish(execution_failure_msg)

            r.sleep()

if __name__ == "__main__":
    execution_failure_node = ExecutionFailureNode()
    execution_failure_node.run()