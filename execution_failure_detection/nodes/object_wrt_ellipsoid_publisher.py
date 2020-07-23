#!/usr/bin/env python
import rospy, tf, copy
from pyquaternion import Quaternion
from std_msgs.msg import Header, ColorRGBA, Bool
from geometry_msgs.msg import Vector3, Pose, PoseStamped
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Point

class ObjectWrtEllipsoidPublisher(object):
    def __init__(self, model_path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/execution_failure_detection/models/'):
        self.table_pose_counter = 0
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

        self.model_path = model_path
        self.models = {}
        self.model_pub = rospy.Publisher('execution_failure_detection/model_visualization', MarkerArray, queue_size=10)

        self.ee_sub = rospy.Subscriber('end_effector_pose', PoseStamped, self._endEffectorPoseCallback)
        self.link_states_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.linkStatesCallback)
        self.broadcaster = tf.TransformBroadcaster()

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
            r_ellipsoid_wrt_ee = [0.02, 0, -0.01]
            
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

    def run(self):
        r = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            self.setEllipsoidOrigin(ellipsoid_type='all')
            self.broadcaster.sendTransform((self.reaching_ellipsoid_origin.position.x, self.reaching_ellipsoid_origin.position.y, self.reaching_ellipsoid_origin.position.z),
                                            (self.ee_pose.orientation.x, self.ee_pose.orientation.y, self.ee_pose.orientation.z, self.ee_pose.orientation.w),
                                            rospy.Time.now(), "reaching_ellipsoid", "base_footprint")
            
            r.sleep()