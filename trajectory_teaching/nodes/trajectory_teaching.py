#!/usr/bin/env python

import rospy, tf
from geomagic_touch_m.msg import GeomagicButtonEvent
from slave_control.msg import ControlState
from os import listdir
from os.path import isfile, join
import os, stat, time
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from scipy.interpolate import interp1d

from trajectory_parser.trajectory_parser import *
from aruco_msgs.msg import MarkerArray
from pyquaternion import Quaternion

import cProfile
from profilehooks import profile

class trajectoryTeaching():
    def __init__(self):
        rospy.init_node("trajectory_teaching")

        self.grey_button = 0
        self.grey_button_previous = 0
        self.white_button = 0
        self.white_button_previous = 0
        self.grey_button_toggle = 0
        self.grey_button_toggle_previous = 0
        self.white_button_toggle = 0
        self.white_button_toggle_previous = 0

        self.EEtrajectory = []

        self.end_effector_pose_sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self._end_effector_pose_callback)
        self.slave_control_state_sub = rospy.Subscriber("slave_control_state", ControlState, self._slave_control_state_callback)

        self.geo_button_sub = rospy.Subscriber("geo_buttons_m", GeomagicButtonEvent, self._buttonCallback)
        self.end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)
        self.marker_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self._marker_detection_callback)
        self.parser = trajectoryParser()

        self.set_marker_pose_counter = 0
        self.object_marker_pose_continuous = Pose()

        # master pose that is used for calculations of the robot arm wrt object
        self.marker_pose_static = Pose()
    
    def _marker_detection_callback(self, data):
        # print("marker pose = " + str(self.marker_pose_static.position))
        for marker in data.markers:
            if marker.id == 582:
                self.object_marker_pose_continuous = marker.pose.pose
            else: continue

    def set_marker_pose(self):
        self.marker_pose_static = self.object_marker_pose_continuous
        
    def _end_effector_pose_callback(self,data):
        self.current_slave_pose = data.pose

    # @profile
    def ee_pose_wrt_object(self, ee_pose):
        # print(ee_pose)
        # print(self.object_marker_pose_continuous)
        ee_pos_wrt_base = ([ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z])
        ee_orient_wrt_base = ([ee_pose.pose.orientation.w, ee_pose.pose.orientation.x, ee_pose.pose.orientation.y, ee_pose.pose.orientation.z])
        # marker_pos_wrt_base = np.asarray([self.marker_pose_static.position.x, self.marker_pose_static.position.y, self.marker_pose_static.position.z])
        # marker_orient_wrt_base = np.asarray([self.marker_pose_static.orientation.x, self.marker_pose_static.orientation.y, self.marker_pose_static.orientation.z, self.marker_pose_static.orientation.w])
        marker_pos_wrt_base = ([self.object_marker_pose_continuous.position.x, self.object_marker_pose_continuous.position.y, self.object_marker_pose_continuous.position.z])
        marker_orient_wrt_base = ([self.object_marker_pose_continuous.orientation.w, self.object_marker_pose_continuous.orientation.x, self.object_marker_pose_continuous.orientation.y, self.object_marker_pose_continuous.orientation.z])

        print(np.subtract(ee_pos_wrt_base, marker_pos_wrt_base))
        # print('ee: ' + str(ee_pos_wrt_base) + '\n')
        # print('marker: ' + str(marker_pos_wrt_base) + '\n')

        # qdelta = qtarget * qcurrent^-1
        q_marker = Quaternion(marker_orient_wrt_base)
        q_ee = Quaternion(ee_orient_wrt_base)
        # marker_orient_wrt_base_inv = [marker_orient_wrt_base[0], marker_orient_wrt_base[1], marker_orient_wrt_base[2], marker_orient_wrt_base[3]]
        # ee_orient_wrt_base_inv = [ee_orient_wrt_base[0], ee_orient_wrt_base[1], ee_orient_wrt_base[2], -ee_orient_wrt_base[3]]
        # q_ee_wrt_marker = tf.transformations.quaternion_multiply(marker_orient_wrt_base, ee_orient_wrt_base_inv)
        q_ee_wrt_marker = q_marker * q_ee.inverse

        # express r_ee in marker frame instead of base_footprint
        # r_ee_wrt_marker = q_ee_wrt_marker.rotate(np.subtract(ee_pos_wrt_base, marker_pos_wrt_base))
        p = list(np.subtract(ee_pos_wrt_base, marker_pos_wrt_base))
        # p.append(0.0)
        # r_ee_wrt_marker = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q_ee_wrt_marker, p), tf.transformations.quaternion_inverse(q_ee_wrt_marker))
        r_ee_wrt_marker = q_ee_wrt_marker.rotate(p)

        # print(tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q_ee_wrt_marker, p), tf.transformations.quaternion_conjugate(q_ee_wrt_marker)))
        # print(r_ee_wrt_marker)

        new_pose = PoseStamped()
        new_pose.pose.position.x = r_ee_wrt_marker[0] 
        new_pose.pose.position.y = r_ee_wrt_marker[1] 
        new_pose.pose.position.z = r_ee_wrt_marker[2] 
        new_pose.pose.orientation.x = q_ee_wrt_marker[0]
        new_pose.pose.orientation.y = q_ee_wrt_marker[1]
        new_pose.pose.orientation.z = q_ee_wrt_marker[2]
        new_pose.pose.orientation.w = q_ee_wrt_marker[3]
        new_pose.header.stamp = ee_pose.header.stamp
        new_pose.header.frame_id = 'marker'

        # print(new_pose)

        return new_pose
        
    def goToInitialPose(self):
        rospy.loginfo("Moving to initial pose")
        rospy.wait_for_message('/end_effector_pose', PoseStamped)
        T = 2
        x = [self.current_slave_pose.position.x, 0.403399335619]
        y = [self.current_slave_pose.position.y, -0.430007534239]
        z = [self.current_slave_pose.position.z, 1.16269467394]

        t = [rospy.Time.now(), rospy.Time.now() + rospy.Duration(T)]

        t = list(self.parser._secsNsecsToFloat(self.parser.durationVector2secsNsecsVector(t)))
        
        fx = interp1d(t, x, fill_value="extrapolate")
        fy = interp1d(t, y, fill_value="extrapolate")
        fz = interp1d(t, z, fill_value="extrapolate")

        dt = 0.1
        tnew = np.arange(t[0],t[-1],dt)
        xnew = fx(tnew)
        ynew = fy(tnew)
        znew = fz(tnew)

        for i in range(len(ynew)):
            slave_goal = PoseStamped()

            slave_goal.pose.position.x = xnew[i]
            slave_goal.pose.position.y = ynew[i]
            slave_goal.pose.position.z = znew[i]

            slave_goal.pose.orientation.w = 0.00470101575578
            slave_goal.pose.orientation.x = 0.994781110161
            slave_goal.pose.orientation.y = 0.100705187531
            slave_goal.pose.orientation.z = 0.0157133230571

            slave_goal.header.seq = 0
            slave_goal.header.frame_id = "/base_footprint"

            slave_goal.header.stamp = (rospy.Time.now())
            self.end_effector_goal_pub.publish(slave_goal)
            
            time.sleep(dt)
    
    def _buttonCallback(self, data):
        self.grey_button_previous = self.grey_button 
        self.grey_button = data.grey_button
        self.white_button_previous = self.white_button
        self.white_button = data.white_button

        if (self.grey_button != self.grey_button_previous) and (self.grey_button == 1):
            
            self.grey_button_toggle = not self.grey_button_toggle
            self.grey_button_toggle_previous = not self.grey_button_toggle

        if (self.white_button != self.white_button_previous) and (self.white_button == 1):
            self.white_button_toggle = not self.white_button_toggle
            self.white_button_toggle_previous = not self.white_button_toggle


    def _slave_control_state_callback(self,data):
        
        # if self.marker_pose_static.position.x != 0.0:
        data.slave_pose = self.ee_pose_wrt_object(data.slave_pose)
        # print(data.slave_pose)

        if self.white_button_toggle_previous == 0 and self.white_button_toggle == 1:
            print("Appending trajectory")
            if self.set_marker_pose_counter % 2 == 0:
                self.set_marker_pose()
                self.set_marker_pose_counter += 1
            
            if self.marker_pose_static.position.x != 0.0:
                data.slave_pose = self.ee_pose_wrt_object(data.slave_pose)

            data.slave_pose.header.stamp.secs = data.header.stamp.secs
            data.slave_pose.header.stamp.nsecs = data.header.stamp.nsecs


            # marker x and y seem to be flipped wrt base_footprint
            # therefore first position.y then position.x for marker_pose
            self.EEtrajectory.append([data.slave_pose.pose.position.x,data.slave_pose.pose.position.y,data.slave_pose.pose.position.z,
             data.slave_pose.pose.orientation.x,data.slave_pose.pose.orientation.y,data.slave_pose.pose.orientation.z,data.slave_pose.pose.orientation.w,
             self.marker_pose_static.position.y, self.marker_pose_static.position.x, self.marker_pose_static.position.z,
             self.marker_pose_static.orientation.x, self.marker_pose_static.orientation.y, self.marker_pose_static.orientation.z, self.marker_pose_static.orientation.w, 
             data.header.stamp.secs, data.header.stamp.nsecs])
            
    def _save_data(self, path, file_name):
        with open(path+file_name, 'w+') as f:
            f.write(str(self.EEtrajectory))
        os.chmod(path+file_name,stat.S_IRWXO)
        os.chmod(path+file_name,stat.S_IRWXU)

    def _get_trajectory_file_name(self, path):
        files = [f for f in listdir(path) if isfile(join(path, f))]
        numbers = [int(os.path.splitext(f)[0].split('_')[-1]) for f in files]
        numbers.sort()
        files = ["raw_trajectory_" + str(number) + ".txt" for number in numbers]

        if os.stat(path + files[-1]).st_size == 0:
            return files[-1]
        else:
            return "raw_trajectory_" + str(int(os.path.splitext(files[-1])[0].split('_')[-1]) + 1) + ".txt"

    def run(self):
        # teaching_node.goToInitialPose()
        r = rospy.Rate(30)
        while not rospy.is_shutdown():

            if self.white_button_toggle_previous == 1 and self.white_button_toggle == 0:
                self.set_marker_pose_counter += 1
                print("Saving trajectory data")
                
                # path = "/home/fmeccanici/Documents/thesis/lfd_ws/src/marco_lfd/data/raw/"
                path = "/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_teaching/data/with_object_wrt_ee/"

                print("file_name = " + self._get_trajectory_file_name(path))
                file_name = self._get_trajectory_file_name(path)
                self._save_data(path, file_name)
                del self.EEtrajectory[:]
                teaching_node.goToInitialPose()

                # set to 0 to prevent multiple savings
                self.white_button_toggle_previous = 0
            r.sleep()

if __name__ == "__main__":
    teaching_node = trajectoryTeaching()
    try:
        teaching_node.run()
    except Exception as e: 
        print(e)