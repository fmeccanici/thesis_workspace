#!/usr/bin/env python2.7

from learning_from_demonstration.learning_from_demonstration import *

# import ros related
import rospy, time, tf, os

import thread

import numpy as np

from spline_interpolation.spline_interpolation import *

from geomagic_touch_m.msg import GeomagicButtonEvent

from scipy.interpolate import interp1d
from geometry_msgs.msg import PoseStamped, WrenchStamped, PoseArray, Pose

from lfd_msgs.msg import TrajectoryVisualization

from pyquaternion import Quaternion 
from scipy.spatial.transform import Rotation as R

from promp_python.promp_python import *
from aruco_msgs.msg import MarkerArray

from scipy.spatial.distance import euclidean
from dynamic_time_warping.dynamic_time_warping import *

import matplotlib.pyplot as plt
from pyquaternion import Quaternion

class trajectoryStorageVariables():
    def __init__(self, open_loop_path, refined_path, resampled_path, raw_path, raw_file, new_path):
        self.open_loop_path = open_loop_path
        self.refined_path = refined_path
        self.resampled_path = resampled_path
        self.raw_path = raw_path
        self.raw_file = raw_file
        self.new_path = new_path

class trajectoryRefinement():
    def __init__(self):
        rospy.init_node("refinement_node")
        self.rospack = rospkg.RosPack()

        # get rosparameters specified in launch
        self._get_parameters()

        # initialize pub/sub
        self.end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)
        self.traj_vis_pub = rospy.Publisher('trajectory_visualizer/trajectory', TrajectoryVisualization, queue_size=10)

        if self.button_source == "omni":
            self.geo_button_sub = rospy.Subscriber("geo_buttons_m", GeomagicButtonEvent, self._buttonCallback)
        elif self.button_source == "keyboard":
            self.geo_button_sub = rospy.Subscriber("keyboard", GeomagicButtonEvent, self._buttonCallbackToggle)


        self.end_effector_pose_sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self._end_effector_pose_callback)
        self.marker_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self._marker_detection_callback)
        self.master_pose_sub = rospy.Subscriber('master_control_comm', ControlComm, self._masterPoseCallback)
        
        # initialize other classes
        # self.visualizer = trajectoryVisualizer()
        self.parser = trajectoryParser()
        self.resampler = trajectoryResampler()

        # initialize button parameters
        self.grey_button = 0
        self.grey_button_prev = 0
        self.grey_button_toggle = 0

        self.white_button = 0
        self.white_button_prev = 0
        self.white_button_toggle = 0

        # initialize other parameters
        self.frame_id = '/base_footprint'
        self.object_marker_pose = Pose()
        self.gripper_wrt_ee = Pose()
    
        # calibrate master pose
        self.initMasterNormalizePose()


    def _get_parameters(self):
        self.button_source = rospy.get_param('~button_source')
        print("Button source set to: " + str(self.button_source))

    def _marker_detection_callback(self, data):
        self.object_marker_pose = data.pose

    def _marker_detection_callback(self, data):
        # rospy.loginfo("marker pose = " + str(self.object_marker_pose.position))
        for marker in data.markers:
            if marker.id == 582:
                # flip x and y as in training data
                self.object_marker_pose.position.x = marker.pose.pose.position.y
                self.object_marker_pose.position.y = marker.pose.pose.position.x
                self.object_marker_pose.position.z = marker.pose.pose.position.z

                self.object_marker_pose.orientation.x = marker.pose.pose.orientation.x
                self.object_marker_pose.orientation.y = marker.pose.pose.orientation.y
                self.object_marker_pose.orientation.z = marker.pose.pose.orientation.z
                self.object_marker_pose.orientation.w = marker.pose.pose.orientation.w

            else: continue


    def _masterPoseCallback(self, data):
        self.master_pose = data.master_pose.pose        

    def _buttonCallback(self, data):
        self.grey_button_prev = self.grey_button
        self.grey_button = data.grey_button
        self.white_button_prev = self.white_button
        self.white_button = data.white_button

        if (self.grey_button != self.grey_button_prev) and (self.grey_button == 1):
            self.grey_button_toggle = not self.grey_button_toggle

        if (self.white_button != self.white_button_prev) and (self.white_button == 1):
            self.white_button_toggle = not self.white_button_toggle

    def _end_effector_pose_callback(self,data):
        self.current_slave_pose = data.pose

    def isCoupled(self):
        if self.grey_button == 1:
            return True
        else: return False

    def calibrate_master_pose_for_normalization(self):
        self.firstMasterPose = PoseStamped()
        self.firstMasterPose.pose.position.x = self.master_pose.position.x
        self.firstMasterPose.pose.position.y = self.master_pose.position.y

        self.firstMasterPose.pose.position.z = self.master_pose.position.z
        
        self.firstMasterPose.pose.orientation.x = self.master_pose.orientation.x
        self.firstMasterPose.pose.orientation.y = self.master_pose.orientation.y
        self.firstMasterPose.pose.orientation.z = self.master_pose.orientation.z
        self.firstMasterPose.pose.orientation.w = self.master_pose.orientation.w

    def initMasterNormalizePose(self):
        self.firstMasterPose = PoseStamped()
        self.firstMasterPose.pose.position.x = 0.412058425026
        self.firstMasterPose.pose.position.y = -0.00570407599211

        # for some reason the minus sign disappeared or ee axis is flipped 180deg
        # self.firstMasterPose.pose.position.z = -0.486718259108
        # for some reason this value changed suddenly ?? -> This is why scaling was wrong of refined trajectory
        # self.firstMasterPose.pose.position.z = -0.169880290808
        # self.firstMasterPose.pose.position.z = -0.327476944265
        # self.firstMasterPose.pose.position.z = -0.510984332781
        self.firstMasterPose.pose.position.z = -0.457386247644
        
        self.firstMasterPose.pose.orientation.x = 0.97947135287
        self.firstMasterPose.pose.orientation.y = 0.0146418957782
        self.firstMasterPose.pose.orientation.z = -0.172556127181
        self.firstMasterPose.pose.orientation.w = -0.103178866925

    def normalizeMasterPose(self, pose):
        normalized_pose = PoseStamped()
        normalized_pose.pose.position.x = pose.position.x - self.firstMasterPose.pose.position.x
        normalized_pose.pose.position.y = pose.position.y - self.firstMasterPose.pose.position.y
        normalized_pose.pose.position.z = pose.position.z - self.firstMasterPose.pose.position.z
        
        # do not normalize the orientation
        normalized_pose.pose.orientation = pose.orientation

        return normalized_pose

    @classmethod
    def reverseTrajectory(cls, traj):

        return traj[::-1]

    def executeTrajectory(self, traj, dt):
        rospy.loginfo("Executing trajectory...")
        slave_goal = PoseStamped()
        for datapoint in traj:
            slave_goal.pose.position.x = datapoint[0]
            slave_goal.pose.position.y = datapoint[1]
            slave_goal.pose.position.z = datapoint[2]

            slave_goal.pose.orientation.x = datapoint[3]
            slave_goal.pose.orientation.y = datapoint[4]
            slave_goal.pose.orientation.z = datapoint[5]
            slave_goal.pose.orientation.w = datapoint[6]

            slave_goal.header.seq = 0
            slave_goal.header.frame_id = "/base_footprint"

            slave_goal.header.stamp = (rospy.Time.now())
            self.end_effector_goal_pub.publish(slave_goal)
            
            time.sleep(dt)

    def goToInitialPose(self):
        rospy.loginfo("Moving to initial pose")
        rospy.wait_for_message('/end_effector_pose', PoseStamped)

        T = 2
        x = [self.current_slave_pose.position.x, 0.401946359213]
        y = [self.current_slave_pose.position.y, -0.0230769199229]
        z = [self.current_slave_pose.position.z, 0.840896642238]

        # x = [self.current_slave_pose.position.x, 0.403399335619]
        # y = [self.current_slave_pose.position.y, -0.430007534239]
        # z = [self.current_slave_pose.position.z, 1.16269467394]
        
        # x = [self.current_slave_pose.position.x, 0.353543514402]
        # y = [self.current_slave_pose.position.y, 0.435045131507]
        # z = [self.current_slave_pose.position.z, 0.760080619348]

        t = [rospy.Time.now(), rospy.Time.now() + rospy.Duration(T)]

        t = list(self.parser.get_time_vector_secs_nsecs(self.parser.durationVector2secsNsecsVector(t)))
        
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
        
    def PoseStampedToCartesianPositionList(self, pose):
        return [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]  

    def moveEEto(self, pose):
        self.end_effector_goal_pub.publish(pose)

    def refineTrajectory(self, traj, dt):

        traj_pos = self.parser.getCartesianPositions(traj)
        refined_traj = []
        i = 0
        t = 0

        master_pose_scaling = 0.5

        # set white button to zero to make sure loop is run         
        self.white_button_toggle = 0

        # -2 because traj_pos[i+1] is called and i starts at 0
        while self.white_button_toggle == 0:

            slave_goal = PoseStamped()

            # try:
            # calculate next pose wrt current pose
            if i <= len(traj_pos)-2:
                pos_next_wrt_pos_current = np.subtract(np.array(traj_pos[i+1]), np.array(traj_pos[i]))
                # pos_next_wrt_pos_current = np.subtract(np.array(traj_pos[i]), np.array(traj_pos[i+1]))

            else:
                pos_next_wrt_pos_current = np.subtract(np.array(traj_pos[-1]), np.array(traj_pos[-2]))
                # pos_next_wrt_pos_current = np.subtract(np.array(traj_pos[-2]), np.array(traj_pos[-1]))


            # add normalized master pose to the next pose wrt current pose to calculate refined pose
            pos_next_wrt_pos_current += [x*master_pose_scaling for x in self.PoseStampedToCartesianPositionList(self.normalizeMasterPose(self.master_pose))]

            ## transform this pose to base_footprint
            p = list(pos_next_wrt_pos_current)

            q2 = list(pos_next_wrt_pos_current)
            q2.append(0.0)
            if i <= len(traj_pos)-1:
                q1 = self.parser.getQuaternion(traj[i])
            else: 
                q1 = self.parser.getQuaternion(traj[-1])

            # f(p) = q*p*q^(-1) --> Rotate vector 
            qv = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q2,q1), tf.transformations.quaternion_conjugate(q1))[:3]
            # q = Quaternion([q1[3], q1[0], q1[1], q1[2]])
            # p_wrt_base = q.rotate(-1*np.asarray(p))
            
            # add position of traj[i] wrt base
            if i <= len(traj_pos)-1:
                vnew = qv + traj_pos[i]
                # vnew = np.add(np.asarray(p_wrt_base), np.asarray(traj_pos[i]))
                # print(vnew)
            else:
                vnew = qv + traj_pos[-1]
                # vnew = np.add(np.asarray(p_wrt_base), np.asarray(traj_pos[-1]))

            # append refined trajectory
            if i <= len(traj_pos)-1:
                # refined_traj.append(list(vnew) + [traj[i][3], traj[i][4], traj[i][5], traj[i][6], rospy.Time.now().secs, rospy.Time.now().nsecs])
                refined_traj.append(list(vnew) + [traj[i][3], traj[i][4], traj[i][5], traj[i][6], rospy.Duration(t).secs,  rospy.Duration(t).nsecs])

                # refined_traj.append(list(vnew) + [traj[i][3], traj[i][4], traj[i][5], traj[i][6], t])

            else:
                # refined_traj.append(list(vnew) + [traj[-1][3], traj[-1][4], traj[-1][5], traj[-1][6], rospy.Time.now().secs, rospy.Time.now().nsecs])
                refined_traj.append(list(vnew) + [traj[-1][3], traj[-1][4], traj[-1][5], traj[-1][6], rospy.Duration(t).secs, rospy.Duration(t).nsecs])

                # refined_traj.append(list(vnew) + [traj[-1][3], traj[-1][4], traj[-1][5], traj[-1][6], t])
            

            slave_goal.pose.position.x = refined_traj[i][0]
            slave_goal.pose.position.y = refined_traj[i][1]
            slave_goal.pose.position.z = refined_traj[i][2]

            if i <= len(traj_pos)-1:
                slave_goal.pose.orientation.x = traj[i][3]
                slave_goal.pose.orientation.y = traj[i][4]
                slave_goal.pose.orientation.z = traj[i][5]
                slave_goal.pose.orientation.w = traj[i][6]

            # if we make the new trajectory longer --> append orientation with last from predicted trajectory
            else: 
                
                slave_goal.pose.orientation.x = traj[-1][3]
                slave_goal.pose.orientation.y = traj[-1][4]
                slave_goal.pose.orientation.z = traj[-1][5]
                slave_goal.pose.orientation.w = traj[-1][6]

            slave_goal.header.frame_id = self.frame_id
            slave_goal.header.stamp = rospy.Time.now()

            self.moveEEto(slave_goal)

            time.sleep(dt)
            t += dt
            i += 1
        

        return refined_traj


    # from Ewerton: tau_D^new = tau_D^old + alpha * (tau_HR - tau_R)
    def determineNewTrajectory(self, pred_traj, refined_traj, alpha = 1):


        trajectories = [pred_traj, refined_traj]

        pred_traj = self.parser.trajFloatToSecsNsecs(pred_traj)
        
        refined_traj = self.parser._normalize(refined_traj)

        new_trajectory = []

        ## resample trajectories such that they can be subtracted
        
        refined_traj_pose = self.parser.getCartesianPositions(refined_traj)
        refined_traj_time = self.parser._getTimeVector(refined_traj)

        pred_traj_pose = self.parser.getCartesianPositions(pred_traj)
        pred_traj_time = self.parser._getTimeVector(pred_traj)

        plt.plot(refined_traj_pose)
        plt.plot(pred_traj_pose)
        plt.title('Predicted and refined trajectory before resampling')
        plt.xlabel('datapoint [-]')
        plt.ylabel('position [m]')
        # plt.show()

        n_pred = len(pred_traj)
        n_refined = len(refined_traj)

        # get lengths of both vectors
        n = [n_pred, n_refined]
        max_length = max(n)

        # dt = self.parser.getTimeInterval(trajectories[np.argmax(n)])
        dt_pred = self.parser.getTimeInterval(pred_traj)
        dt_refined = self.parser.getTimeInterval(refined_traj)

        T_pred = self.parser.secsNsecsToFloatSingle(pred_traj_time[-1])
        T_refined = self.parser.secsNsecsToFloatSingle(refined_traj_time[-1])

        l = max_length

        xvals_refined = np.linspace(0.0, T_refined, l)
        xvals_pred = np.linspace(0.0, T_pred, l)
        print('check2')

        refined_traj_time = ((np.asarray(self.parser._secsNsecsToFloat(refined_traj_time))))
        refined_traj_pos_x = (np.asarray(self.parser.getXpositions(refined_traj_pose)).reshape(len(refined_traj_time), 1))
        refined_traj_pos_y = (np.asarray(self.parser.getYpositions(refined_traj_pose)).reshape(len(refined_traj_time), 1))
        refined_traj_pos_z = (np.asarray(self.parser.getZpositions(refined_traj_pose)).reshape(len(refined_traj_time), 1))

        yinterp_refined_x = interp1d((refined_traj_time), np.transpose(refined_traj_pos_x), axis=1, fill_value="extrapolate")
        yinterp_refined_y = interp1d((refined_traj_time), np.transpose(refined_traj_pos_y), axis=1, fill_value="extrapolate")
        yinterp_refined_z = interp1d((refined_traj_time), np.transpose(refined_traj_pos_z), axis=1, fill_value="extrapolate")

        y_refined_new_x = yinterp_refined_x(xvals_refined)
        y_refined_new_y = yinterp_refined_y(xvals_refined)
        y_refined_new_z = yinterp_refined_z(xvals_refined)
        

        pred_traj_time = ((np.asarray(self.parser._secsNsecsToFloat(pred_traj_time))))
        pred_traj_pos_x = (np.asarray(self.parser.getXpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))

        pred_traj_pos_y = (np.asarray(self.parser.getYpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))
        pred_traj_pos_z = (np.asarray(self.parser.getZpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))

        print(pred_traj_pos_x[0])
        yinterp_pred_x = interp1d((pred_traj_time), np.transpose(pred_traj_pos_x), axis=1, fill_value="extrapolate")
        yinterp_pred_y = interp1d((pred_traj_time), np.transpose(pred_traj_pos_y), axis=1, fill_value="extrapolate")
        yinterp_pred_z = interp1d((pred_traj_time), np.transpose(pred_traj_pos_z), axis=1, fill_value="extrapolate")

        print('check3')


        y_pred_new_x = yinterp_pred_x(xvals_pred)
        y_pred_new_y = yinterp_pred_y(xvals_pred)
        y_pred_new_z = yinterp_pred_z(xvals_pred)

        plt.plot(np.asarray([list(y_refined_new_x[0]), list(y_refined_new_y[0]), list(y_refined_new_z[0])]).transpose())
        plt.plot(np.asarray([list(y_pred_new_x[0]), list(y_pred_new_y[0]), list(y_pred_new_z[0])]).transpose())

        plt.title('Predicted and refined trajectory after resampling')
        plt.xlabel('datapoint [-]')
        plt.ylabel('position [m]')
        # plt.show()
        # apply DTW
        # print(y_pred_new_x[0])
        y_refined_aligned, y_pred_aligned = DTW.apply_dtw([list(y_refined_new_x[0]), list(y_refined_new_y[0]), list(y_refined_new_z[0])] , [list(y_pred_new_x[0]), list(y_pred_new_y[0]), list(y_pred_new_z[0])])
        # print(y_pred_aligned[0])

        plt.plot(y_refined_aligned)
        plt.plot(y_pred_aligned)

        plt.title('Predicted and refined trajectory after DTW')
        plt.xlabel('datapoint [-]')
        plt.ylabel('position [m]')
        # plt.show()
        # lengths are the same so doesnt matter which length I take
        n = len(y_refined_aligned)
        qstart = refined_traj[0][3:7]
        qend = refined_traj[-1][3:7]

        refined_traj = []
        pred_traj = []

        new_trajectory = []

        t_refined = np.linspace(0.0, T_refined, n)
        t_pred = np.linspace(0.0, T_pred, n)

        for i,q in enumerate(trajectoryParser.interpolateQuaternions(qstart, qend, n, False)):
            refined_traj = [list(y_refined_aligned[i])[0], list(y_refined_aligned[i])[1], list(y_refined_aligned[i])[2], q[1], q[2], q[3], q[0], t_refined[i]] 
            pred_traj = [list(y_pred_aligned[i])[0], list(y_pred_aligned[i])[1], list(y_pred_aligned[i])[2], q[1], q[2], q[3], q[0], t_pred[i]]

            # print(np.asarray(refined_traj[0:3]))
            # print((pred_traj[0:3])) 
            # print(alpha * (np.subtract(np.asarray(refined_traj[0:3]), np.asarray(pred_traj[0:3])) ))
            # print(np.asarray(pred_traj[0:3]))
            # tau_D^new = tau_D^old + alpha * (tau_HR - tau_R)
            new_trajectory.append(list(np.add(np.asarray(pred_traj[0:3]), alpha * (np.subtract(np.asarray(refined_traj[0:3]), np.asarray(pred_traj[0:3])) ))) + refined_traj[3:])
        
        dt_new = t_refined[1]

        return new_trajectory, dt_new


    def run(self):
        self.goToInitialPose()
        self.calibrate_master_pose_for_normalization()

        while not rospy.is_shutdown() and refinement_node.grey_button_toggle == 0:
            
if __name__ == "__main__":
    node = trajectoryRefinement()

        
    while not rospy.is_shutdown() and refinement_node.grey_button_toggle == 0:

        if refine_counter % 2 == 0:
            rospy.loginfo("Determining trajectory...")
            goal[7:-1] = refinement_node.getMarkerWRTee()
            # goal[8:] = refinement_node.getMarkerWRTBase()

            while goal[8] == 0.0:        
                goal[7:-1] = refinement_node.getMarkerWRTee()
                # goal[8:] = refinement_node.getMarkerWRTBase()
        
            promp.clear_viapoints()
            promp.set_goal(goal, sigma=1e-6)
            generated_trajectory = promp.generate_trajectory(sigma_noise)

            traj_pred, dt = refinement_node.generate_trajectory_to_pred_traj(generated_trajectory)

            # print(traj_pred[-1])
            
            traj_pred_keypoints = learnedToExecuted(traj_pred, refinement_node.getMarkerWRTBase()).pred_traj_to_executed()
            
            traj_pred, dt = refinement_node.parser.interpolate_learned_keypoints(traj_pred_keypoints, 100)
            
            # for data in traj_pred:
            #     print(data)
            refine_counter += 1
            promp.plot_unconditioned_joints()
            ## plot_conditioned_joints doesnt work, use this instead:
            plt.figure()
            for joint_id, joint_name in enumerate(joints[0:3]):
                # print(joint_id)
                plt.plot(generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0], label=joint_name)
                plt.xlabel("datapoint [-]")

            plt.legend()
            plt.show()

        rospy.loginfo("Executing current predicted trajectory...")
        for i in range(50):
            # refinement_node.traj_pred_pub.publish(refinement_node.trajToVisMsg(refinement_node.ee_to_gripper_pose(traj_pred), r=1, g=0, b=0))
            refinement_node.traj_pub.publish(refinement_node.trajToVisMsg((traj_pred), r=1, g=0, b=0))
            refinement_node.traj_ref_pub.publish(refinement_node.trajToVisMsg((traj_pred_keypoints), r=0, g=0, b=1))

        time.sleep(5)
        traj_refined = refinement_node.refineTrajectory(traj_pred, dt)
        
        traj_new, dt_new = refinement_node.determineNewTrajectory(traj_pred, traj_refined, alpha)

        

        traj_refined_reversed = trajectoryRefinement.reverseTrajectory(traj_refined)
        refinement_node.executeTrajectory(traj_refined_reversed, dt_new)
        time.sleep(1)
        # refinement_node.goToInitialPose()

        for i in range(50):
            refinement_node.traj_ref_pub.publish(refinement_node.trajToVisMsg((traj_new), r=0, g=1, b=0))
        time.sleep(1)

        if input("Satisfied with this trajectory? 1/0") == 1: 
            rospy.loginfo("Adding trajectory to model...")
            traj_add = []
            for i in range(len(traj_new)):
                traj_add.append(traj_new[i][:-1] + [dt_new] + list(refinement_node.getMarkerWRTBase()))

            traj_add_for_learning = refinement_node.parse_to_relative_traj(traj_add)

            plt.plot([t[0:3] for t in traj_add_for_learning])
            plt.title('New trajectory')
            plt.xlabel("datapoint [-]")
            plt.ylabel("position [m]")
            # plt.show()
            promp.add_demonstration(np.array(traj_add_for_learning))

            
            # promp.plot_unconditioned_joints()
            ## plot_conditioned_joints doesnt work, use this instead:
            plt.figure()
            for joint_id, joint_name in enumerate(joints[0:3]):
                # print(joint_id)
                plt.plot(generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0], label=joint_name)
                plt.xlabel("datapoint [-]")
                plt.ylabel('position [m]')
                plt.title('Predicted trajectory')

            plt.legend()
            # plt.show()
            print('check')
            refine_counter += 1
        else:
            if input("Use refined or predicted trajectory for refinement? 1/0") == 1:
                traj_pred = traj_new
            else: pass

        refinement_node.clearTrajectoriesRviz()

          