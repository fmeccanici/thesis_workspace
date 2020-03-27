#!/usr/bin/env python

import rospy, time, tf, os
import thread

import numpy as np
from python2to3_function_caller.python2to3_function_caller import *
from trajectory_parser.trajectory_parser import *
from spline_interpolation.spline_interpolation import *

from dmp.srv import *
from dmp.msg import *
from dmp_python.dmp_python import *

from visualization_msgs.msg import MarkerArray
from geomagic_touch_m.msg import GeomagicButtonEvent

from slave_control.msg import ControlState
from master_control.msg import ControlComm
from aruco_msgs.msg import MarkerArray

from scipy.interpolate import interp1d
from geometry_msgs.msg import PoseStamped, WrenchStamped, PoseArray, Pose

from lfd_msgs.msg import TrajectoryVisualization

from pyquaternion import Quaternion 
from scipy.spatial.transform import Rotation as R

from promp_python.promp_python import *
from aruco_msgs.msg import MarkerArray
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
        self.parser = trajectoryParser()
        self.spl_interp = splineInterpolation()

        self.dmp_python = dmpPython()
        self.master_pose = Pose()

        self.refinementFlag = 0

        self.grey_button = 0
        self.grey_button_prev = 0
        self.grey_button_toggle = 0

        self.white_button = 0
        self.white_button_prev = 0
        self.white_button_toggle = 0

        self.end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)
        self.geo_button_sub = rospy.Subscriber("geo_buttons_m", GeomagicButtonEvent, self._buttonCallbackToggle)
        self.end_effector_pose_sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self._end_effector_pose_callback)
        self.marker_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self._marker_detection_callback)

        self.traj_pred_pub = rospy.Publisher('trajectory_visualizer/trajectory_predicted', TrajectoryVisualization, queue_size=10)
        self.traj_ref_pub = rospy.Publisher('trajectory_visualizer/trajectory_refined', TrajectoryVisualization, queue_size=10)

        self.master_pose_sub = rospy.Subscriber('master_control_comm', ControlComm, self._masterPoseCallback)
        self.force_state_pub = rospy.Publisher('geo_force_state_m', WrenchStamped, queue_size=10)
        self.effort_pub = rospy.Publisher('refinement_force', WrenchStamped, queue_size=10)
        self.gripper_wrt_ee_sub = rospy.Subscriber('gripper_wrt_ee', PoseStamped, self._gripper_wrt_ee_callback)

        self.frame_id = '/base_footprint'
        self.initMasterNormalizePose()
        
        self.object_marker_pose = Pose()
        self.gripper_wrt_ee = Pose()
    
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

    def _gripper_wrt_ee_callback(self, data):
        self.gripper_wrt_ee = data.pose

    def _masterPoseCallback(self, data):
        self.master_pose = data.master_pose.pose        

    def _buttonCallbackToggle(self, data):
        self.grey_button_prev = self.grey_button
        self.grey_button = data.grey_button
        self.white_button_prev = self.white_button
        self.white_button = data.white_button

        if (self.grey_button != self.grey_button_prev) and (self.grey_button == 1):
            self.grey_button_toggle = not self.grey_button_toggle

        if (self.white_button != self.white_button_prev) and (self.white_button == 1):
            self.white_button_toggle = not self.white_button_toggle

    def _buttonCallbackNoToggle(self, data):
        self.grey_button = data.grey_button
        self.white_button = data.white_button

    def _end_effector_pose_callback(self,data):
        self.current_slave_pose = data.pose

    def isCoupled(self):
        if self.grey_button == 1:
            return True
        else: return False

    def initMasterNormalizePose(self):
        self.firstMasterPose = PoseStamped()
        self.firstMasterPose.pose.position.x = 0.412058425026
        self.firstMasterPose.pose.position.y = -0.00570407599211

        # for some reason the minus sign disappeared or ee axis is flipped 180deg
        # self.firstMasterPose.pose.position.z = -0.486718259108
        # for some reason this value changed suddenly ?? -> This is why scaling was wrong of refined trajectory
        # self.firstMasterPose.pose.position.z = -0.169880290808
        self.firstMasterPose.pose.position.z = -0.327476944265


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
        x = [self.current_slave_pose.position.x, 0.403399335619]
        y = [self.current_slave_pose.position.y, -0.430007534239]
        z = [self.current_slave_pose.position.z, 1.16269467394]
        
        # x = [self.current_slave_pose.position.x, 0.353543514402]
        # y = [self.current_slave_pose.position.y, 0.435045131507]
        # z = [self.current_slave_pose.position.z, 0.760080619348]

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
        
    def PoseStampedToCartesianPositionList(self, pose):
        return [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]  


    def moveEEto(self, pose):
        self.end_effector_goal_pub.publish(pose)


    def refineTrajectory(self, traj, dt):
        # dt = self.parser.getTimeInterval(traj)
        traj_pos = self.parser.getCartesianPositions(traj)
        refined_traj = []
        i = 0

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
            else:
                pos_next_wrt_pos_current = np.subtract(np.array(traj_pos[-1]), np.array(traj_pos[-2]))


            # add normalized master pose to the next pose wrt current pose to calculate refined pose
            pos_next_wrt_pos_current += [x*master_pose_scaling for x in self.PoseStampedToCartesianPositionList(self.normalizeMasterPose(self.master_pose))]

            ## transform this pose to base_footprint
            q2 = list(pos_next_wrt_pos_current)
            q2.append(0.0)
            if i <= len(traj_pos)-1:
                q1 = self.parser.getQuaternion(traj[i])
            else: 
                q1 = self.parser.getQuaternion(traj[-1])

            # f(p) = q*p*q^(-1) --> Rotate vector 
            qv = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q2,q1), tf.transformations.quaternion_conjugate(q1))[:3]
            
            # add position of traj[i] wrt base
            if i <= len(traj_pos)-1:
                vnew = qv + traj_pos[i]
            else:
                vnew = qv + traj_pos[-1]

            # t = self.parser._secsNsecsToFloat([rospy.Time.now().secs, rospy.Time.now().nsecs])

            # append refined trajectory
            if i <= len(traj_pos)-1:
                refined_traj.append(list(vnew) + [traj[i][3], traj[i][4], traj[i][5], traj[i][6], rospy.Time.now().secs, rospy.Time.now().nsecs])
                # refined_traj.append(list(vnew) + [traj[i][3], traj[i][4], traj[i][5], traj[i][6], t])

            else:
                refined_traj.append(list(vnew) + [traj[-1][3], traj[-1][4], traj[-1][5], traj[-1][6], rospy.Time.now().secs, rospy.Time.now().nsecs])
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
            i += 1
        

        return refined_traj


    # from Ewerton: tau_D^new = tau_D^old + alpha * (tau_HR - tau_R)
    def determineNewTrajectory(self, pred_traj, refined_traj, alpha = 1):
        
        trajectories = [pred_traj, refined_traj]

        pred_traj = self.parser.trajFloatToSecsNsecs(pred_traj)
        refined_traj = self.parser._normalize(refined_traj)

        # print(pred_traj)
        new_trajectory = []

        ## resample trajectories such that they can be subtracted
        

  
        # print('dt ' + str(dt))
        # downsample refined trajectory to match dt
        # refined_traj = self.parser.downsample(refined_traj, dt)
        
        
        # print(refined_traj)
        # print(pred_traj)



        refined_traj_pose = self.parser.getCartesianPositions(refined_traj)
        refined_traj_time = self.parser._getTimeVector(refined_traj)
        


        pred_traj_pose = self.parser.getCartesianPositions(pred_traj)
        pred_traj_time = self.parser._getTimeVector(pred_traj)
        
        n_pred = len(pred_traj)
        n_refined = len(refined_traj)

        # get lengths of both vectors
        n = [n_pred, n_refined]
        max_length = max(n)
        min_length = min(n)

        # dt = self.parser.getTimeInterval(trajectories[np.argmax(n)])
        dt_pred = self.parser.getTimeInterval(pred_traj)
        dt_refined = self.parser.getTimeInterval(refined_traj)
        dt = [dt_pred, dt_refined]

        T_pred = self.parser.secsNsecsToFloatSingle(pred_traj_time[-1])
        T_refined = self.parser.secsNsecsToFloatSingle(refined_traj_time[-1])
        T = [T_pred, T_refined]
        # print(pred_traj_time)
        # print(refined_traj_time)

        # plt.plot(refined_traj_pose)
        # plt.plot(pred_traj_pose)
        # plt.show()

        # print(len(pred_traj_pose))
        # print(len(refined_traj_pose))

        l = max_length
        # l = min_length

        # if n_pred > n_refined:
        #     # dt_refined = dt_pred * T_pred / T_refined
        #     # dt_refined = n_refined / (n_pred / dt_pred)
        #     T_refined = T_pred / dt_pred * dt_refined
        # elif n_pred < n_refined:
        #     # dt_pred = dt_refined * T_refined / T_pred
        #     # dt_pred = n_pred / (n_refined / dt_refined)
        #     T_pred = T_refined / dt_refined * dt_pred
        # else: pass

        # dt_refined = dt * len(refined_traj_time) / n[np.argmax(lengths)]
        # dt_pred = dt * len(pred_traj_time) / n[np.argmax(lengths)]

        # dt_refined = dt * len(refined_traj_time) / lengths[np.argmin(lengths)]
        # dt_pred = dt * len(pred_traj_time) / lengths[np.argmin(lengths)]
        # print(T_refined)

        xvals_refined = np.linspace(0.0, T_refined, l)
        xvals_pred = np.linspace(0.0, T_pred, l)

        # print(xvals_refined)
        # print(xvals_pred)
        
        # print(xvals_refined)
        # print(T_refined)
        # print(xvals_pred)
        # print(T_pred)

        # print(self.parser._secsNsecsToFloat(refined_traj_time))

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

        # print(len(xvals_refined))
        # print(len(xvals_pred))
        # print(len(y_refined_new_x[0]))

        # print('ref' + str(list(y_refined_new_x[0])))
        # plt.plot(xvals_refined.reshape(1, len(y_refined_new_x[0])), list(y_refined_new_x[0]))
        # plt.plot(xvals_refined, list(y_refined_new_x[0]))

        # plt.show()

        pred_traj_time = ((np.asarray(self.parser._secsNsecsToFloat(pred_traj_time))))
        pred_traj_pos_x = (np.asarray(self.parser.getXpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))

        pred_traj_pos_y = (np.asarray(self.parser.getYpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))
        pred_traj_pos_z = (np.asarray(self.parser.getZpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))

        yinterp_pred_x = interp1d((pred_traj_time), np.transpose(pred_traj_pos_x), axis=1, fill_value="extrapolate")
        yinterp_pred_y = interp1d((pred_traj_time), np.transpose(pred_traj_pos_y), axis=1, fill_value="extrapolate")
        yinterp_pred_z = interp1d((pred_traj_time), np.transpose(pred_traj_pos_z), axis=1, fill_value="extrapolate")

        y_pred_new_x = yinterp_pred_x(xvals_pred)
        y_pred_new_y = yinterp_pred_y(xvals_pred)
        y_pred_new_z = yinterp_pred_z(xvals_pred)

        print(y_pred_new_x)
        print(y_refined_new_x)
        # plt.plot(pred_traj_time, pred_traj_pos_x)
        # plt.plot(refined_traj_time, refined_traj_pos_x)
        # plt.plot(xvals_pred, y_pred_new_x[0])
        # plt.plot(xvals_refined, y_refined_new_x[0])
        plt.plot(refined_traj_time, refined_traj_pose)
        plt.plot(pred_traj_time, pred_traj_pose)
        
        # plt.plot(pred_traj_time, pred_traj_pos_x)
        # plt.plot((refined_traj_time), refined_traj_pos_x)

        # plt.plot(list(y_pred_new_x[0]))
        # plt.plot(list(y_refined_new_x[0]))
        plt.show()
        # print(y_pred_new_x - y_refined_new_x)
        
        # lengths are the same so doesnt matter which length I take
        n = len(xvals_pred)
        qstart = refined_traj[0][3:7]
        qend = refined_traj[-1][3:7]

        refined_traj = []
        pred_traj = []

        new_trajectory = []


        for i,q in enumerate(trajectoryParser.interpolateQuaternions(qstart, qend, n, False)):

            refined_traj = [y_refined_new_x[0][i], y_refined_new_y[0][i], y_refined_new_z[0][i], q[1], q[2], q[3], q[0], xvals_refined[i]] 
            pred_traj = [y_pred_new_x[0][i], y_pred_new_y[0][i], y_pred_new_z[0][i], q[1], q[2], q[3], q[0], xvals_pred[i]]
            
            # tau_D^new = tau_D^old + alpha * (tau_HR - tau_R)
            # print((np.subtract(refined_traj[0:3], pred_traj[0:3]) ))
            new_trajectory.append(list(np.add(pred_traj[0:3], alpha * (np.subtract(refined_traj[0:3], pred_traj[0:3]) ))) + refined_traj[3:])

        dt_new = dt_refined

        return new_trajectory, dt_new

    def trajToVisMsg(self, traj, r, g, b):
        
        pose_array = PoseArray()
        visualization_msg = TrajectoryVisualization()

        for pose in traj:
            pose_ = Pose()

            pose_.position.x = pose[0]
            pose_.position.y = pose[1]
            pose_.position.z = pose[2]

            pose_.orientation.x = pose[3]
            pose_.orientation.y = pose[4]
            pose_.orientation.z = pose[5]
            pose_.orientation.w = pose[6]

            pose_array.poses.append(pose_)

            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = self.frame_id

        visualization_msg.pose_array = pose_array
        visualization_msg.r = r
        visualization_msg.g = g
        visualization_msg.b = b

        return visualization_msg
    
    # doesnt work yet
    def ee_to_gripper_pose(self, traj):
        traj_gripper_wrt_base = []

        for ee_pose in traj:
            
            # q = Quaternion(ee_pose[3:7])
            r = R.from_quat(ee_pose[3:7])
            v = [self.gripper_wrt_ee.position.x, self.gripper_wrt_ee.position.y, self.gripper_wrt_ee.position.z]
            
            
            # gripper_wrt_base = q.rotate([self.gripper_wrt_ee.position.x, self.gripper_wrt_ee.position.y, self.gripper_wrt_ee.position.z])
            gripper_wrt_base = r.apply(v)

            gripper_wrt_base_x = gripper_wrt_base[0] + ee_pose[0]
            gripper_wrt_base_y = gripper_wrt_base[1] + ee_pose[1]
            gripper_wrt_base_z = gripper_wrt_base[2] + ee_pose[2]

            # doesnt matter for pose array which orientation I use, they should be wrt base frame and are now wrt ee frame 
            traj_gripper_wrt_base.append([gripper_wrt_base_x, gripper_wrt_base_y, gripper_wrt_base_z, self.gripper_wrt_ee.orientation.x, self.gripper_wrt_ee.orientation.y, self.gripper_wrt_ee.orientation.z, self.gripper_wrt_ee.orientation.w])

        return traj_gripper_wrt_base

    def getGoalFromMarker(self):
        x = self.object_marker_pose.position.x
        y = self.object_marker_pose.position.y
        z = self.object_marker_pose.position.z

        return [x,y,z]

    def getQuaternionForInterpolation(self):
        DIR = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_refinement/data/raw/'
        traj_file = 'raw_trajectory_1.txt'

        traj = self.parser.openTrajectoryFile(traj_file, DIR)
        qstart = traj[0][3:7]
        qend = traj[-1][3:7]

        return qstart, qend

    def generate_trajectory_to_pred_traj(self, generated_trajectory):
        joint_id = 0
        pred_traj_x = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

        joint_id = 1
        pred_traj_y = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

        joint_id = 2
        pred_traj_z = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

        joint_id = 3
        pred_traj_qx = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

        joint_id = 4
        pred_traj_qy = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

        joint_id = 5
        pred_traj_qz = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

        joint_id = 6
        pred_traj_qw = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

        joint_id = 7
        pred_traj_dt = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

        pred_traj = []
        dt = pred_traj_dt[0]

        t = 0

        for i in range(len(pred_traj_dt)):
            pred_traj.append([pred_traj_x[i], pred_traj_y[i], pred_traj_z[i], pred_traj_qx[i], pred_traj_qy[i], pred_traj_qz[i], pred_traj_qw[i], t])
            t += dt
        

        return pred_traj, dt

if __name__ == "__main__":
    refinement_node = trajectoryRefinement()
    
    DIR = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_refinement/data/resampled/'
    traj_files = [name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))]
    
    trajectories = []
    # joints = ["joint_x", "joint_y", "joint_z", "dt", "object_x", "object_y", "object_z"]
    joints = ["joint_x", "joint_y", "joint_z", "qx", "qy", "qz", "qw",  "dt", "object_x", "object_y", "object_z"]
    
    for traj in traj_files:
        trajectory = refinement_node.parser.openTrajectoryFile(traj, DIR)
        trajectory = np.array(trajectory)
        trajectories.append(trajectory)


    num_points = len(trajectories[0])

    promp = ProMPContext(joints, num_points=num_points)

    goal = np.zeros(len(joints))
    goal[8:] = refinement_node.getGoalFromMarker()

    refinement_node.goToInitialPose()

    while goal[8] == 0.0:        
        goal[8:] = refinement_node.getGoalFromMarker()


    for traj in trajectories:
        # print(traj)
        promp.add_demonstration(traj)
    
    r = rospy.Rate(30)

    time.sleep(1)

    sigma_noise=0.03

    promp.clear_viapoints()
    promp.set_goal(goal, sigma=1e-6)

    generated_trajectory = promp.generate_trajectory(sigma_noise)

    traj_pred, dt = refinement_node.generate_trajectory_to_pred_traj(generated_trajectory)

    # print(traj_pred)

    # refinement_node.executeTrajectory(traj_pred, dt)

    alpha = 1


    while not rospy.is_shutdown() and refinement_node.grey_button_toggle == 0:
        
        for i in range(10):
            # refinement_node.traj_pred_pub.publish(refinement_node.trajToVisMsg(refinement_node.ee_to_gripper_pose(traj_pred), r=1, g=0, b=0))
            refinement_node.traj_pred_pub.publish(refinement_node.trajToVisMsg((traj_pred), r=1, g=0, b=0))
        

        traj_refined = refinement_node.refineTrajectory(traj_pred, dt)
        
        traj_new, dt_new = refinement_node.determineNewTrajectory(traj_pred, traj_refined, alpha)


        traj_refined_reversed = trajectoryRefinement.reverseTrajectory(traj_refined)
        refinement_node.executeTrajectory(traj_refined_reversed, dt_new)
        time.sleep(1)
        refinement_node.goToInitialPose()

        for i in range(10):
            refinement_node.traj_ref_pub.publish(refinement_node.trajToVisMsg((traj_new), r=0, g=1, b=0))
        time.sleep(1)

        for i in range(len(traj_new)):

            traj_new[i][:] = traj_new[i][:-1] + [dt_new] + list(goal[8:])



        promp.add_demonstration(np.array(traj_new))

        goal[8:] = refinement_node.getGoalFromMarker()
        promp.set_goal(goal, sigma=1e-6)

        generated_trajectory = promp.generate_trajectory(sigma_noise)

        traj_pred, dt = refinement_node.generate_trajectory_to_pred_traj(generated_trajectory)

