#!/usr/bin/env python3.5

# import external python packages
import rospy, time, os, rospkg
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

from pyquaternion import Quaternion
from pynput.keyboard import Key, Listener, KeyCode

# import ros messages
from learning_from_demonstration.srv import AddDemonstration, AddDemonstrationResponse, MakePrediction, MakePredictionResponse, SetObject, SetObjectResponse
from promp_context_ros.msg import prompTraj
from trajectory_refinement.srv import RefineTrajectory, RefineTrajectoryResponse, CalibrateMasterPose, CalibrateMasterPoseResponse
from execution_failure_detection.srv import GetExecutionFailure

from geomagic_touch_m.msg import GeomagicButtonEvent
from master_control.msg import ControlComm
from trajectory_visualizer.msg import TrajectoryVisualization
from geometry_msgs.msg import PoseStamped, WrenchStamped, PoseArray, Pose, Point
from aruco_msgs.msg import MarkerArray
from teleop_control.msg import Keyboard
from std_msgs.msg import Bool
from execution_failure_detection.msg import ExecutionFailure

# import own python packages
from trajectory_visualizer_python.trajectory_visualizer_python import trajectoryVisualizer
from learning_from_demonstration_python.trajectory_resampler import trajectoryResampler
from learning_from_demonstration_python.dynamic_time_warping import DTW
from learning_from_demonstration_python.trajectory_parser import trajectoryParser


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
        
        # initialize button parameters
        self.grey_button = 0
        self.grey_button_prev = 0
        self.grey_button_toggle = 0

        self.white_button = 0
        self.white_button_prev = 0
        self.white_button_toggle = 0

        # initialize other parameters
        self.base_frame = 'base_footprint'
        self.object_marker_pose = Pose()
        self.gripper_wrt_ee = Pose()
        
        # get rosparameters specified in launch
        self._get_parameters()

        # initialize pub/sub
        self.end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)
        self.traj_vis_pub = rospy.Publisher('trajectory_visualizer/trajectory', TrajectoryVisualization, queue_size=10)

        if self.button_source == "omni":
            self.geo_button_sub = rospy.Subscriber("geo_buttons_m", GeomagicButtonEvent, self._buttonCallback)
        elif self.button_source == "keyboard":
            # self.geo_button_sub = rospy.Subscriber("keyboard", GeomagicButtonEvent, self._buttonCallback)
            self.keyboard_sub = rospy.Subscriber('keyboard_control', Keyboard, self._keyboard_callback)


        self.end_effector_pose_sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self._end_effector_pose_callback)
        self.marker_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self._marker_detection_callback)
        # self.master_pose_sub = rospy.Subscriber('master_control_comm', ControlComm, self._masterPoseCallback)
        # self.keyboard_sub = rospy.Subscriber('keyboard_control', Keyboard, self._keyboard_callback)
        self.execution_failure_sub = rospy.Subscriber("/execution_failure", ExecutionFailure, self._executionFailureCallback)

        # services
        self._refine_trajectory_service = rospy.Service('refine_trajectory', RefineTrajectory, self._refine_trajectory)
        self._calibrate_master_pose_service = rospy.Service('calibrate_master_pose', CalibrateMasterPose, self._calibrate_master_pose)        
        
        # initialize other classes
        self.parser = trajectoryParser()
        self.resampler = trajectoryResampler()
        self.visualizer = trajectoryVisualizer()
        self.dtw = DTW()


        self.master_pose = Pose()

        # calibrate master pose
        self.calibrate_master_pose_for_normalization()


    def _get_parameters(self):
        self.button_source = rospy.get_param('~button_source')
        print("Button source set to: " + str(self.button_source))

    def _executionFailureCallback(self, data):
        self.object_reached = data.object_reached.data
        self.obstacle_hit = data.obstacle_hit.data

    def _keyboard_callback(self, data):

        added_value = 0.01
        if data.key.data == 'q':
            self.master_pose.position.x += added_value
        elif data.key.data == 'a':
            self.master_pose.position.x -= added_value

        elif data.key.data == 'w':
            self.master_pose.position.y += added_value
        elif data.key.data == 's':
            self.master_pose.position.y -= added_value

        elif data.key.data == 'e':
            self.master_pose.position.z += added_value
        elif data.key.data == 'd':
            self.master_pose.position.z -= added_value


        elif data.key.data == '':
            if self.master_pose.position.y > 0.0:
                self.master_pose.position.y -= added_value/10
            elif self.master_pose.position.y < 0.0:
                self.master_pose.position.y += added_value/10


            if self.master_pose.position.x > 0.0:
                self.master_pose.position.x -= added_value/10
            elif self.master_pose.position.x < 0.0:
                self.master_pose.position.x += added_value/10
        
            if self.master_pose.position.z > 0.0:
                self.master_pose.position.z -= added_value/10
            elif self.master_pose.position.z < 0.0:
                self.master_pose.position.z += added_value/10       

        elif data.key.data == 'space':
            if self.white_button_toggle == 0:
                self.white_button_toggle = 1
                print("set button to " + str(self.white_button_toggle))
            # else: self.white_button_toggle = 0
        # print(self.white_button_toggle)
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
    
    def getMarkerWRTBase(self):
        x = self.object_marker_pose.position.x
        y = self.object_marker_pose.position.y
        z = self.object_marker_pose.position.z

        return [x,y,z]

    def getMarkerWRTee(self):
        # x = self.object_marker_pose.position.x
        # y = self.object_marker_pose.position.y
        # z = self.object_marker_pose.position.z

        x = self.object_marker_pose.position.x - self.current_slave_pose.position.x  
        y = self.object_marker_pose.position.y - self.current_slave_pose.position.y 
        z = self.object_marker_pose.position.z - self.current_slave_pose.position.z 

    def _calibrate_master_pose(self, req):
        self.calibrate_master_pose_for_normalization()
        response = CalibrateMasterPoseResponse()

        return response

    def calibrate_master_pose_for_normalization(self):
        # rospy.wait_for_message('/master_control_comm', ControlComm, timeout=5.0)
        self.firstMasterPose = PoseStamped()
        self.firstMasterPose.pose.position.x = self.master_pose.position.x
        self.firstMasterPose.pose.position.y = self.master_pose.position.y

        self.firstMasterPose.pose.position.z = self.master_pose.position.z
        
        self.firstMasterPose.pose.orientation.x = self.master_pose.orientation.x
        self.firstMasterPose.pose.orientation.y = self.master_pose.orientation.y
        self.firstMasterPose.pose.orientation.z = self.master_pose.orientation.z
        self.firstMasterPose.pose.orientation.w = self.master_pose.orientation.w

        rospy.loginfo('Calibrated master pose for refinement')

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
        
    def PoseStampedToCartesianPositionList(self, pose):
        return [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]  

    def moveEEto(self, pose):
        self.end_effector_goal_pub.publish(pose)

    def refineTrajectory(self, traj, dt):
        self.obstacle_hit_once = False

        traj_pos = self.parser.getCartesianPositions(traj)
        refined_traj = []
        i = 0
        t = 0

        master_pose_scaling = 0.5

        # set white button to zero to make sure loop is run         
        self.white_button_toggle = 0

        # print('traj pos[0] = ' + str(traj_pos[0]))
        # -2 because traj_pos[i+1] is called and i starts at 0
        while self.white_button_toggle == 0:

            slave_goal = PoseStamped()

            # calculate next pose wrt current pose
            if i <= len(traj_pos)-2:
                pos_next_wrt_pos_current = np.subtract(np.array(traj_pos[i+1]), np.array(traj_pos[i]))

            else:
                pos_next_wrt_pos_current = np.subtract(np.array(traj_pos[-1]), np.array(traj_pos[-2]))

            # add normalized master pose to the next pose wrt current pose to calculate refined pose
            pos_next_wrt_pos_current += [x*master_pose_scaling for x in self.PoseStampedToCartesianPositionList(self.normalizeMasterPose(self.master_pose))]
        
            # print('self.normalizeMasterPose(self.master_pose) = ' + str(self.normalizeMasterPose(self.master_pose)))
            # print('pos_next_wrt_pos_current = ' + str(pos_next_wrt_pos_current))


            ## transform this pose to base_footprint
            p = list(pos_next_wrt_pos_current)

            # add position of traj[i] wrt base
            if i <= len(traj_pos)-1:
                vnew = np.add(p, traj_pos[i])
            else:
                vnew = np.add(p, traj_pos[-1])
            if i == 0:
                vnew = traj_pos[i]
            # print('vnew = ' + str(vnew))

            ee_position = list(vnew)
            t_list = [t]

            # print(t_list)

            # append refined trajectory
            if i <= len(traj_pos)-1:
                # ee_orientation = [traj[i][3], traj[i][4], traj[i][5], traj[i][6]]
                qx = self.current_slave_pose.orientation.x
                qy = self.current_slave_pose.orientation.y
                qz = self.current_slave_pose.orientation.z
                qw = self.current_slave_pose.orientation.w

                ee_orientation = [traj[i][3], traj[i][4], traj[i][5], traj[i][6]]
                # ee_orientation = [qx, qy, qz, qw]
                refined_traj.append(ee_position + ee_orientation + t_list)
            else:
                ee_orientation = [traj[-1][3], traj[-1][4], traj[-1][5], traj[-1][6]]
                # ee_orientation = [qx, qy, qz, qw]

                refined_traj.append(ee_position + ee_orientation + t_list)
            

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

            slave_goal.header.frame_id = self.base_frame
            slave_goal.header.stamp = rospy.Time.now() 

            self.moveEEto(slave_goal)

            # check if collision occurs with environment
            try:
                # get_execution_failure = rospy.ServiceProxy('get_execution_failure', GetExecutionFailure)
                # resp = get_execution_failure()

                # # if collision occurs set to true for this refinement
                # if resp.obstacle_hit.data == True:
                #     self.obstacle_hit = resp.obstacle_hit.data
                if self.obstacle_hit == True and self.obstacle_hit_once == False:
                    self.obstacle_hit_once = True

            except (rospy.ServiceException, rospy.ROSException) as e:
                print("Service call failed: %s"%e)

            # print("dt = " + str(dt))

            time.sleep(dt)
            t += dt
            i += 1
        
        # needed since there is a delay in the object kicked over detection
        time.sleep(4)
        
        if self.obstacle_hit == True and self.obstacle_hit_once == False:
            self.obstacle_hit_once = True

        return refined_traj


    # from Ewerton: tau_D^new = tau_D^old + alpha * (tau_HR - tau_R)
    def determineNewTrajectory(self, pred_traj, refined_traj, alpha = 1):
        
        # quaternions used for interpolation
        qstart = pred_traj[0][3:7]
        qend = pred_traj[-1][3:7]  
        
        T_refined = self.parser.get_total_time(refined_traj)
        T_pred = self.parser.get_total_time(pred_traj)

        # resample so their n matches
        y_pred, y_ref = self.resampler.match_refined_predicted(pred_traj, refined_traj)
        
        # apply DTW to align them
        y_refined_aligned, y_pred_aligned = self.dtw.apply_dtw(y_ref, y_pred)



        # plt.plot(y_refined_aligned)
        # plt.plot(y_pred_aligned)

        # plt.title('Predicted and refined trajectory after DTW')
        # plt.xlabel('datapoint [-]')
        # plt.ylabel('position [m]')
        # plt.show()

        # lengths are the same so doesnt matter which length I take
        n = len(y_refined_aligned)


        refined_traj = []
        pred_traj = []

        new_trajectory = []

        t_refined = np.linspace(0.0, T_refined, n)
        t_pred = np.linspace(0.0, T_pred, n)

        for i,q in enumerate(self.resampler.interpolate_quaternions(qstart, qend, n, False)):
            ref_pos = [list(y_refined_aligned[i])[0], list(y_refined_aligned[i])[1], list(y_refined_aligned[i])[2]]
            ref_ori = [q[1], q[2], q[3], q[0]]
            ref_t = [t_refined[i]]
            refined_traj = ref_pos + ref_ori + ref_t

            pred_pos = [list(y_pred_aligned[i])[0], list(y_pred_aligned[i])[1], list(y_pred_aligned[i])[2]]
            pred_ori = [q[1], q[2], q[3], q[0]]
            pred_t = [t_pred[i]]
            

            pred_traj = pred_pos + pred_ori + pred_t

            # tau_D^new = tau_D^old + alpha * (tau_HR - tau_R)
            new_trajectory.append(list(np.add(np.asarray(pred_pos), alpha * (np.subtract(np.asarray(ref_pos), np.asarray(pred_pos)) ))) + refined_traj[3:])
        
        dt_new = t_refined[1]
        # print( "new_traj time = " + str([ x[-1] for x in new_trajectory]))
        return new_trajectory, dt_new

    # ROS service for refining trajectory
    def _refine_trajectory(self, req):
        rospy.loginfo("Refining trajectory...")
        prediction, dt = self.parser.promptraj_msg_to_execution_format(req.trajectory)
        
        # print("prediction = " + str(prediction))
        ndesired = 75

        if req.T_desired != 0.0:
            dt = req.T_desired / ndesired

        n = len(prediction)
        # dt = req.trajectory.times[1]
        print("len prediction = " + str(len(prediction)))
        if n < ndesired:
            prediction, dt = self.resampler.interpolate_learned_keypoints(prediction, ndesired)

        # print("time vector = " + str([x[-1] for x in prediction]))

        refined_prediction = self.refineTrajectory(prediction, dt)
        new_traj, new_dt = self.determineNewTrajectory(prediction, refined_prediction)

        new_traj = self.resampler.resample_time(new_traj, self.parser.get_total_time(prediction))
        # print("new_traj good T = " + str(new_traj))
        n_pred = len(prediction)
        print("len refinement = " + str(len(new_traj)))
        # plt.figure()
        # plt.plot(self.parser.getCartesianPositions(new_traj), color='green', label='refined')
        # plt.plot(self.parser.getCartesianPositions(prediction), color='red', label='predicted')
        # plt.title("Comparison refined and predicted trajectory (before resampling)")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("position [m]")
        # plt.grid()
        # plt.legend()
        # plt.savefig('/home/fmeccanici/Documents/thesis/figures/debug_refinement/comparison_new_predicted_before_resampling.png')
        # plt.close()

        # resample new trajectory to match the prediction
        new_traj = self.resampler.interpolate_predicted_trajectory(new_traj, n_pred)
            

        # plt.figure()
        # plt.plot(self.parser.getCartesianPositions(new_traj), color='green', label='refined')
        # plt.plot(self.parser.getCartesianPositions(prediction), color='red', label='predicted')
        # plt.title("Comparison refined and predicted trajectory (after resampling)")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("position [m]")
        # plt.grid()
        # plt.legend()
        # plt.savefig('/home/fmeccanici/Documents/thesis/figures/debug_refinement/comparison_new_predicted_after_resampling.png')
        # plt.close()   

        rospy.loginfo("len_new_traj before dtw = " + str(len(new_traj)))
        rospy.loginfo("len pred before dtw = " + str(len(prediction)))

        # print("time vector before dtw = " + str([x[-1] for x in new_traj]))

        # apply dynamic time warping --> reference = prediction
        # pred_aligned, new_traj = self.dtw.apply_dtw(prediction, new_traj)
        # rospy.loginfo("len pred after dtw = " + str(len(pred_aligned)))
        # rospy.loginfo("len_new_traj after dtw = " + str(len(new_traj)))

        # print("pred_aligned_time = " + str( [x[-1] for x in pred_aligned]))

        # plt.figure()
        # plt.plot(self.parser.getCartesianPositions(new_traj), color='green', label='refined')
        # # plt.plot(self.parser.getCartesianPositions(pred_aligned), color='red', label='predicted')
        # plt.title("Comparison refined and predicted trajectory (after DTW)")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("position [m]")
        # plt.grid()
        # plt.legend()
        # plt.savefig('/home/fmeccanici/Documents/thesis/figures/debug_refinement/comparison_new_predicted_DTW_resampling_both.png')
        # plt.close()   

        # object_pos_wrt_base = self.getMarkerWRTBase()
        # new_traj = self.parser.get_trajectory_wrt_context(new_traj, object_pos_wrt_base)
        # plt.figure()
        # plt.plot(self.parser.getCartesianPositions(new_traj))
        # # plt.plot(self.parser.getCartesianPositions(pred_aligned), color='red', label='predicted')
        # plt.title("Relative trajectory")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("position [m]")
        # plt.grid()
        # plt.legend()
        # plt.savefig('/home/fmeccanici/Documents/thesis/figures/debug_refinement/new_traj_relative.png')
        # plt.close()   

        # print("time vector after dtw = " + str([x[-1] for x in new_traj]))

        context = [req.trajectory.object_position.x, req.trajectory.object_position.y, req.trajectory.object_position.z]
        new_traj_msg = self.parser.predicted_trajectory_to_prompTraj_message(new_traj, context)
        # rospy.loginfo("context = " + str(context))
        # print("new_traj_msg = " + str(new_traj_msg.times))
        response = RefineTrajectoryResponse()
        response.refined_trajectory = new_traj_msg
        
        obstacle_hit_msg = Bool()
        
        obstacle_hit_msg.data = self.obstacle_hit_once

        response.obstacle_hit = obstacle_hit_msg

        return response
    
    def run(self):
        pass
        # self.goToInitialPose()
        # self.calibrate_master_pose_for_normalization()
        # self.clear_trajectories_rviz()

        # n_pred = 10
        # n_exec = 100

        # alpha = 1
        # refine_counter = 0

        # # refinement loop
        # while not rospy.is_shutdown() and self.grey_button_toggle == 0:
            
        #     # set object position
        #     object_position = Point()
        #     object_position.x = 0.78
        #     object_position.y = -0.0231
        #     object_position.z = 1

        #     self.set_object_position_client(object_position)

        #     # if this is the first time we are running the loop, make prediction
        #     # else we keep refining the current prediction

        #     if refine_counter % 2 == 0:

        #         # set context
        #         context = self.getMarkerWRTBase()
                
        #         context_msg = Point()
        #         context_msg.x = context[0]
        #         context_msg.y = context[1]
        #         context_msg.z = context[2]

        #         # use server to get the prediction from the lfd_node
        #         prediction = self.make_prediction_client(context_msg)

        #         # make list from this message
        #         prediction = self.prompTrajMessage_to_correct_format(prediction)
                
        #         # resample to n 
        #         prediction_resampled, dt = self.resampler.interpolate_learned_keypoints(prediction, n_exec)

        #         refine_counter += 1

        #     # visualize both resampled and keypoints
        #     self.visualize_trajectory(prediction, r=0, g=0, b=1)
        #     self.visualize_trajectory(prediction_resampled, r=1, g=0, b=0)

        #     # refine trajectory
        #     traj_refined = self.refineTrajectory(prediction_resampled, dt)
        #     traj_new, dt_new = self.determineNewTrajectory(prediction_resampled, traj_refined, alpha)
            
        #     # visualize new trajectory
        #     self.visualize_trajectory(traj_new, r=0, g=1, b=0)

        #     # execute reversed trajectory
        #     traj_refined_reversed = self.reverseTrajectory(traj_refined)
        #     self.executeTrajectory(traj_refined_reversed, dt_new)

        #     if input("Satisfied with this trajectory? 1/0: ") == "1": 
        #         rospy.loginfo("Adding trajectory to model...")
        #         traj_add = self.resampler.interpolate_predicted_trajectory(traj_new, n_pred)

        #         self.add_demonstration_client(traj_add)

        #         # make trajectory relative
        #         # traj_add_for_learning = self.parser.get_trajectory_wrt_object(traj_add)

        #         refine_counter += 1
        #     else:
        #         if input("Use refined or predicted trajectory for refinement? 1/0") == 1:
        #             prediction_resampled = traj_new
        #         else: pass
            
        #     self.clear_trajectories_rviz()

if __name__ == "__main__":
    node = trajectoryRefinement()
    
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        node.run()
        r.sleep()
       
