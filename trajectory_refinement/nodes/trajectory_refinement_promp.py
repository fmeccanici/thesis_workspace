#!/usr/bin/env python3.5

# import external python packages
import rospy, time, os, rospkg
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from pyquaternion import Quaternion

# import ros messages
from learning_from_demonstration.srv import AddDemonstration, AddDemonstrationResponse, MakePrediction, MakePredictionResponse, SetObject, SetObjectResponse
from learning_from_demonstration.msg import prompTraj
from geomagic_touch_m.msg import GeomagicButtonEvent
from master_control.msg import ControlComm
from lfd_msgs.msg import TrajectoryVisualization
from geometry_msgs.msg import PoseStamped, WrenchStamped, PoseArray, Pose, Point
from aruco_msgs.msg import MarkerArray


# import own python packages
from trajectory_visualizer_python.trajectory_visualizer_python import trajectoryVisualizer
from learning_from_demonstration.trajectory_resampler import trajectoryResampler
from learning_from_demonstration.dynamic_time_warping import DTW
from learning_from_demonstration.trajectory_parser import trajectoryParser


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
            self.geo_button_sub = rospy.Subscriber("keyboard", GeomagicButtonEvent, self._buttonCallback)


        self.end_effector_pose_sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self._end_effector_pose_callback)
        self.marker_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self._marker_detection_callback)
        self.master_pose_sub = rospy.Subscriber('master_control_comm', ControlComm, self._masterPoseCallback)
        
        # initialize other classes
        self.parser = trajectoryParser()
        self.resampler = trajectoryResampler()
        self.visualizer = trajectoryVisualizer()
        self.dtw = DTW()


    
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
            slave_goal.header.frame_id = self.base_frame

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

        t = list(self.parser.secs_nsecs_to_float_vector(self.parser.durationVector2secsNsecsVector(t)))
        print(t)
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

            slave_goal.pose.orientation.x = 0.980837824843
            slave_goal.pose.orientation.y = -0.00365989846539
            slave_goal.pose.orientation.z = -0.194791016723
            slave_goal.pose.orientation.w = 0.000475714270521

            slave_goal.header.seq = 0
            slave_goal.header.frame_id = self.base_frame

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

            # add position of traj[i] wrt base
            if i <= len(traj_pos)-1:
                vnew = np.add(p, traj_pos[i])
            else:
                vnew = np.add(p, traj_pos[-1])

            object_position = self.parser.get_object_position(traj)
            ee_position = list(vnew)
            t_list = [t]
            
            # append refined trajectory
            if i <= len(traj_pos)-1:
                ee_orientation = [traj[i][3], traj[i][4], traj[i][5], traj[i][6]]
                refined_traj.append(ee_position + ee_orientation + object_position + t_list)
            else:
                ee_orientation = [traj[-1][3], traj[-1][4], traj[-1][5], traj[-1][6]]

                refined_traj.append(ee_position + ee_orientation + object_position + t_list)
            

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

            time.sleep(dt)
            t += dt
            i += 1
        

        return refined_traj


    # from Ewerton: tau_D^new = tau_D^old + alpha * (tau_HR - tau_R)
    def determineNewTrajectory(self, pred_traj, refined_traj, alpha = 1):
        
        # quaternions used for interpolation
        qstart = pred_traj[0][3:7]
        qend = pred_traj[-1][3:7]  
        
        # object position
        object_position = pred_traj[0][7:10] 

        T_refined = self.parser.get_total_time(refined_traj)
        T_pred = self.parser.get_total_time(pred_traj)

        y_pred, y_ref = self.resampler.match_refined_predicted(pred_traj, refined_traj)
        y_refined_aligned, y_pred_aligned = self.dtw.apply_dtw(y_ref, y_pred)



        plt.plot(y_refined_aligned)
        plt.plot(y_pred_aligned)

        plt.title('Predicted and refined trajectory after DTW')
        plt.xlabel('datapoint [-]')
        plt.ylabel('position [m]')
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
            refined_traj = ref_pos + ref_ori + object_position + ref_t

            pred_pos = [list(y_pred_aligned[i])[0], list(y_pred_aligned[i])[1], list(y_pred_aligned[i])[2]]
            pred_ori = [q[1], q[2], q[3], q[0]]
            pred_t = [t_pred[i]]

            pred_traj = pred_pos + pred_ori + object_position + pred_t


            # tau_D^new = tau_D^old + alpha * (tau_HR - tau_R)
            new_trajectory.append(list(np.add(np.asarray(pred_pos), alpha * (np.subtract(np.asarray(ref_pos), np.asarray(pred_pos)) ))) + refined_traj[3:])
        
        dt_new = t_refined[1]

        return new_trajectory, dt_new

    def set_object_position_client(self, object_position):
        rospy.wait_for_service('set_object')
        try:
            set_object = rospy.ServiceProxy('set_object', SetObject)
            resp = set_object(object_position)
            return resp.success

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    def predicted_trajectory_to_prompTraj_message(self, traj):
        t_list = []
        message = prompTraj()
        message.object_position.x = traj[0][7]
        message.object_position.y = traj[0][8]
        message.object_position.z = traj[0][9]

        for data in traj:
            # message.end_effector_pose.header.stamp = rospy.Duration(secs=data[-2], nsecs=data[-1])
            ee_pose = Pose()
            ee_pose.position.x = data[0]
            ee_pose.position.y = data[1]
            ee_pose.position.z = data[2]
            ee_pose.orientation.x = data[3]
            ee_pose.orientation.y = data[4]
            ee_pose.orientation.z = data[5]
            ee_pose.orientation.w = data[6]

            message.poses.append(ee_pose)
            
            t_float = data[-1]
            # message.times.append([t_float])
            t_list += [t_float]

        message.times = t_list

        return message
    def add_demonstration_client(self, demo):
        rospy.wait_for_service('add_demonstration')
        try:
            add_demonstration = rospy.ServiceProxy('add_demonstration', AddDemonstration)
            demo = self.predicted_trajectory_to_prompTraj_message(demo)
            resp = add_demonstration(demo)
            return resp.success

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def make_prediction_client(self, context):
        rospy.wait_for_service('make_prediction')
        try:
            make_prediction = rospy.ServiceProxy('make_prediction', MakePrediction)
            resp = make_prediction(context)
            return resp.prediction

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
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
    
    def visualize_trajectory(self, traj, r, g, b):
        for i in range(50):
            self.traj_vis_pub.publish(self.visualizer.trajToVisMsg(traj, r=r, g=g, b=b, frame_id=self.base_frame))
    
    def clear_trajectories_rviz(self):
        empty_traj = np.zeros((1, 7))
        for i in range(10):
            self.traj_vis_pub.publish(self.visualizer.trajToVisMsg(list(empty_traj), r=0, g=0, b=0))
    
    def run(self):
        self.goToInitialPose()
        self.calibrate_master_pose_for_normalization()
        self.clear_trajectories_rviz()

        n_pred = 10
        n_exec = 100

        alpha = 1
        refine_counter = 0

        # refinement loop
        while not rospy.is_shutdown() and self.grey_button_toggle == 0:
            
            # set object position
            object_position = Point()
            object_position.x = 0.8
            object_position.y = -0.0231
            object_position.z = 1

            self.set_object_position_client(object_position)

            # if this is the first time we are running the loop, make prediction
            # else we keep refining the current prediction

            if refine_counter % 2 == 0:

                # set context
                context = self.getMarkerWRTBase()
                
                context_msg = Point()
                context_msg.x = context[0]
                context_msg.y = context[1]
                context_msg.z = context[2]

                # use server to get the prediction from the lfd_node
                prediction = self.make_prediction_client(context_msg)

                # make list from this message
                prediction = self.prompTrajMessage_to_correct_format(prediction)
                
                # resample to n 
                prediction_resampled, dt = self.resampler.interpolate_learned_keypoints(prediction, n_exec)

                refine_counter += 1

            # visualize both resampled and keypoints
            self.visualize_trajectory(prediction, r=0, g=0, b=1)
            self.visualize_trajectory(prediction_resampled, r=1, g=0, b=0)

            # refine trajectory
            traj_refined = self.refineTrajectory(prediction_resampled, dt)
            traj_new, dt_new = self.determineNewTrajectory(prediction_resampled, traj_refined, alpha)
            
            # visualize new trajectory
            self.visualize_trajectory(traj_new, r=0, g=1, b=0)

            # execute reversed trajectory
            traj_refined_reversed = self.reverseTrajectory(traj_refined)
            self.executeTrajectory(traj_refined_reversed, dt_new)

            if input("Satisfied with this trajectory? 1/0: ") == "1": 
                rospy.loginfo("Adding trajectory to model...")
                traj_add = self.resampler.interpolate_predicted_trajectory(traj_new, n_pred)

                self.add_demonstration_client(traj_add)

                # make trajectory relative
                # traj_add_for_learning = self.parser.get_trajectory_wrt_object(traj_add)

                refine_counter += 1
            else:
                if input("Use refined or predicted trajectory for refinement? 1/0") == 1:
                    prediction_resampled = traj_new
                else: pass
            
            self.clear_trajectories_rviz()

if __name__ == "__main__":
    node = trajectoryRefinement()
    node.run()
       