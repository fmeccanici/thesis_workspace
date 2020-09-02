#!/usr/bin/env python3.5

# ros related
import rospy, rospkg
from geomagic_touch_m.msg import GeomagicButtonEvent
from slave_control.msg import ControlState
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

# other classes
from os import listdir
from os.path import isfile, join
import os, stat, time
import numpy as np
from scipy.interpolate import interp1d

# my classes
from learning_from_demonstration_python.trajectory_parser import trajectoryParser
from learning_from_demonstration_python.trajectory_resampler import trajectoryResampler

from learning_from_demonstration.srv import SetTeachStateOmni, SetTeachStateOmniResponse, GetTeachStateOmni, GetTeachStateOmniResponse, GetTrajectory, GetTrajectoryResponse, ClearTrajectory, ClearTrajectoryResponse, GetContext
from experiment_variables.experiment_variables import ExperimentVariables

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
        self.teach_state = False

        self.EEtrajectory = []
        self.parser = trajectoryParser()
        self.resampler = trajectoryResampler()
        self.experiment_variables = ExperimentVariables()

        ## init ros related classes
        self.rospack = rospkg.RosPack()

        # get ros parameters
        self._get_parameters()

        # set publishers/subscribers
        self.end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)

        if self.button_source == "omni":
            self.geo_button_sub = rospy.Subscriber("geo_buttons_m", GeomagicButtonEvent, self._buttonCallback)
        elif self.button_source == "keyboard":
            self.geo_button_sub = rospy.Subscriber("keyboard", GeomagicButtonEvent, self._buttonCallback)

        self.marker_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self._marker_detection_callback)
        self.end_effector_pose_sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self._end_effector_pose_callback)
        
        self._set_teach_state_service = rospy.Service('trajectory_teaching/set_teach_state', SetTeachStateOmni, self._setTeachState)
        self._get_teach_state_service = rospy.Service('trajectory_teaching/get_teach_state', GetTeachStateOmni, self._getTeachState)
        self._get_trajectory_service = rospy.Service('trajectory_teaching/get_trajectory', GetTrajectory, self._getTrajectory)
        self._clear_trajectory_service = rospy.Service('trajectory_teaching/clear_trajectory', ClearTrajectory, self._clearTrajectory)


    def is_correct_raw_format(self, raw_traj):
        if len(raw_traj[0]) == 15:
            return True
        else:
            return False

    def convert_raw_to_correct_format(self, raw_traj):
        if self.is_correct_raw_format(raw_traj):
            print("Raw trajectory already in correct format")
        else:
            try:
                # check if time format is incorrect (secs, nsecs) instead of float
                if isinstance(raw_traj[0][14], int):
                    # time format is (secs, nsecs)
                    print("Raw trajectory contains secs/nsecs values")
                    print("Converting to float...")
                    return self.parser.convert_raw_secs_nsecs_to_float(raw_traj)
                else:
                    print("No secs/nsecs value detected")
            except IndexError:
                print("Raw trajectory has incorrect length, check if it contains essential paramaters")

    def parse_relevant_learning_data(self, traj):
        traj_relevant_data = []
        T = self.parser.get_total_time(traj)
        object_positions = traj[0][7:10]

        # T doesnt work properly --> chose dt as output
        for data in traj:
            ee_pose = data[0:7]
            traj_relevant_data.append(ee_pose + object_positions + [T] )

        return traj_relevant_data

    def convertRawToDemo(self, desired_datapoints=10):
        desired_datapoints = self.experiment_variables.desired_datapoints

        traj = self.convert_raw_to_correct_format(self.EEtrajectory)
        traj = self.parser.normalize_trajectory_time_float(traj)

        traj = self.resampler.interpolate_raw_trajectory(traj, desired_datapoints)
        
        traj = self.parse_relevant_learning_data(traj)

        return traj

    def getContext(self):
        try:
            rospy.wait_for_service('get_context', timeout=2.0)

            get_context = rospy.ServiceProxy('get_context', GetContext)
            resp = get_context()
            self.context = resp.context

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e) 

    def _getTrajectory(self, req):
        resp = GetTrajectoryResponse()

        self.getContext()

        resp.demo = self.parser.predicted_trajectory_to_prompTraj_message(self.convertRawToDemo(), self.parser.point_to_list(self.context))

        return resp

    def _clearTrajectory(self, req):
        resp = ClearTrajectoryResponse()

        self.EEtrajectory = []

        return resp

    def _getTeachState(self, req):
        resp = GetTeachStateOmniResponse()
        resp.teach_state.data = self.teach_state
        return resp

    def _setTeachState(self, req):
        self.teach_state = bool(req.teach_state.data)
        # self.white_button_previous = self.white_button
        # self.white_button = req.teach_state.data

        # if (self.white_button != self.white_button_previous) and (self.white_button == 1):
        #     self.white_button_toggle = not self.white_button_toggle
        #     self.white_button_toggle_previous = not self.white_button_toggle
        
        resp = SetTeachStateOmniResponse()

        return resp

    def _get_parameters(self):
        raw_folder = rospy.get_param('~raw_folder')
        self.path = self.rospack.get_path('learning_from_demonstration') + "/data/raw/" + str(raw_folder) + "/"
        print("Storing path set to: " + str(self.path))

        self.button_source = rospy.get_param('~button_source')
        print("Button source set to: " + str(self.button_source))

        self.is_experiment = rospy.get_param('~is_experiment')
        print("is_experiment set to " + str(self.is_experiment))

    def _marker_detection_callback(self, data):
        for marker in data.markers:
            if marker.id == 582:
                self.marker_pose = marker.pose.pose
            else: continue

    def setTeachState(self):
        if self.white_button_toggle_previous == 0 and self.white_button_toggle == 1:
            self.teach_state = True

        elif self.white_button_toggle_previous == 1 and self.white_button_toggle == 0:
            self.teach_state = False    

    def _end_effector_pose_callback(self, data):
        self.current_slave_pose = data.pose
        self.setTeachState()
        
        if self.teach_state == True:

            # print("Appending trajectory")
            data.header.stamp = rospy.Time.now()

            # marker x and y seem to be flipped wrt base_footprint
            self.EEtrajectory.append([data.pose.position.x,data.pose.position.y,data.pose.position.z,
             data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w,
             self.marker_pose.position.y, self.marker_pose.position.x, self.marker_pose.position.z,
             self.marker_pose.orientation.x, self.marker_pose.orientation.y, self.marker_pose.orientation.z, self.marker_pose.orientation.w, 
             data.header.stamp.secs, data.header.stamp.nsecs])
    
    def set_aruco_position(self, x=0.7, y=-0.43, z=1):
        state_msg = ModelState()
        state_msg.model_name = 'aruco_cube'
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = z
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
    
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
        
        fx = interp1d(t, x)
        fy = interp1d(t, y)
        fz = interp1d(t, z)

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

    def _save_data(self, path, file_name):
        with open(path+file_name, 'w+') as f:
            f.write(str(self.EEtrajectory))
        os.chmod(path+file_name,stat.S_IRWXO)
        os.chmod(path+file_name,stat.S_IRWXU)

    def _get_trajectory_file_name(self, path):
        # get existing files from folder
        files = [f for f in listdir(path) if isfile(join(path, f))]
        
        # get numbers from these files
        numbers = [int(os.path.splitext(f)[0].split('_')[-1]) for f in files]
        
        # sort them in ascending order
        numbers.sort()

        # make list of these files
        files = ["raw_trajectory_" + str(number) + ".txt" for number in numbers]
        
        try:
            # add 1 to the last trajectory number and create new name
            return "raw_trajectory_" + str(int(os.path.splitext(files[-1])[0].split('_')[-1]) + 1) + ".txt"
        except IndexError:
            # no files in folder yet --> create first file
            return "raw_trajectory_1.txt"
            
    def run(self):
        # self.goToInitialPose()
        # x = 0.75
        # y = -0.0231
        
        # self.set_aruco_position(x, y)

        r = rospy.Rate(30)
        while not rospy.is_shutdown():

            if not self.is_experiment:
                if self.white_button_toggle_previous == 1 and self.white_button_toggle == 0:
                    print("Saving trajectory data")
                    
                    
                    # "/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/data/raw/one_plane"

                    print("file_name = " + self._get_trajectory_file_name(self.path))
                    file_name = self._get_trajectory_file_name(self.path)
                    self._save_data(self.path, file_name)
                    del self.EEtrajectory[:]
                    # teaching_node.goToInitialPose()

                    # set to 0 to prevent multiple savings
                    self.white_button_toggle_previous = 0
                    
                    # place aruco box back to init position
                    # self.set_aruco_position(x, y)
                    time.sleep(1)
                else:
                    continue

            r.sleep()

if __name__ == "__main__":
    teaching_node = trajectoryTeaching()
    # try:
    teaching_node.run()
    # except Exception as e: 
    #     print(e)