#!/usr/bin/env python2.7

import rospy, rospkg, tf
from geomagic_touch_m.msg import GeomagicButtonEvent
from slave_control.msg import ControlState
from os import listdir
from os.path import isfile, join
import os, stat, time
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from scipy.interpolate import interp1d

from learning_from_demonstration.trajectory_parser import trajectoryParser
from aruco_msgs.msg import MarkerArray

# import cProfile
# from profilehooks import profile

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
        self.parser = trajectoryParser()
        
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



        # self.marker_pose = Pose()
        # self.current_slave_pose = Pose()

    def _get_parameters(self):
        folder = rospy.get_param('~folder')
        self.path = self.rospack.get_path('learning_from_demonstration') + "/data/raw/" + str(folder) + "/"
        print("Storing path set to: " + str(self.path))

        self.button_source = rospy.get_param('~button_source')
        print("Button source set to: " + str(self.button_source))

    def _marker_detection_callback(self, data):
        
        for marker in data.markers:
            if marker.id == 582:
                self.marker_pose = marker.pose.pose
            else: continue

    def _end_effector_pose_callback(self, data):
        self.current_slave_pose = data.pose

        if self.white_button_toggle_previous == 0 and self.white_button_toggle == 1:
            print("Appending trajectory")

            data.header.stamp = rospy.Time.now()


            # marker x and y seem to be flipped wrt base_footprint
            self.EEtrajectory.append([data.pose.position.x,data.pose.position.y,data.pose.position.z,
             data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w,
             self.marker_pose.position.y, self.marker_pose.position.x, self.marker_pose.position.z,
             self.marker_pose.orientation.x, self.marker_pose.orientation.y, self.marker_pose.orientation.z, self.marker_pose.orientation.w, 
             data.header.stamp.secs, data.header.stamp.nsecs])

            
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
        teaching_node.goToInitialPose()
        r = rospy.Rate(30)
        while not rospy.is_shutdown():

            if self.white_button_toggle_previous == 1 and self.white_button_toggle == 0:
                print("Saving trajectory data")
                
                
                # "/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/data/raw/one_plane"

                print("file_name = " + self._get_trajectory_file_name(self.path))
                file_name = self._get_trajectory_file_name(self.path)
                self._save_data(self.path, file_name)
                del self.EEtrajectory[:]
                teaching_node.goToInitialPose()

                # set to 0 to prevent multiple savings
                self.white_button_toggle_previous = 0


            r.sleep()

if __name__ == "__main__":
    teaching_node = trajectoryTeaching()
    # try:
    teaching_node.run()
    # except Exception as e: 
    #     print(e)