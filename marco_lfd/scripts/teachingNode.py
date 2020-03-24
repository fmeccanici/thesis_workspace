#!/usr/bin/env python

import rospy
from geomagic_touch_m.msg import GeomagicButtonEvent
from slave_control.msg import ControlState
from os import listdir
from os.path import isfile, join
import os, stat
from geometry_msgs.msg import PoseStamped

class teachingNode():
    def __init__(self):
        geo_button_sub = rospy.Subscriber("geo_buttons_m", GeomagicButtonEvent, self._buttonCallback)
        slave_control_state_sub = rospy.Subscriber("slave_control_state", ControlState, self._slave_control_state_callback)
        self._slave_end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)
        
        self.grey_button = 0
        self.grey_button_previous = 0
        self.white_button = 0
        self.white_button_previous = 0
        self.EEtrajectory = []

    def init_teaching_pose(self):
        rospy.loginfo("Initializing teaching pose")
        init_pose = PoseStamped()
        init_pose.pose.position.x = 0.635182905421
        init_pose.pose.position.y = -0.11908358959
        init_pose.pose.position.z = 0.904816137971
        init_pose.pose.orientation.x = 0.994780396556
        init_pose.pose.orientation.y = 0.100705866371
        init_pose.pose.orientation.z = 0.0157496554368
        init_pose.pose.orientation.w = 0.00471587310671
        init_pose.header.frame_id = "/base_footprint"
        init_pose.header.seq = 1
        init_pose.header.stamp = rospy.Time.now()
        rospy.loginfo(init_pose)
        self._slave_end_effector_goal_pub.publish(init_pose)
    
    def _buttonCallback(self, data):
        self.grey_button_previous = self.grey_button 
        self.grey_button = data.grey_button
        self.white_button_previous = self.white_button
        self.white_button = data.white_button
        
    def _demonstrationStage(self):

        if (self.white_button == 0) and (self.white_button_previous) == 1:
            # print("Demonstration finished")
            return 0
        elif (self.white_button == 1) and (self.white_button_previous == 1):
            # print("Still demonstrating")
            return 1
        elif (self.white_button == 1) and (self.white_button_previous == 0):
            # print("Starting demonstration")
            return 2
        elif (self.white_button == 0) and (self.white_button_previous == 0):
            # print("Not demonstrating")
            return 3

    def _slave_control_state_callback(self,data):

        if self._demonstrationStage() == 0 or self._demonstrationStage() == 1 or self._demonstrationStage() == 2:
            print("Appending trajectory")
            data.slave_pose.header.stamp.secs = data.header.stamp.secs
            data.slave_pose.header.stamp.nsecs = data.header.stamp.nsecs
            self.EEtrajectory.append([data.slave_pose.pose.position.x,data.slave_pose.pose.position.y,data.slave_pose.pose.position.z, data.slave_pose.pose.orientation.x,data.slave_pose.pose.orientation.y,data.slave_pose.pose.orientation.z,data.slave_pose.pose.orientation.w, data.header.stamp.secs, data.header.stamp.nsecs])
            
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
        print(files)
        if os.stat(path + files[-1]).st_size == 0:
            return files[-1]
        else:
            return "raw_trajectory_" + str(int(os.path.splitext(files[-1])[0].split('_')[-1]) + 1) + ".txt"

    def run(self):
        if self._demonstrationStage() == 0:
            print("Saving trajectory data")
            # path = "~/shared_docker/marco_fix_ws/marco_ws/src/marco_lfd/teaching/data/"
            path = "/home/fmeccanici/Documents/thesis/lfd_ws/src/marco_lfd/data/raw/"

            print("file_name = " + self._get_trajectory_file_name(path))
            file_name = self._get_trajectory_file_name(path)
            self._save_data(path, file_name)
            del self.EEtrajectory[:]


if __name__ == "__main__":
    rospy.init_node("teaching")
    # try:
    teaching_node = teachingNode()

    for i in range(100):
        teaching_node.init_teaching_pose()

    while not rospy.is_shutdown():
        teaching_node.run()
        rospy.rostime.wallsleep(0.00001)
    # except Exception as e: 
    #     print(e)