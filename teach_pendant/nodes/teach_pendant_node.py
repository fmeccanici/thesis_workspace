#!/usr/bin/env python3.5

import rospy, keyboard, os, rospkg
from std_msgs.msg import Bool
from pynput.keyboard import Key, Listener, KeyCode
import threading, pynput
from teleop_control.msg import Keyboard

from geometry_msgs.msg import PoseStamped, Pose
from pyquaternion import Quaternion
import numpy as np
import time, stat
from scipy.interpolate import interp1d, InterpolatedUnivariateSpline
from learning_from_demonstration.srv import GetEEPose, AddDemonstration, GetObjectPosition, GetContext
from learning_from_demonstration_python.trajectory_parser import trajectoryParser
from std_msgs.msg import String
from os import listdir
from os.path import isfile, join

class KeyboardControl():
    def __init__(self):
        rospy.init_node('teach_pendant')
        self.end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)

        self.keyboard_pub_ = rospy.Publisher('teach_pendant', Keyboard, queue_size=10)
        self.keyboard = Keyboard()
        
        self.parser = trajectoryParser()
        self.trajectory = []
    
        # [x_n y_n z_n qx_n qy_n qz_n qw_n] 
        self.waypoints = []
        self.ee_pose = Pose()
        self._rospack = rospkg.RosPack()

        # get current ee pose
        try:
            rospy.wait_for_service('get_ee_pose', timeout=2.0)
            get_ee_pose = rospy.ServiceProxy('get_ee_pose', GetEEPose)
            resp = get_ee_pose()
            self.ee_pose = resp.pose

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def quaternion_rotation(self, axis, angle):
        w = np.cos(angle/2)
        x = 0
        y = 0
        z = 0

        if axis == 'x':
            x = np.sin(angle/2)
        elif axis == 'y':
            y = np.sin(angle/2)
        elif axis == 'z':
            z = np.sin(angle/2)
              
        return Quaternion(w, x, y, z).normalised
    
    def interpolate_quaternions(self, qstart, qend, n, include_endpoints=True):
        q1 = Quaternion(a=qstart[3], b=qstart[0], c=qstart[1], d=qstart[2])
        q2 = Quaternion(a=qend[3], b=qend[0], c=qend[1], d=qend[2])

        return Quaternion.intermediates(q1, q2, n, include_endpoints=include_endpoints)
    
    def interpolate(self):
        n = 100
        T = 10
        x = np.linspace(0, T, len(self.waypoints))
        x_desired = np.linspace(0, T, n)

        cartx = [data.position.x for data in self.waypoints]
        carty = [data.position.y for data in self.waypoints]
        cartz = [data.position.z for data in self.waypoints]

        qstart = [self.waypoints[0].orientation.w, self.waypoints[0].orientation.x, self.waypoints[0].orientation.y, 
                self.waypoints[0].orientation.z]

        qend = [self.waypoints[-1].orientation.w, self.waypoints[-1].orientation.x, self.waypoints[-1].orientation.y, 
                self.waypoints[-1].orientation.z]

        splinex = InterpolatedUnivariateSpline(x, cartx)
        spliney = InterpolatedUnivariateSpline(x, carty)
        splinez = InterpolatedUnivariateSpline(x, cartz)
        
        cartx_new = splinex(x_desired)
        carty_new = spliney(x_desired)
        cartz_new = splinez(x_desired)

        interpol_pred_traj = []
        for i,q in enumerate(self.interpolate_quaternions(qstart, qend, n, False)):
            pose = [cartx_new[i], carty_new[i], cartz_new[i], q[1], q[2], q[3], q[0]]
            
            ynew = pose + [x_desired[i]]

            self.trajectory.append(ynew)
    
    def teach_loop(self):
        q_current = Quaternion(self.ee_pose.orientation.w, self.ee_pose.orientation.x, 
                        self.ee_pose.orientation.y, self.ee_pose.orientation.z)
        
        q_rotation = self.quaternion_rotation('', 0)
        
        translation = 0.01
        rotation = 0.1
        
        if self.keyboard.key.data == 'q':
            self.ee_pose.position.x += translation
        
        elif self.keyboard.key.data == 'a':
            self.ee_pose.position.x -= translation
         
        elif self.keyboard.key.data == 'w':
            self.ee_pose.position.y += translation
        
        elif self.keyboard.key.data == 's':
            self.ee_pose.position.y -= translation

        elif self.keyboard.key.data == 'e':
            self.ee_pose.position.z += translation

        elif self.keyboard.key.data == 'd':
            self.ee_pose.position.z -= translation

        elif self.keyboard.key.data == 'r':
            # q_rotation = Quaternion(axis=np.array([1.0, 0.0, 0.0]), angle=rotation)
            q_rotation = self.quaternion_rotation('x', rotation)
        
        elif self.keyboard.key.data == 'f':
            # q_rotation = Quaternion(axis=np.array([1.0, 0.0, 0.0]), angle=-rotation)
            q_rotation = self.quaternion_rotation('x', -rotation)
        
        elif self.keyboard.key.data == 't':
            # q_rotation = Quaternion(axis=np.array([0.0, 1.0, 0.0]), angle=rotation)
            q_rotation = self.quaternion_rotation('y', rotation)

        elif self.keyboard.key.data == 'g':
            q_rotation = self.quaternion_rotation('y', -rotation)
            # q_rotation = Quaternion(axis=np.array([0.0, 1.0, 0.0]), angle=-rotation)

        elif self.keyboard.key.data == 'y':
            # q_rotation = Quaternion(axis=np.array([0.0, 0.0, 1.0]), angle=rotation)
            q_rotation = self.quaternion_rotation('z', rotation)
        
        elif self.keyboard.key.data == 'h':
            # q_rotation = Quaternion(axis=np.array([0.0, 0.0, 1.0]), angle=-rotation)

            q_rotation = self.quaternion_rotation('z', -rotation)

        q_rotated = q_current * q_rotation
        
        self.ee_pose.orientation.w = q_rotated[0]
        self.ee_pose.orientation.x = q_rotated[1]
        self.ee_pose.orientation.y = q_rotated[2]
        self.ee_pose.orientation.z = q_rotated[3]

        pose_publish = PoseStamped()
        pose_publish.pose = self.ee_pose
        pose_publish.header.stamp = rospy.Time.now()
        pose_publish.header.frame_id = 'base_footprint'
        self.end_effector_goal_pub.publish(pose_publish)

    # saving data in folder
    def _save_data(self, path, file_name):
        with open(path+file_name, 'w+') as f:
            f.write(str(self.trajectory))
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

    def ros_loop(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.keyboard_pub_.publish(self.keyboard)
            self.teach_loop()

            if self.keyboard.key.data == 'space':
                self.waypoints.append(self.ee_pose)
            if self.keyboard.key.data == 'enter':
                self.interpolate()
                
                rospy.wait_for_service('get_object_position', timeout=2.0)

                reference_frame = String()
                reference_frame.data = 'base'
                get_object = rospy.ServiceProxy('get_object_position', GetObjectPosition)

                resp = get_object(reference_frame)
                object_wrt_base = resp.object_position

                try:
                    rospy.wait_for_service('get_context', timeout=2.0)
                except (rospy.ServiceException, rospy.ROSException) as e:
                    print("Service call failed: %s" %e)   

                get_context = rospy.ServiceProxy('get_context', GetContext)
                resp = get_context()
                self.context = resp.context

                try:
                    rospy.wait_for_service('add_demonstration', timeout=2.0)
                
                except (rospy.ServiceException, rospy.ROSException) as e:
                    print("Service call failed: %s" %e)
                
                trajectory_wrt_object = self.parser.get_trajectory_wrt_context(self.trajectory, self.parser.point_to_list(object_wrt_base))

                rospy.loginfo(trajectory_wrt_object)

                add_demonstration = rospy.ServiceProxy('add_demonstration', AddDemonstration)
                trajectory_wrt_object_msg = self.parser.predicted_trajectory_to_prompTraj_message(trajectory_wrt_object, self.parser.point_to_list(self.context))

                resp = add_demonstration(trajectory_wrt_object_msg)
            
                raw_path = self._rospack.get_path('teach_pendant') + "/data/"
                file_name = self._get_trajectory_file_name(raw_path)
                self._save_data(raw_path, file_name)

                self.EEtrajectory = []

            r.sleep()
    
    def on_press(self, key):
        # publishing is need for refinement node
        if key == KeyCode(char = 'q'):
            self.keyboard.key.data = 'q'
        elif key == KeyCode(char = 'a'):
            self.keyboard.key.data = 'a' 

        elif key == KeyCode(char = 'w'):
            self.keyboard.key.data = 'w'     
        elif key == KeyCode(char = 's'):
            self.keyboard.key.data = 's'  

        elif key == KeyCode(char = 'e'):
            self.keyboard.key.data = 'e'     
        elif key == KeyCode(char = 'd'):
            self.keyboard.key.data = 'd'     
        elif key == KeyCode(char = 'r'):
            self.keyboard.key.data = 'r'     
        elif key == KeyCode(char = 'f'):
            self.keyboard.key.data = 'f'     
        elif key == KeyCode(char = 't'):
            self.keyboard.key.data = 't'     
        elif key == KeyCode(char = 'g'):
            self.keyboard.key.data = 'g'     
        elif key == KeyCode(char = 'y'):
            self.keyboard.key.data = 'y'     
        elif key == KeyCode(char = 'h'):
            self.keyboard.key.data = 'h'   
        elif key == Key.space:
            self.keyboard.key.data = 'space'
        elif key == Key.enter:
            self.keyboard.key.data = 'enter'
        
    def on_release(self, key):
        if key == KeyCode(char = 'q'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'a'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'd'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 's'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'w'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'e'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'r'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'f'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 't'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'g'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'y'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'h'):
            self.keyboard.key.data = ''
        elif key == Key.space:
            self.keyboard.key.data = ''
        elif key == Key.enter:
            self.keyboard.key.data = ''

        elif key == Key.esc:
            # kill node when esc is pressed
            os.system('kill %d' % os.getpid())
            raise pynput.keyboard.Listener.StopException

    def run(self):
        ros_thread = threading.Thread(target=self.ros_loop, args = ())
        ros_thread.start()

        # Collect events until release
        with Listener(
                on_press=self.on_press,
                on_release=self.on_release) as listener:
            listener.join()

if __name__ == "__main__":
    node = KeyboardControl()
    try:
        node.run()
    except Exception as e: 
        print(e)