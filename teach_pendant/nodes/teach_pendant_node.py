#!/usr/bin/env python3.5

import rospy, keyboard, os
from std_msgs.msg import Bool
from pynput.keyboard import Key, Listener, KeyCode
import threading, pynput
from teleop_control.msg import Keyboard

from geometry_msgs.msg import PoseStamped, Pose
from pyquaternion import Quaternion
import numpy as np
import time
from scipy.interpolate import interp1d, InterpolatedUnivariateSpline

class KeyboardControl():
    def __init__(self):
        rospy.init_node('teach_pendant')
        # self.up_pub_ = rospy.Publisher('keyboard/up', Bool, queue_size=10)
        # self.down_pub_ = rospy.Publisher('keyboard/down', Bool, queue_size=10)
        self.end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)
        self._end_effector_pose_sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self._end_effector_pose_callback)

        self.keyboard_pub_ = rospy.Publisher('teach_pendant', Keyboard, queue_size=10)
        self.keyboard = Keyboard()
        
        # [x_n y_n z_n qx_n qy_n qz_n qw_n] 
        self.waypoints = []
        self.ee_pose = Pose()

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

    def _end_effector_pose_callback(self, data):

        self.ee_pose = data.pose
            
        q_current = Quaternion(self.ee_pose.orientation.w, self.ee_pose.orientation.x, 
                        self.ee_pose.orientation.y, self.ee_pose.orientation.z)
        
        q_rotation = self.quaternion_rotation('', 0)
        
        translation = 0.05
        rotation = 5
        
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
            print(self.ee_pose.position.z)

        elif self.keyboard.key.data == 'd':
            self.ee_pose.position.z -= translation

        elif self.keyboard.key.data == 'r':
            q_rotation = self.quaternion_rotation('x', rotation)
        elif self.keyboard.key.data == 'f':
            q_rotation = self.quaternion_rotation('x', -rotation)
        elif self.keyboard.key.data == 't':
            q_rotation = self.quaternion_rotation('y', rotation)
        elif self.keyboard.key.data == 'g':
            q_rotation = self.quaternion_rotation('y', -rotation)
        elif self.keyboard.key.data == 'y':
            q_rotation = self.quaternion_rotation('z', rotation)
        elif self.keyboard.key.data == 'h':
            q_rotation = self.quaternion_rotation('z', -rotation)

        q_rotated = q_current * q_rotation
        

        self.ee_pose.orientation.w = q_rotated[0]
        self.ee_pose.orientation.x = q_rotated[1]
        self.ee_pose.orientation.y = q_rotated[2]
        self.ee_pose.orientation.z = q_rotated[3]



        pose_publish = PoseStamped()
        pose_publish.pose = self.ee_pose
        pose_publish.header.stamp = rospy.Time.now()

        self.end_effector_goal_pub.publish(pose_publish)
    
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

        self.trajectory = ynew

    def pub_keys(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.keyboard_pub_.publish(self.keyboard)
            if self.keyboard.key.data == 'space':
                self.waypoints.append(self.ee_pose)
            if self.keyboard.key.data == 'enter':
                self.interpolate()

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
        elif key == KeyCode(char = 'w'):
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
        ros_thread = threading.Thread(target=self.pub_keys, args = ())
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