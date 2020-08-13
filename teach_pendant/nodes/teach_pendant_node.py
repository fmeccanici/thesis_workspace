#!/usr/bin/env python3.5

import rospy, keyboard, os, rospkg
from std_msgs.msg import Bool
from pynput.keyboard import Key, Listener, KeyCode
import threading, pynput
from teleop_control.msg import Keyboard
from data_logger_python.text_updater import TextUpdater

from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Pose
from pyquaternion import Quaternion
import numpy as np
import time, stat
from scipy.interpolate import interp1d, InterpolatedUnivariateSpline, CubicSpline, UnivariateSpline, dfitpack
from learning_from_demonstration.srv import AddDemonstration, GetObjectPosition, GetContext, GoToPose
from learning_from_demonstration_python.trajectory_parser import trajectoryParser
from teach_pendant.srv import GetDemonstrationPendant, GetDemonstrationPendantResponse, GetTeachState, GetTeachStateResponse, SetTeachState, SetTeachStateResponse, GetEEPose, AddWaypoint, AddWaypointResponse, ClearWaypoints, ClearWaypointsResponse

from std_msgs.msg import String, Bool
from os import listdir
from os.path import isfile, join
from trajectory_visualizer.srv import VisualizeTrajectory, ClearTrajectories
from trajectory_visualizer.msg import TrajectoryVisualization
from execution_failure_detection.msg import ExecutionFailure

from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

class KeyboardControl():
    def __init__(self):
        rospy.init_node('teach_pendant')
        self.debug_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/data/debug/'
        self.end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)
        self._get_demonstration_service = rospy.Service('get_demonstration_pendant', GetDemonstrationPendant, self._get_demonstration)
        self._get_teach_state_service = rospy.Service('/offline_pendant/get_teach_state', GetTeachState, self._getTeachState)
        self._set_teach_state_service = rospy.Service('/offline_pendant/set_teach_state', SetTeachState, self._setTeachState)
        self._add_waypoint_service = rospy.Service('/offline_pendant/add_waypoint', AddWaypoint, self._addWaypoint)
        self._clear_waypoints_service = rospy.Service('/offline_pendant/clear_waypoints', ClearWaypoints, self._clearWaypoints)

        self.text_updater = TextUpdater()

        self.execution_failure_sub = rospy.Subscriber("/execution_failure", ExecutionFailure, self._executionFailureCallback)

        self.keyboard_pub_ = rospy.Publisher('teach_pendant', Keyboard, queue_size=10)
        self.keyboard = Keyboard()
        
        self.parser = trajectoryParser()
        self.EEtrajectory = []
    
        # [x_n y_n z_n qx_n qy_n qz_n qw_n] 
        self.waypoints = []
        self.ee_pose = Pose()
        self._rospack = rospkg.RosPack()

        self.teach_state = False

        # self.listener = tf.TransformListener()

        # get current ee pose
        # try:
        #     # (trans,rot) = self.listener.lookupTransform('base_footprint', 'arm_tool_link', rospy.Time(0))
            
        #     rospy.wait_for_service('get_ee_pose', timeout=2.0)
        #     get_ee_pose = rospy.ServiceProxy('get_ee_pose', GetEEPose)
        #     resp = get_ee_pose()
        #     self.ee_pose = resp.pose
            

        #     ee_pose = [self.ee_pose.position.x, self.ee_pose.position.y, self.ee_pose.position.z,
        #     self.ee_pose.orientation.x, self.ee_pose.orientation.y, self.ee_pose.orientation.z,
        #     self.ee_pose.orientation.w]
            
        #     # ee_pose = [trans[0], trans[1], trans[2], trans[3], trans[4], trans[5], trans[6]]

        #     self.waypoints.append(ee_pose)
        #     pose_publish = PoseStamped()
        #     pose_publish.pose = self.ee_pose
        #     pose_publish.header.stamp = rospy.Time.now()
        #     pose_publish.header.frame_id = 'base_footprint'
        #     # self.end_effector_goal_pub.publish(pose_publish)

        # except (rospy.ServiceException, rospy.ROSException) as e:
            # print("Service call failed: %s" %e)
        
        rospy.loginfo("You can start the teach pendant")
    
    def _addWaypoint(self, req):
        self.addWaypoint()
        time.sleep(2)
        resp = AddWaypointResponse()

        return resp
    
    def _clearWaypoints(self, req):
        self.clearWaypoints()

        resp = ClearWaypointsResponse()

        return resp

    def addWaypoint(self):
        self.getEEPose()

        ee_pose = [self.ee_pose.position.x, self.ee_pose.position.y, self.ee_pose.position.z,
            self.ee_pose.orientation.x, self.ee_pose.orientation.y, self.ee_pose.orientation.z,
            self.ee_pose.orientation.w]
        
        if len(self.waypoints) == 0:
            with open(self.debug_path + 'initial_waypoint.txt', 'w+') as f:
                f.write(str(ee_pose))
        
        self.waypoints.append(ee_pose)

    def getEEPose(self):
        try:
            rospy.wait_for_service('get_end_effector_pose', timeout=2.0)
            get_ee_pose = rospy.ServiceProxy('get_end_effector_pose', GetEEPose)
            resp = get_ee_pose()
            self.ee_pose = resp.pose

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
        

    def _executionFailureCallback(self, data):
        self.object_reached = data.object_reached.data
        self.obstacle_hit = data.obstacle_hit.data

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
    
    def _getTeachState(self, req):
        resp = GetTeachStateResponse()
        resp.teach_state = Bool(self.teach_state)
        return resp

    def clearWaypoints(self):
        self.waypoints = []

    def _setTeachState(self, req):
        if req.teach_state.data == True:
            self.getEEPose()

        self.teach_state = req.teach_state.data

        resp = SetTeachStateResponse()

        return resp

    def _get_demonstration(self, req):

        resp = GetDemonstrationPendantResponse()
        resp.demo = self.parser.predicted_trajectory_to_prompTraj_message(self.EEtrajectory, self.parser.point_to_list(self.context))

        return resp
 
    def interpolate(self):
        n = 75
        T = 10

        x = np.linspace(0, T, len(self.waypoints))
        x_desired = np.linspace(0, T, n)

        cartx = [data[0] for data in self.waypoints]
        carty = [data[1] for data in self.waypoints]
        cartz = [data[2] for data in self.waypoints]
        q = [ [data[3], data[4], data[5], data[6]] for data in self.waypoints]
        
        qstart = [self.waypoints[0][3], self.waypoints[0][4], 
                self.waypoints[0][5], self.waypoints[0][6]]

        qend = [self.waypoints[-1][3], self.waypoints[-1][4], 
                self.waypoints[-1][5], self.waypoints[-1][6]]

        # splinex = InterpolatedUnivariateSpline(x, cartx)
        # spliney = InterpolatedUnivariateSpline(x, carty)
        # splinez = InterpolatedUnivariateSpline(x, cartz)

        # splinex = CubicSpline(x, cartx)
        # spliney = CubicSpline(x, carty)
        # splinez = CubicSpline(x, cartz)

        splinex = interp1d(x, cartx, kind='quadratic')
        spliney = interp1d(x, carty, kind='quadratic')
        splinez = interp1d(x, cartz, kind='quadratic')

        # splinex = UnivariateSpline(x, cartx)
        # spliney = UnivariateSpline(x, carty)
        # splinez = UnivariateSpline(x, cartz)

        cartx_new = splinex(x_desired)
        carty_new = spliney(x_desired) 
        cartz_new = splinez(x_desired)

        slerp = Slerp(x, R.from_quat(q))
        interp_rots = slerp(x_desired)

        self.EEtrajectory = []

        for i, data in enumerate(cartx_new):
            demo = [cartx_new[i], carty_new[i], cartz_new[i]] + list(interp_rots[i].as_quat())
            self.EEtrajectory.append( demo )

        # include_endpoints = True

        # if include_endpoints == True:
        #     n_slerp = n - 2

        # with open(self.debug_path + 'cartz_new.txt', 'w+') as f:
        #     f.write(str(cartz_new))
        
        # with open(self.debug_path + 'cartz.txt', 'w+') as f:
        #     f.write(str(cartz))

        # for i,q in enumerate(self.interpolate_quaternions(qstart, qend, n_slerp, include_endpoints)):
        #     pose = [cartx_new[i], carty_new[i], cartz_new[i], q[1], q[2], q[3], q[0]]
        #     ynew = pose + [x_desired[i]]
        #     self.EEtrajectory.append(ynew)

        
        # with open(self.debug_path + 'interp_waypoints.txt', "w") as f:
        #     f.write(str(self.EEtrajectory))
        
        # with open(self.debug_path + 'final_waypoints.txt', "w") as f:
        #     f.write(str(self.waypoints))

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
    
    def goToInitialPose(self):
        pose = Pose()
        try:
            pose.position.x = 0.609
            # pose.position.y = -0.306
            pose.position.y = -0.290


            pose.position.z = 0.816

            pose.orientation.x = 0.985
            pose.orientation.y = -0.103
            pose.orientation.z = -0.124
            pose.orientation.w = 0.064
            
            rospy.wait_for_service('go_to_pose', timeout=2.0)
            go_to_pose = rospy.ServiceProxy('go_to_pose', GoToPose)
            resp = go_to_pose(pose)
        
        except ValueError:
            rospy.loginfo("Make sure you set a pose!")
        
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def ros_loop(self):
        r = rospy.Rate(30)
        self.keyboard_toggle = 1

        while not rospy.is_shutdown():

            if self.teach_state == True:
                self.teach_loop()

                if self.keyboard.key.data == 'space' and self.keyboard_toggle == 1:

                    self.addWaypoint()
                    self.text_updater.update("STORED WAYPOINT " + str(len(self.waypoints)-1))

                    self.keyboard_toggle = 0

                elif self.keyboard.key.data == 'space_released' and self.keyboard_toggle == 0:
                    self.keyboard_toggle = 1

                elif self.keyboard.key.data == 'enter':
                    self.text_updater.update("GENERATING TRAJECTORY")

                    # catch fitpack error
                    try:    
                        dfitpack.sproot(-1, -1, -1)    
                    except Exception as e:
                        dfitpack_error = type(e)
                    try:
                        self.interpolate()
                    except dfitpack_error:
                        print("Not enough waypoints")

                    try: 
                        rospy.wait_for_service('get_context', timeout=2.0)
                        get_context = rospy.ServiceProxy('get_context', GetContext)
                        resp = get_context()
                        
                        self.context = resp.context

                        # empty waypoints
                        self.waypoints = []

                        self.teach_state = False

                    except (rospy.ServiceException, rospy.ROSException) as e:
                        print("Service call failed: %s" %e) 
                    

                
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
            self.keyboard.key.data = 'space_released'
        elif key == Key.enter:
            self.keyboard.key.data = ''

        elif key == Key.ctrl_r:
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