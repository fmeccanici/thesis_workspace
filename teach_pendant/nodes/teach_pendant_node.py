#!/usr/bin/env python3.5

import rospy, keyboard, os, rospkg
from std_msgs.msg import Bool
from pynput.keyboard import Key, Listener, KeyCode
import threading, pynput
from teleop_control.msg import Keyboard

from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Pose
from pyquaternion import Quaternion
import numpy as np
import time, stat
from scipy.interpolate import interp1d, InterpolatedUnivariateSpline, CubicSpline, UnivariateSpline
from learning_from_demonstration.srv import GetEEPose, AddDemonstration, GetObjectPosition, GetContext, GoToPose
from learning_from_demonstration_python.trajectory_parser import trajectoryParser
from teach_pendant.srv import GetDemonstrationPendant, GetDemonstrationPendantResponse
from std_msgs.msg import String
from os import listdir
from os.path import isfile, join
from trajectory_visualizer.srv import VisualizeTrajectory, ClearTrajectories
from trajectory_visualizer.msg import TrajectoryVisualization

class KeyboardControl():
    def __init__(self):
        rospy.init_node('teach_pendant')
        self.end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)
        self._get_demonstration_service = rospy.Service('get_demonstration_pendant', GetDemonstrationPendant, self._get_demonstration)

        self.keyboard_pub_ = rospy.Publisher('teach_pendant', Keyboard, queue_size=10)
        self.keyboard = Keyboard()
        
        self.execution_phase = True

        self.parser = trajectoryParser()
        self.EEtrajectory = []
    
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
        rospy.loginfo("You can start the teach pendant")
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
    
    def _get_demonstration(self, req):

        resp = GetDemonstrationPendantResponse()
        resp.demo = self.parser.predicted_trajectory_to_prompTraj_message(self.EEtrajectory, self.parser.point_to_list(self.context))

        return resp
 
    def interpolate(self):
        n = 100
        T = 10

        x = np.linspace(0, T, len(self.waypoints))
        x_desired = np.linspace(0, T, n)

        cartx = [data[0] for data in self.waypoints]
        carty = [data[1] for data in self.waypoints]
        cartz = [data[2] for data in self.waypoints]

        
        qstart = [self.waypoints[0][3], self.waypoints[0][4], 
                self.waypoints[0][5], self.waypoints[0][6]]

        qend = [self.waypoints[-1][3], self.waypoints[-1][4], 
                self.waypoints[-1][5], self.waypoints[-1][6]]

        with open('/home/fmeccanici/Documents/thesis/thesis_workspace/src/teach_pendant/qstart.txt', 'w+') as f:
            f.write(str(qstart))
        
        with open('/home/fmeccanici/Documents/thesis/thesis_workspace/src/teach_pendant/qend.txt', 'w+') as f:
            f.write(str(qend))

        # splinex = InterpolatedUnivariateSpline(x, cartx)
        # spliney = InterpolatedUnivariateSpline(x, carty)
        # splinez = InterpolatedUnivariateSpline(x, cartz)

        # splinex = CubicSpline(x, cartx)
        # spliney = CubicSpline(x, carty)
        # splinez = CubicSpline(x, cartz)

        # splinex = interp1d(x, cartx, kind='quadratic')
        # spliney = interp1d(x, carty, kind='quadratic')
        # splinez = interp1d(x, cartz, kind='quadratic')

        splinex = UnivariateSpline(x, cartx)
        spliney = UnivariateSpline(x, carty)
        splinez = UnivariateSpline(x, cartz)

        cartx_new = splinex(x_desired)
        carty_new = spliney(x_desired) 
        cartz_new = splinez(x_desired)

        interpol_pred_traj = []
        self.EEtrajectory = []

        for i,q in enumerate(self.interpolate_quaternions(qstart, qend, n, False)):
            pose = [cartx_new[i], carty_new[i], cartz_new[i], q[1], q[2], q[3], q[0]]
            ynew = pose + [x_desired[i]]

            self.EEtrajectory.append(ynew)
        
        with open('/home/fmeccanici/Documents/thesis/thesis_workspace/src/teach_pendant/eval_traj.txt', 'w+') as f:
            f.write(str(self.EEtrajectory))
        print(self.waypoints)

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
            

            ### Only do offline teaching when we are not executing the trajectory
            if self.execution_phase == False:

                self.keyboard_pub_.publish(self.keyboard)
                self.teach_loop()

                if self.keyboard.key.data == 'space' and self.keyboard_toggle == 1:
                    try:
                        rospy.wait_for_service('get_ee_pose', timeout=2.0)
                        get_ee_pose = rospy.ServiceProxy('get_ee_pose', GetEEPose)
                        resp = get_ee_pose()
                        self.ee_pose = resp.pose
                        ee_pose = [self.ee_pose.position.x, self.ee_pose.position.y, self.ee_pose.position.z,
                                    self.ee_pose.orientation.x, self.ee_pose.orientation.y, self.ee_pose.orientation.z,
                                    self.ee_pose.orientation.w]

                        self.waypoints.append(ee_pose)

                    except (rospy.ServiceException, rospy.ROSException) as e:
                        print("Service call failed: %s" %e)
                    
                    self.keyboard_toggle = 0

                elif self.keyboard.key.data == 'space_released' and self.keyboard_toggle == 0:

                    self.keyboard_toggle = 1

            elif self.execution_phase == True and self.keyboard.key.data == 'space':
                # stop execution
                # set phase to false so we can teach
                self.execution_phase = False
                try:
                    rospy.wait_for_service('stop_execution', timeout=2.0)
                    stop_execution = rospy.ServiceProxy('stop_execution', Empty)
                    resp = stop_execution()
                        
                except (rospy.ServiceException, rospy.ROSException) as e:
                    print("Service call failed: %s" %e)
                

                # go to initial pose
                self.goToInitialPose()
                
                # get pose for the teach loop to not jump back
                try:
                    rospy.wait_for_service('get_ee_pose', timeout=2.0)
                    get_ee_pose = rospy.ServiceProxy('get_ee_pose', GetEEPose)
                    resp = get_ee_pose()
                    self.ee_pose = resp.pose
                    ee_pose = [self.ee_pose.position.x, self.ee_pose.position.y, self.ee_pose.position.z,
                                self.ee_pose.orientation.x, self.ee_pose.orientation.y, self.ee_pose.orientation.z,
                                self.ee_pose.orientation.w]

                    # set first waypoint
                    self.waypoints.append(ee_pose)

                except (rospy.ServiceException, rospy.ROSException) as e:
                    print("Service call failed: %s" %e)

            if self.execution_phase == False and self.keyboard.key.data == 'enter':
                self.interpolate()

                # empty waypoints
                self.waypoints = []

                try:
                    rospy.wait_for_service('get_context', timeout=2.0)
                except (rospy.ServiceException, rospy.ROSException) as e:
                    print("Service call failed: %s" %e)   

                get_context = rospy.ServiceProxy('get_context', GetContext)
                resp = get_context()
                
                self.context = resp.context
                
                try:
                    rospy.wait_for_service('visualize_trajectory', timeout=2.0)
                    rospy.wait_for_service('clear_trajectories', timeout=2.0)

                    visualize_trajectory = rospy.ServiceProxy('visualize_trajectory', VisualizeTrajectory)
                    clear_trajectories = rospy.ServiceProxy('clear_trajectories', ClearTrajectories)

                    visualization_msg = TrajectoryVisualization()

                except (rospy.ServiceException, rospy.ROSException) as e:
                    print("Service call failed: %s" %e)

                except AttributeError:
                    rospy.loginfo("No prediction made yet!")


                resp = clear_trajectories()
        
                # visualize refinement
                visualization_msg.pose_array = self.parser.predicted_trajectory_to_prompTraj_message(self.EEtrajectory, self.parser.point_to_list(self.context)).poses
                visualization_msg.r = 0
                visualization_msg.g = 1
                visualization_msg.b = 0

                resp = visualize_trajectory(visualization_msg)

                # go to initial position
                self.goToInitialPose()

                # set execution phase to true again so we can execute trajectory
                # without the end_effector publisher to intervene
                self.execution_phase = True

                try:
                    rospy.wait_for_service('stop_timer', timeout=2.0)

                    stop_timer = rospy.ServiceProxy('stop_timer', Empty)
                    visualization_msg = TrajectoryVisualization()

                    resp = stop_timer()
                    rospy.loginfo("Timer stopped using service")

                except (rospy.ServiceException, rospy.ROSException) as e:
                    print("Service call failed: %s" %e)                    
            
            
            
            elif self.execution_phase == True and self.keyboard.key.data == 'enter':
                rospy.loginfo("Executing trajectory: No interpolation possible")
                # rospy.wait_for_service('get_object_position', timeout=2.0)

                # reference_frame = String()
                # reference_frame.data = 'base'
                # get_object = rospy.ServiceProxy('get_object_position', GetObjectPosition)

                # resp = get_object(reference_frame)
                # object_wrt_base = resp.object_position

                # try:
                #     rospy.wait_for_service('get_context', timeout=2.0)
                # except (rospy.ServiceException, rospy.ROSException) as e:
                #     print("Service call failed: %s" %e)   

                # get_context = rospy.ServiceProxy('get_context', GetContext)
                # resp = get_context()
                # self.context = resp.context

                # try:
                #     rospy.wait_for_service('add_demonstration', timeout=2.0)
                
                # except (rospy.ServiceException, rospy.ROSException) as e:
                #     print("Service call failed: %s" %e)
                
                # trajectory_wrt_object = self.parser.get_trajectory_wrt_context(self.EEtrajectory, self.parser.point_to_list(object_wrt_base))

                # rospy.loginfo(trajectory_wrt_object)

                # add_demonstration = rospy.ServiceProxy('add_demonstration', AddDemonstration)
                # trajectory_wrt_object_msg = self.parser.predicted_trajectory_to_prompTraj_message(trajectory_wrt_object, self.parser.point_to_list(self.context))

                # resp = add_demonstration(trajectory_wrt_object_msg)
            
                # raw_path = self._rospack.get_path('teach_pendant') + "/data/"
                # file_name = self._get_trajectory_file_name(raw_path)
                # self._save_data(raw_path, file_name)

                # self.EEtrajectory = []

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