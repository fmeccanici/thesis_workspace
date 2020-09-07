#!/usr/bin/env python3.5

from subprocess import call
import rospy, rospkg, roslaunch, time, random, copy, os
from learning_from_demonstration_python.trajectory_parser import trajectoryParser
from learning_from_demonstration_python.trajectory_resampler import trajectoryResampler
from data_logger_python.text_updater import TextUpdater
from pyquaternion import Quaternion
import numpy as np
from experiment_variables.experiment_variables import ExperimentVariables
from traffic_light.traffic_light_updater import TrafficLightUpdater
from collections import deque

# import ros messages
from sensor_msgs.msg import JointState
from data_logger.msg import TrajectoryData, OperatorGUIinteraction
from std_msgs.msg import Byte, Bool, Float32, String, Float64
from geometry_msgs.msg import Point, Pose, PoseStamped

from gazebo_msgs.msg import ModelState 
from trajectory_visualizer.msg import TrajectoryVisualization
from execution_failure_detection.msg import ExecutionFailure
from teleop_control.msg import Keyboard
from teleop_control.srv import SetPartToPublish, SetPartToPublishResponse
from slave_control.msg import ControlState


# import ros services
from data_logger.srv import (CreateParticipant, AddRefinement,
                                SetPrediction, ToCsv, SetObstaclesHit,
                                SetObjectMissed, IncrementNumberOfRefinements,
                                SetParameters, SetTime, SetNumberOfRefinements)

from learning_from_demonstration.srv import (GoToPose, MakePrediction, 
                                                GetContext, GetObjectPosition,
                                                WelfordUpdate, ExecuteTrajectory, 
                                                GetEEPose, AddDemonstration, SetTeachStateOmni,
                                                ClearTrajectory, GetTrajectory, BuildInitialModel)

from gazebo_msgs.srv import SetModelState, SetModelConfiguration
from trajectory_visualizer.srv import VisualizeTrajectory, ClearTrajectories
from trajectory_refinement.srv import RefineTrajectory, CalibrateMasterPose
from execution_failure_detection.srv import GetExecutionFailure, SetExpectedObjectPosition
from experiment.srv import SetText
from teach_pendant.srv import GetTeachState, GetDemonstrationPendant, SetTeachState, AddWaypoint, ClearWaypoints

class TrainingNode(object):
    def __init__(self):
        rospy.init_node('experiment')
        self._getRosParameters()

        self.training_scores = deque([0,0,0,0])
        self.stop_updating_flag = 0
        self.collision_updating_flag = 0
        self.pressed_key = ""

        self.rospack = rospkg.RosPack()
        self.parser = trajectoryParser()
        self.resampler = trajectoryResampler()
        self.text_updater = TextUpdater()
        self.object_missed_updater = TextUpdater(text_file='object_missed.txt')
        self.object_kicked_over_updater = TextUpdater(text_file='object_kicked_over.txt')
        self.obstacle_hit_updater = TextUpdater(text_file='obstacle_hit.txt')
        self.number_of_refinements_updater = TextUpdater(text_file='number_of_refinements.txt')
        self.number_of_refinements_updater.update(str(0))
        self.traffic_light_updater = TrafficLightUpdater()
        self.traffic_light_updater.update('red')

        self.experiment_variables = ExperimentVariables()
        self.T_desired = self.experiment_variables.T_desired
        self.start_time = 0
        self.elapsed_time = 0
        self.elapsed_time_prev = 0

        self.execution_failure_sub = rospy.Subscriber('execution_failure', ExecutionFailure, self._executionFailureCallback)
        self.keyboard_sub = rospy.Subscriber('keyboard_control', Keyboard, self._keyboardCallback)
        self.operator_gui_interaction_sub = rospy.Subscriber('/operator_gui_interaction', OperatorGUIinteraction, self._operatorGuiInteraction)
        self.slave_control_state_sub = rospy.Subscriber('slave_control_state', ControlState, self._slaveControlStateCallback)

        self.lift_goal_pub = rospy.Publisher('/lift_controller_ref', JointState, queue_size=10)
        self.head_goal_pub = rospy.Publisher('/head_controller_ref', JointState, queue_size=10)


    def _getRosParameters(self):
        self.method = rospy.get_param('~method')            

    def isStringEmpty(self, string):
        return string == "" or string == ''
    
    def _slaveControlStateCallback(self, data):
        self.en_arm = data.en_arm.data
    
    def _keyboardCallback(self, data):
        if not self.isStringEmpty(data.key.data):
            self.pressed_key = data.key.data
    
    def _operatorGuiInteraction(self, data):    
        self.participant_number = data.number.data
        print("Participant number is " + str(self.participant_number))

    def startTimer(self):
        self.start_time = time.time()
    
    def stopTimer(self):
        rospy.loginfo("Stopped timer")
        
        if self.start_time == 0:
            self.elapsed_time = 0
        else:
            self.elapsed_time = self.elapsed_time_prev + (time.time() - self.start_time)
        rospy.loginfo("time = " + str(self.elapsed_time))
        self.elapsed_time_prev = self.elapsed_time
    
    def zeroTimer(self):
        self.elapsed_time = 0
        self.elapsed_time_prev = 0
        self.start_time = 0

    # failure detection callback
    def _executionFailureCallback(self, data):
        if self.stop_updating_flag == 0:
            self.object_missed_updater.update(str(not data.object_reached.data))
            
            if self.collision_updating_flag == 1:
                self.obstacle_hit_updater.update( str(data.obstacle_hit.data ))

            self.object_kicked_over_updater.update( str(data.object_kicked_over.data ))

    def openGripper(self):
        rc = call("/home/fmeccanici/Documents/thesis/thesis_workspace/src/teleop_control/scripts/gripper_opener.sh")

    def getContext(self):
        try:
            rospy.wait_for_service('get_context', timeout=2.0)

            get_context = rospy.ServiceProxy('get_context', GetContext)
            resp = get_context()
            self.context = resp.context

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)  

    def executeTrajectory(self, traj):
        rospy.wait_for_service('execute_trajectory', timeout=2.0)
        execute_trajectory = rospy.ServiceProxy('execute_trajectory', ExecuteTrajectory)
        resp = execute_trajectory(traj, self.T_desired)

        return resp.obstacle_hit.data, resp.object_reached.data, resp.object_kicked_over.data
    
    def setObjectPosition(self):
        try:
            object_position = ModelState()
            object_position.model_name = 'aruco_cube'

            object_position.pose.position.x = 0.8

            # get random y value
            y = self.y_position

            object_position.pose.position.y = y

            object_position.pose.position.z = 0.7

            object_position.pose.orientation.x = 0
            object_position.pose.orientation.y = 0
            object_position.pose.orientation.z = 0
            object_position.pose.orientation.w = 1
            
            rospy.wait_for_service('/gazebo/set_model_state')

            set_object = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

            resp = set_object(object_position)

            rospy.wait_for_service('/set_expected_object_position')

            set_expected_object_pose = rospy.ServiceProxy('/set_expected_object_position', SetExpectedObjectPosition)
            set_expected_object_pose(object_position.pose)

            return resp.success

        except ValueError:
            rospy.loginfo("Invalid value for position!")

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s"%e)

    def setDishwasherPosition(self):
        try:
            dishwasher = ModelState()
            dishwasher.model_name = 'dishwasher'

            dishwasher.pose.position.x = 1.75
            dishwasher.pose.position.y = 0.336

            dishwasher.pose.position.z = 0.098

            dishwasher.pose.orientation.x = 0
            dishwasher.pose.orientation.y = 0
            dishwasher.pose.orientation.z = -0.6995
            dishwasher.pose.orientation.w = 0.7146
            
            rospy.wait_for_service('/gazebo/set_model_state')

            set_object = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

            resp = set_object(dishwasher)

            rospy.loginfo("Reset dishwasher pose")
            return resp.success

        except ValueError:
            rospy.loginfo("Invalid value for position!")

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s"%e)

    def refinePrediction(self):
        return self.pressed_key == 'left' or self.pressed_key == 'right'

    def isKeyPressedLeftOrRight(self, key):
        return key == 'left' or key == 'right'

    def waitForKeyPress(self):
        self.resetKeyPressed()
        while True:
            if self.isKeyPressedLeftOrRight(self.pressed_key):
                print(str(self.pressed_key) + " key presssed")
                return
    
    def resetKeyPressed(self):
        self.pressed_key = ""

    def goToInitialPose(self):
        pose = Pose()
        try:

            # first waypoint for dishwasher
            pose.position.x = 0.445
            pose.position.y = 0.013
            pose.position.z = 0.877

            pose.orientation.x = 0.982
            pose.orientation.y = 0.009
            pose.orientation.z = -0.189
            pose.orientation.w = 0.009

            rospy.wait_for_service('go_to_pose', timeout=2.0)
            go_to_pose = rospy.ServiceProxy('go_to_pose', GoToPose)
            resp = go_to_pose(pose)

            # second waypoint for dishwasher
            pose.position.x = 0.482
            pose.position.y = -0.373
            pose.position.z = 0.866

            pose.orientation.x = 0.989
            pose.orientation.y = -0.021
            pose.orientation.z = -0.096
            pose.orientation.w = 0.109

            rospy.wait_for_service('go_to_pose', timeout=2.0)
            go_to_pose = rospy.ServiceProxy('go_to_pose', GoToPose)
            resp = go_to_pose(pose)

            # third waypoint for dishwasher
            pose.position.x = 0.394
            pose.position.y = -0.305
            pose.position.z = 0.46

            pose.orientation.x = 0.886
            pose.orientation.y = -0.219
            pose.orientation.z = -0.407
            pose.orientation.w = -0.037

            rospy.wait_for_service('go_to_pose', timeout=2.0)
            go_to_pose = rospy.ServiceProxy('go_to_pose', GoToPose)
            resp = go_to_pose(pose)
            
            # initial pose dishwasher moved backwards2
            pose.position.x = 0.537
            pose.position.y = 0.083
            pose.position.z = 0.444

            pose.orientation.x = 0.944
            pose.orientation.y = -0.007
            pose.orientation.z = -0.329
            pose.orientation.w = 0.02

            rospy.wait_for_service('go_to_pose', timeout=2.0)
            go_to_pose = rospy.ServiceProxy('go_to_pose', GoToPose)
            resp = go_to_pose(pose)
            resp = go_to_pose(pose)
            resp = go_to_pose(pose)
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def predict(self):
        try:
            rospy.wait_for_service('make_prediction', timeout=2.0)

            make_prediction = rospy.ServiceProxy('make_prediction', MakePrediction)
            resp = make_prediction(self.context)
            self.prediction = resp.prediction
        
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
        
        except AttributeError:
            rospy.loginfo("Context not yet extracted!")

    def visualize(self, which):
        try:
            rospy.wait_for_service('visualize_trajectory', timeout=2.0)

            visualize_trajectory = rospy.ServiceProxy('visualize_trajectory', VisualizeTrajectory)
            visualization_msg = TrajectoryVisualization()

            rospy.wait_for_service('clear_trajectories', timeout=2.0)
            clear_trajectories = rospy.ServiceProxy('clear_trajectories', ClearTrajectories)

            resp = clear_trajectories()

            if which == 'both':
                visualization_msg.pose_array = self.refined_trajectory.poses
                visualization_msg.r = 0.0
                visualization_msg.g = 1.0
                visualization_msg.b = 0.0
                resp = visualize_trajectory(visualization_msg)

                # visualize keypoints used for the promp model
                n_keypoints = 10
                refined_traj_keypoints = self.parser.promptraj_msg_to_execution_format(self.refined_trajectory)
                refined_traj_keypoints = self.resampler.interpolate_learned_keypoints(refined_traj_keypoints[0], n_keypoints)
                refined_traj_keypoints = self.parser.predicted_trajectory_to_prompTraj_message(refined_traj_keypoints, self.parser.point_to_list(self.context))
                
                visualization_msg.pose_array = refined_traj_keypoints.poses
                visualization_msg.r = 1.0
                visualization_msg.g = 1.0
                visualization_msg.b = 0.0

                resp = visualize_trajectory(visualization_msg)
                
                visualization_msg.pose_array = self.prediction.poses
                visualization_msg.r = 1.0
                visualization_msg.g = 0.0
                visualization_msg.b = 0.0

                resp = visualize_trajectory(visualization_msg)
                
                # visualize keypoints used for the promp model
                n_keypoints = 10
                prediction_traj_keypoints = self.parser.promptraj_msg_to_execution_format(self.prediction)
                prediction_traj_keypoints = self.resampler.interpolate_learned_keypoints(prediction_traj_keypoints[0], n_keypoints)
                prediction_traj_keypoints = self.parser.predicted_trajectory_to_prompTraj_message(prediction_traj_keypoints, self.parser.point_to_list(self.context))
                
                visualization_msg.pose_array = prediction_traj_keypoints.poses
                visualization_msg.r = 0.0
                visualization_msg.g = 0.0
                visualization_msg.b = 1.0
                resp = visualize_trajectory(visualization_msg)
                
            elif which == 'refinement':
                visualization_msg.pose_array = self.refined_trajectory.poses
                visualization_msg.r = 0.0
                visualization_msg.g = 1.0
                visualization_msg.b = 0.0
                resp = visualize_trajectory(visualization_msg)
                
                # visualize keypoints used for the promp model
                n_keypoints = 10
                refined_traj_keypoints = self.parser.promptraj_msg_to_execution_format(self.refined_trajectory)
                refined_traj_keypoints = self.resampler.interpolate_learned_keypoints(refined_traj_keypoints[0], n_keypoints)
                refined_traj_keypoints = self.parser.predicted_trajectory_to_prompTraj_message(refined_traj_keypoints, self.parser.point_to_list(self.context))
                
                visualization_msg.pose_array = refined_traj_keypoints.poses
                visualization_msg.r = 1.0
                visualization_msg.g = 1.0
                visualization_msg.b = 0.0
                resp = visualize_trajectory(visualization_msg)

            elif which == 'prediction':

                visualization_msg.pose_array = self.prediction.poses
                visualization_msg.r = 1.0
                visualization_msg.g = 0.0
                visualization_msg.b = 0.0                
                resp = visualize_trajectory(visualization_msg)

                # visualize keypoints used for the promp model
                n_keypoints = 10
                prediction_traj_keypoints = self.parser.promptraj_msg_to_execution_format(self.prediction)
                prediction_traj_keypoints = self.resampler.interpolate_learned_keypoints(prediction_traj_keypoints[0], n_keypoints)
                prediction_traj_keypoints = self.parser.predicted_trajectory_to_prompTraj_message(prediction_traj_keypoints, self.parser.point_to_list(self.context))
                
                visualization_msg.pose_array = prediction_traj_keypoints.poses
                visualization_msg.r = 0.0
                visualization_msg.g = 0.0
                visualization_msg.b = 1.0
                resp = visualize_trajectory(visualization_msg)

            elif 'clear':
                pass

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

        except AttributeError as e:
            rospy.loginfo("No prediction made yet!:")
            rospy.loginfo(str(e))

    def saveData(self):
        path = self.experiment_variables.data_path + 'participant_' + str(self.participant_number) + '/training/' + str(self.method) + '/'

        if not os.path.isdir(path):
            os.makedirs(path)

        data = {'moving_average': list(self.training_scores), 'time': self.elapsed_time}
        
        with open(path + 'data.txt', 'w+') as f:
            f.write(str(data))

    def initializeHeadLiftJoint(self):
        lift_goal = JointState()
        head_goal = JointState()

        lift_goal.name = ["torso_lift_joint"]
        head_goal.name = ["head_2_joint", "head_1_joint"]

        lift_goal.position = [0.3]

        head_goal.position = [-0.9, 0.0]

        head_goal.effort = [0.0, 0.0]
        lift_goal.effort = [0.0]

        head_goal.velocity = [0.0, 0.0]
        lift_goal.velocity = [0.0]

        self.lift_goal_pub.publish(lift_goal)
        self.head_goal_pub.publish(head_goal)

    def isArmEnabled(self):
        return self.en_arm == 1

    def storeData(self, success):
        self.training_scores.rotate()
        self.training_scores[0] = success
        print(self.training_scores)
    
    def startTraining(self):
        
        text = "FILL IN PARTICIPANT NUMBER"
        self.text_updater.update(text)
            
        rospy.wait_for_message('operator_gui_interaction', OperatorGUIinteraction)
        
        self.y_position = 0.05

        self.initializeHeadLiftJoint()

        self.openGripper()
        self.goToInitialPose()
        self.setDishwasherPosition()
        self.setObjectPosition()
        time.sleep(2)
        self.getContext()
        self.predict()
        self.visualize('prediction')
        self.traffic_light_updater.update('red')
        self.text_updater.update("AUTONOMOUS EXECUTION")
        obstacle_hit, object_reached, object_kicked_over = self.executeTrajectory(self.prediction)
        
        # store prediction along with failure
        
        # update text in operator gui
        if obstacle_hit or not object_reached or object_kicked_over:
            self.text_updater.update("FAILURE:")
        else:
            self.text_updater.update("SUCCESS!")
        
        if obstacle_hit:
            self.text_updater.append("OBSTACLE HIT")
        if not object_reached:
            self.text_updater.append("OBJECT MISSED")
        if object_kicked_over:
            self.text_updater.append("OBJECT KICKED OVER")


        number_of_refinements = 0

        if self.method == 'offline+pendant':
            rospy.wait_for_service('/offline_pendant/set_teach_state', timeout=2.0)
            set_teach_state = rospy.ServiceProxy('/offline_pendant/set_teach_state', SetTeachState)
            set_teach_state(Bool(False))
        elif self.method == 'offline+omni':
            rospy.wait_for_service('/trajectory_teaching/set_teach_state', timeout=2.0)
            set_teach_state = rospy.ServiceProxy('/trajectory_teaching/set_teach_state', SetTeachStateOmni)
            set_teach_state(Bool(False))

        if self.method == 'online+pendant':
            while np.mean(list(self.training_scores)) < 1/2: 
                print(np.mean(list(self.training_scores)))

                self.goToInitialPose()
                self.setDishwasherPosition()
                time.sleep(3)
                self.setObjectPosition()

                self.traffic_light_updater.update('green')
                # wait until the operator clicked the red or green button
                self.text_updater.update("REFINE RED OR GREEN?")
                # rospy.wait_for_message('operator_gui_interaction', OperatorGUIinteraction)
                # rospy.wait_for_message('keyboard_control', Keyboard)
                self.waitForKeyPress()

                self.stop_updating_flag = 0

                refine_trajectory = rospy.ServiceProxy('refine_trajectory', RefineTrajectory)
                
                if self.refinePrediction():

                    # we only need to start the timer if it is equal to zero, else just keep the timer running
                    if self.start_time == 0:
                        # start timer
                        self.startTimer()
                    else: pass

                    resp = refine_trajectory(self.prediction, self.T_desired)
                
                self.traffic_light_updater.update('red')

                self.refined_trajectory = resp.refined_trajectory
                with open('/home/fmeccanici/Documents/thesis/thesis_workspace/src/experiment/debug/refined_trajectory.txt', 'w+') as f:
                    f.write(str(self.refined_trajectory))
                    
                time.sleep(5)

                obstacle_hit = resp.obstacle_hit.data
                execution_failure = rospy.ServiceProxy('get_execution_failure', GetExecutionFailure)
                resp = execution_failure()
                object_reached = resp.object_reached.data
                object_kicked_over = resp.object_kicked_over.data

                print("\n")

                rospy.loginfo("object missed: " + str(not object_reached))
                rospy.loginfo("obstacle hit: " + str(obstacle_hit))
                rospy.loginfo("object kicked over: " + str(object_kicked_over))

                print("\n")

                self.stop_updating_flag = 1

                # update text in operator gui
                if obstacle_hit or not object_reached or object_kicked_over:
                    success = 0
                    self.storeData(success)
                    self.text_updater.update("FAILURE: Score = " + str(int(np.mean(list(self.training_scores)) * 100)) + '%')

                else:
                    success = 1
                    self.storeData(success)
                    self.text_updater.update("SUCCESS!: Score = " + str(int(np.mean(list(self.training_scores)) * 100)) + '%')

                if obstacle_hit:
                    self.text_updater.append("OBSTACLE HIT")
                if not object_reached:
                    self.text_updater.append("OBJECT MISSED")
                if object_kicked_over:
                    self.text_updater.append("OBJECT KICKED OVER")

                time.sleep(2)
                # store refinement along with if it failed or not
               
                number_of_refinements += 1
                rospy.loginfo("Got a refined trajectory")

                self.visualize('both')
                print("number of refinement = " + str(number_of_refinements))
                self.number_of_refinements_updater.update(str(number_of_refinements))

        elif self.method == 'offline+pendant':

            while np.mean(list(self.training_scores)) < 1/2: 
                self.goToInitialPose()
                self.setDishwasherPosition()
                time.sleep(3)
                self.setObjectPosition()

                rospy.wait_for_service('/offline_pendant/add_waypoint', timeout=2.0)
                add_waypoint = rospy.ServiceProxy('/offline_pendant/add_waypoint', AddWaypoint)
                add_waypoint()

                set_teach_state(Bool(True))

                self.traffic_light_updater.update('green')

                self.text_updater.update("START TEACHING")
                self.collision_updating_flag = 1

                # we only need to start the timer if it is equal to zero, else just keep the timer running
                if self.start_time == 0:
                    # start timer
                    self.startTimer()
                else: pass

                rospy.wait_for_service('offline_pendant/get_teach_state', timeout=2.0)
                get_teach_state = rospy.ServiceProxy('offline_pendant/get_teach_state', GetTeachState)
                resp = get_teach_state()
                isTeachingOffline = resp.teach_state.data      
                
                # use teach_pendant node to teach offline
                while isTeachingOffline:
                    resp = get_teach_state()
                    isTeachingOffline = resp.teach_state.data        

                self.traffic_light_updater.update('red')

                
                rospy.wait_for_service('get_demonstration_pendant', timeout=2.0)
                get_demo_pendant = rospy.ServiceProxy('get_demonstration_pendant', GetDemonstrationPendant)
                resp = get_demo_pendant()

                set_teach_state(Bool(False))

                # self.stopNode('teach_pendant.launch')
                
                self.goToInitialPose()
                self.setDishwasherPosition()
                time.sleep(3)
                self.setObjectPosition()

                self.refined_trajectory = resp.demo
                self.visualize('both')
                time.sleep(3)

                self.collision_updating_flag = 0

                self.text_updater.update("AUTONOMOUS EXECUTION")

                self.stopTimer()
                obstacle_hit, object_reached, object_kicked_over = self.executeTrajectory(self.refined_trajectory)
                self.startTimer()

                print("\n")

                rospy.loginfo("object missed: " + str(not object_reached))
                rospy.loginfo("obstacle hit: " + str(obstacle_hit))
                rospy.loginfo("object kicked over: " + str(object_kicked_over))

                print("\n")

                # update text in operator gui
                if obstacle_hit or not object_reached or object_kicked_over:
                    success = 0
                    self.storeData(success)
                    self.text_updater.update("FAILURE: Score = " + str(int(np.mean(list(self.training_scores)) * 100)) + '%')
                else:
                    success = 1
                    self.storeData(success)
                    self.text_updater.update("SUCCESS!: Score = " + str(int(np.mean(list(self.training_scores)) * 100)) + '%')

                if obstacle_hit:
                    self.text_updater.append("OBSTACLE HIT")
                if not object_reached:
                    self.text_updater.append("OBJECT MISSED")
                if object_kicked_over:
                    self.text_updater.append("OBJECT KICKED OVER")
               
                number_of_refinements += 1

                rospy.loginfo("Got a refined trajectory")

                print("number of refinement = " + str(number_of_refinements))
                self.number_of_refinements_updater.update(str(number_of_refinements))

            rospy.wait_for_service('offline_pendant/clear_waypoints', timeout=2.0)
            clear_waypoints = rospy.ServiceProxy('offline_pendant/clear_waypoints', ClearWaypoints)
            clear_waypoints()
        
        elif self.method == 'online+omni':
            while np.mean(list(self.training_scores)) < 1/2: 
                print("Trajectory failure!")

                self.goToInitialPose()
                self.setDishwasherPosition()
                time.sleep(3)
                self.setObjectPosition()

                self.traffic_light_updater.update('green')
                # wait until the operator clicked the red or green button
                self.text_updater.update("REFINE RED OR GREEN?")
                self.waitForKeyPress()
                self.text_updater.update("PRESS WHITE BUTTON TO STOP REFINING")

                self.stop_updating_flag = 0

                refine_trajectory = rospy.ServiceProxy('refine_trajectory', RefineTrajectory)
                
                if self.refinePrediction():

                    # we only need to start the timer if it is equal to zero, else just keep the timer running
                    if self.start_time == 0:
                        # start timer
                        self.startTimer()
                    else: pass

                    resp = refine_trajectory(self.prediction, self.T_desired)
                
                elif self.refineRefinement():

                    # we only need to start the timer if it is equal to zero, else just keep the timer running
                    if self.start_time == 0:
                        # start timer
                        self.startTimer()
                    else: pass

                    resp = refine_trajectory(self.refined_trajectory, self.T_desired)
                
                self.traffic_light_updater.update('red')


                self.refined_trajectory = resp.refined_trajectory

                time.sleep(5)

                obstacle_hit = resp.obstacle_hit.data
                execution_failure = rospy.ServiceProxy('get_execution_failure', GetExecutionFailure)
                resp = execution_failure()
                object_reached = resp.object_reached.data
                object_kicked_over = resp.object_kicked_over.data

                print("\n")

                rospy.loginfo("object missed: " + str(not object_reached))
                rospy.loginfo("obstacle hit: " + str(obstacle_hit))
                rospy.loginfo("object kicked over: " + str(object_kicked_over))

                print("\n")

                self.stop_updating_flag = 1

                # update text in operator gui
                if obstacle_hit or not object_reached or object_kicked_over:
                    success = 0
                    self.storeData(success)
                    self.text_updater.update("FAILURE: Score = " + str(int(np.mean(list(self.training_scores)) * 100)) + '%')
                else:
                    success = 1
                    self.storeData(success)
                    self.text_updater.update("SUCCESS!: Score = " + str(int(np.mean(list(self.training_scores)) * 100)) + '%')

                if obstacle_hit:
                    self.text_updater.append("OBSTACLE HIT")
                if not object_reached:
                    self.text_updater.append("OBJECT MISSED")
                if object_kicked_over:
                    self.text_updater.append("OBJECT KICKED OVER")
               
                number_of_refinements += 1

                rospy.loginfo("Got a refined trajectory")

                self.visualize('both')
                print("number of refinement = " + str(number_of_refinements))
                self.number_of_refinements_updater.update(str(number_of_refinements))

        elif self.method == 'offline+omni':

            while np.mean(list(self.training_scores)) < 1/2: 
                self.goToInitialPose()
                self.setDishwasherPosition()
                time.sleep(3)
                self.setObjectPosition()

                rospy.wait_for_service('/set_part_to_publish', timeout=2.0)
                set_part_to_publish = rospy.ServiceProxy('/set_part_to_publish', SetPartToPublish)
                set_part_to_publish(String('both'))

                self.collision_updating_flag = 1

                # we only need to start the timer if it is equal to zero, else just keep the timer running
                if self.start_time == 0:
                    # start timer
                    self.startTimer()
                else: pass

                rospy.wait_for_service('trajectory_teaching/get_teach_state', timeout=2.0)
                get_teach_state = rospy.ServiceProxy('trajectory_teaching/get_teach_state', GetTeachState)
                resp = get_teach_state()
                isTeachingOffline = resp.teach_state.data      
                
                self.traffic_light_updater.update('green')

                while not isTeachingOffline:
                    self.text_updater.update("PRESS WHITE BUTTON TO START TEACHING")
                    resp = get_teach_state()
                    isTeachingOffline = resp.teach_state.data 

                # use teach_pendant node to teach offline
                while isTeachingOffline:
                    self.text_updater.update("PRESS WHITE BUTTON TO STOP TEACHING")

                    resp = get_teach_state()                
                    isTeachingOffline = resp.teach_state.data 

                self.traffic_light_updater.update('red')

                self.text_updater.update("STOPPED TEACHING")

                rospy.wait_for_service('trajectory_teaching/get_trajectory', timeout=2.0)
                get_demo = rospy.ServiceProxy('trajectory_teaching/get_trajectory', GetTrajectory)
                resp = get_demo()

                set_teach_state(Bool(False))

                while self.isArmEnabled():
                    self.text_updater.update("PRESS GREY BUTTON")
                
                self.text_updater.update("GREY BUTTON PRESSED")

                self.goToInitialPose()
                self.setDishwasherPosition()
                time.sleep(3)
                self.setObjectPosition()

                self.refined_trajectory = resp.demo
                self.visualize('both')
                time.sleep(3)

                self.collision_updating_flag = 0
                
                self.text_updater.update("AUTONOMOUS EXECUTION")
                obstacle_hit, object_reached, object_kicked_over = self.executeTrajectory(self.refined_trajectory)
                
                print("\n")

                rospy.loginfo("object missed: " + str(not object_reached))
                rospy.loginfo("obstacle hit: " + str(obstacle_hit))
                rospy.loginfo("object kicked over: " + str(object_kicked_over))

                print("\n")

                # update text in operator gui
                if obstacle_hit or not object_reached or object_kicked_over:
                    success = 0
                    self.storeData(success)
                    self.text_updater.update("FAILURE: Score = " + str(int(np.mean(list(self.training_scores)) * 100)) + '%')
                else:
                    success = 1
                    self.storeData(success)
                    self.text_updater.update("SUCCESS!: Score = " + str(int(np.mean(list(self.training_scores)) * 100)) + '%')

                if obstacle_hit:
                    self.text_updater.append("OBSTACLE HIT")
                if not object_reached:
                    self.text_updater.append("OBJECT MISSED")
                if object_kicked_over:
                    self.text_updater.append("OBJECT KICKED OVER")
               
                # increment number of refinements                
                number_of_refinements += 1

                rospy.loginfo("Got a refined trajectory")

                print("number of refinement = " + str(number_of_refinements))
                self.number_of_refinements_updater.update(str(number_of_refinements))
                
                rospy.wait_for_service('trajectory_teaching/clear_trajectory', timeout=2.0)
                clear_trajectory = rospy.ServiceProxy('trajectory_teaching/clear_trajectory', ClearTrajectory)
                resp = clear_trajectory()

            rospy.wait_for_service('trajectory_teaching/clear_trajectory', timeout=2.0)
            clear_trajectory = rospy.ServiceProxy('trajectory_teaching/clear_trajectory', ClearTrajectory)
            clear_trajectory()
        
        self.number_of_refinements_updater.update(str(0))
        
        self.stopTimer()
        
        ###### save data ######
        self.saveData()
        self.zeroTimer()

        self.text_updater.update("END TRAINING: Final score = " + str(int(np.mean(list(self.training_scores)) * 100)) + '%')
    
    def run(self):
        self.startTraining()

if __name__ == "__main__":
    training_node = TrainingNode()
    training_node.startTraining()
