#!/usr/bin/env python3.5

from subprocess import call
import rospy, rospkg, roslaunch, time, random, copy
from learning_from_demonstration_python.trajectory_parser import trajectoryParser
from learning_from_demonstration_python.trajectory_resampler import trajectoryResampler
from data_logger_python.text_updater import TextUpdater
from pyquaternion import Quaternion
import numpy as np
from experiment_variables.experiment_variables import ExperimentVariables
from traffic_light.traffic_light_updater import TrafficLightUpdater

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

class ExperimentNode(object):
    def __init__(self):
        rospy.init_node('experiment')
        self._getParameters()
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
        self.nodes = {}
        self.method_mapping = self.experiment_variables.method_mapping_str_to_number

        self.num_models = self.experiment_variables.num_models
        self.num_trials = self.experiment_variables.num_trials
        self.num_object_positions = self.experiment_variables.num_object_positions
        self.trials = range(1,self.num_trials+1)

        self.object_positions = range(1,self.num_object_positions+1)
        self.models = range(1,self.num_models+1)

        self.max_refinements = self.experiment_variables.max_refinements
        self.elapsed_time = 0
        self.elapsed_time_prev = 0

        self.T_desired = self.experiment_variables.T_desired
        self.start_time = 0
        self.set_data = False

        self.current_trial = 1
        self.current_object_position = 1
        self.current_model = 1

        # self.y_position_step_dict = {1: 0.0, 2: 0.066, 3: 2*0.066, 4: 3*0.066, 5: 4*0.066, 6: 5*0.066}
        self.y_position_step_dict = copy.deepcopy(self.experiment_variables.y_position_step_dict)

        # ros stuff
        self.lift_goal_pub = rospy.Publisher('/lift_controller_ref', JointState, queue_size=10)
        self.head_goal_pub = rospy.Publisher('/head_controller_ref', JointState, queue_size=10)
        self.operator_gui_interaction_sub = rospy.Subscriber('/operator_gui_interaction', OperatorGUIinteraction, self._operatorGuiInteraction)
        # self.operator_gui_text_pub = rospy.Publisher('/operator_gui/text', String, queue_size=10)

        self.slave_control_state_sub = rospy.Subscriber('slave_control_state', ControlState, self._slaveControlStateCallback)

        self.execution_failure_sub = rospy.Subscriber('execution_failure', ExecutionFailure, self._executionFailureCallback)
        self.keyboard_sub = rospy.Subscriber('keyboard_control', Keyboard, self._keyboardCallback)

        self.stop_updating_flag = 0
        self.collision_updating_flag = 0

        self.pressed_key = ""

    def _slaveControlStateCallback(self, data):
        self.en_arm = data.en_arm.data

    def isStringEmpty(self, string):
        return string == "" or string == ''

    def _keyboardCallback(self, data):
        if not self.isStringEmpty(data.key.data):
            self.pressed_key = data.key.data
            
    # failure detection callback
    def _executionFailureCallback(self, data):
        if self.stop_updating_flag == 0:
            self.object_missed_updater.update(str(not data.object_reached.data))
            
            if self.collision_updating_flag == 1:
                self.obstacle_hit_updater.update( str(data.obstacle_hit.data ))

            self.object_kicked_over_updater.update( str(data.object_kicked_over.data ))

    def _operatorGuiInteraction(self, data):    
        self.participant_number = data.number.data
        print("Participant number is " + str(self.participant_number))

    def _getParameters(self):
        self.method = rospy.get_param('~method')            

    def setOperatorGuiText(self, text):
        set_text = rospy.ServiceProxy('operator_gui/set_text', SetText)
        set_text(String(text))

    def startNode(self, package, launch_file):
        if launch_file not in self.nodes: 
            abs_path = self.rospack.get_path(package) + "/launch/" + launch_file
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [abs_path])

            # needed to be able to stop the node
            self.nodes[launch_file] = launch

            # start node
            self.nodes[launch_file].start()
            
        else:
            # we need to delete the previous node from dict
            # else it gives an error
            del self.nodes[launch_file]

            # and append a new one again
            abs_path = self.rospack.get_path(package) + "/launch/" + launch_file
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [abs_path])

            # needed to be able to stop the node
            self.nodes[launch_file] = launch

            self.nodes[launch_file].start()
            rospy.loginfo( ("Started {} ").format(launch_file) )
    
    def stopNode(self, launch_file):
        # look through dictionary to find corresponding launch object
        try:
            self.nodes[launch_file].shutdown()
        except (AttributeError, KeyError):
            rospy.loginfo( ("Node not launched yet") )
    
    def openGripper(self):
        rc = call("/home/fmeccanici/Documents/thesis/thesis_workspace/src/teleop_control/scripts/gripper_opener.sh")

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
    
    def determineYPosition(self):
        random_object_position = random.choice(list(self.y_position_step_dict.keys()))
        
        step = self.y_position_step_dict[random_object_position]

        del self.y_position_step_dict[random_object_position]

        y0 = self.experiment_variables.y0

        return y0 - step

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

            # table
            """
            pose.position.x = 0.609
            pose.position.y = -0.290
            pose.position.z = 0.816

            pose.orientation.x = 0.985
            pose.orientation.y = -0.103
            pose.orientation.z = -0.124
            pose.orientation.w = 0.064
            
            rospy.wait_for_service('go_to_pose', timeout=2.0)
            go_to_pose = rospy.ServiceProxy('go_to_pose', GoToPose)
            resp = go_to_pose(pose)
            """
            
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def getContext(self):
        try:
            rospy.wait_for_service('get_context', timeout=2.0)

            get_context = rospy.ServiceProxy('get_context', GetContext)
            resp = get_context()
            self.context = resp.context

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
    
    def setDataLoggerParameters(self):
        number_msg = Byte(self.participant_number)

        model_msg = Byte(self.current_model)
        method_msg = Byte(self.method_mapping[self.method])

        object_position_msg = Byte(self.current_object_position)

        context_msg = Point()
        try:
            context_msg = self.context
        except AttributeError as e:
            rospy.loginfo("Context not set!: " + str(e))
            return -1
        trial_msg = Byte(self.current_trial)


        rospy.wait_for_service('data_logger/set_parameters', timeout=2.0)
        
        set_parameters = rospy.ServiceProxy('data_logger/set_parameters', SetParameters)
        resp = set_parameters(number_msg, object_position_msg, trial_msg, method_msg, model_msg)

    def loadParticipant(self):
        try: 
            rospy.wait_for_service('create_participant', timeout=2.0)
            create_participant = rospy.ServiceProxy('create_participant', CreateParticipant)
            number_msg = Byte()
            
            # wait until we have operator interaction
            text = "FILL IN PARTICIPANT NUMBER"
            self.text_updater.update(text)
            
            rospy.wait_for_message('operator_gui_interaction', OperatorGUIinteraction)
            number_msg.data = self.participant_number
            self.text_updater.empty()

            # dummy variable names --> not necessary when data exists --> data logger loads this automatically instead of writing it
            gender_msg = Bool(0)
            age_msg = Byte(1)
            teleop_experience_msg = Byte(1)
            keyboard_experience_msg = Byte(1)
            left_right_handed_msg = Bool(1)

            resp = create_participant(number_msg, gender_msg, age_msg, teleop_experience_msg, keyboard_experience_msg, left_right_handed_msg)     

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
    
    def executeTrajectory(self, traj):
        rospy.wait_for_service('execute_trajectory', timeout=2.0)
        execute_trajectory = rospy.ServiceProxy('execute_trajectory', ExecuteTrajectory)
        resp = execute_trajectory(traj, self.T_desired)

        return resp.obstacle_hit.data, resp.object_reached.data, resp.object_kicked_over.data
    
    def storePrediction(self, prediction, number_msg):
        try:
            rospy.wait_for_service('set_prediction', timeout=2.0)
            set_prediction = rospy.ServiceProxy('set_prediction', SetPrediction)
            resp = set_prediction(number_msg, prediction)
        
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def storeRefinement(self, refinement, number_msg):
        try:
            rospy.wait_for_service('add_refinement', timeout=2.0)
            add_refinement = rospy.ServiceProxy('add_refinement', AddRefinement)
            resp = add_refinement(number_msg, refinement)
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def storeTime(self, number_msg, time_msg):
        try:
            rospy.wait_for_service('data_logger/set_time', timeout=2.0)
            set_time = rospy.ServiceProxy('data_logger/set_time', SetTime)

            resp = set_time(number_msg, time_msg)
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def storeData(self, *args, **kwargs):
        number_msg = Byte(self.participant_number)

        if "time" in kwargs:
            time_msg = Float32(self.elapsed_time)
            self.storeTime(number_msg, time_msg)

        # path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/gui/data/experiment/'
        if "prediction" in kwargs and "refinement" in kwargs and "object_missed" in kwargs and "obstacle_hit" in kwargs:
            prediction = TrajectoryData(self.prediction, Bool(kwargs["object_missed"]), Bool(kwargs["object_kicked_over"]), Bool(kwargs["obstacle_hit"]))
            refinement = TrajectoryData(self.refined_trajectory, Bool(kwargs["object_missed"]), Bool(kwargs["object_kicked_over"]), Bool(kwargs["obstacle_hit"]))
            
            # store prediction and refinement in dictionary in data logger
            self.storePrediction(prediction, number_msg)
            self.storeRefinement(refinement, number_msg)

        elif "prediction" in kwargs and "refinement" not in kwargs and "object_missed" in kwargs and "obstacle_hit" in kwargs:
            prediction = TrajectoryData(self.prediction, Bool(kwargs["object_missed"]), Bool(kwargs["object_kicked_over"]), Bool(kwargs["obstacle_hit"]))
            print('store prediction')
            self.storePrediction(prediction, number_msg)

        elif "prediction" not in kwargs and "refinement" in kwargs and "object_missed" in kwargs and "obstacle_hit" in kwargs:
            refinement = TrajectoryData(self.refined_trajectory, Bool(kwargs["object_missed"]), Bool(kwargs["object_kicked_over"]), Bool(kwargs["obstacle_hit"]))
            print('store refinement')

            self.storeRefinement(refinement, number_msg)
    
    def saveData(self):
        try: 
            rospy.wait_for_service('to_csv')
            to_csv = rospy.ServiceProxy('to_csv', ToCsv)
            number_msg = Byte(self.participant_number)
            resp = to_csv(number_msg)     

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
    
    def addToModel(self):
        # 2 was too much, later trajectories had too little influence
        amount = self.experiment_variables.num_updates

        try:

            if self.method == 'online+pendant':
                rospy.wait_for_service('welford_update', timeout=2.0)
                add_to_model = rospy.ServiceProxy('welford_update', WelfordUpdate)
                logmessage = "Added " + str(amount) + " trajectories to model using Welford"
            elif self.method == 'offline+pendant':
                rospy.wait_for_service('add_demonstration', timeout=2.0)
                add_to_model = rospy.ServiceProxy('add_demonstration', AddDemonstration)
                logmessage = "Added " + str(amount) + " trajectories to model using AddDemonstration"
            elif self.method == 'online+omni':
                rospy.wait_for_service('welford_update', timeout=2.0)
                add_to_model = rospy.ServiceProxy('welford_update', WelfordUpdate)
                logmessage = "Added " + str(amount) + " trajectories to model using Welford"
            elif self.method == 'offline+omni':
                rospy.wait_for_service('add_demonstration', timeout=2.0)
                add_to_model = rospy.ServiceProxy('add_demonstration', AddDemonstration)
                logmessage = "Added " + str(amount) + " trajectories to model using AddDemonstration"

            rospy.wait_for_service('get_object_position', timeout=2.0)

            reference_frame = String()
            reference_frame.data = 'base'
            get_object = rospy.ServiceProxy('get_object_position', GetObjectPosition)

            resp = get_object(reference_frame)
            object_wrt_base = resp.object_position

            refined_trajectory, dt = self.parser.promptraj_msg_to_execution_format(self.refined_trajectory)
            refined_trajectory_wrt_object = self.parser.get_trajectory_wrt_context(refined_trajectory, self.parser.point_to_list(object_wrt_base))

            refined_trajectory_wrt_object_msg = self.parser.predicted_trajectory_to_prompTraj_message(refined_trajectory_wrt_object, self.parser.point_to_list(self.context))
            
            for i in range(amount):
                resp = add_to_model(refined_trajectory_wrt_object_msg)
            
            rospy.loginfo(logmessage)

        except (AttributeError, ValueError) as e:
                rospy.loginfo("Problem with adding trajectory: %s" %e)

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)   
    
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

    def quaternionRotation(self, axis, angle):
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

    def refinePrediction(self):
        return self.pressed_key == 'left'

    def refineRefinement(self):
        return self.pressed_key == 'right'

    def isKeyPressedLeftOrRight(self, key):
        return key == 'left' or key == 'right'

    def waitForKeyPress(self):
        self.resetKeyPressed()
        while True:
            if self.isKeyPressedLeftOrRight(self.pressed_key):
                print(str(self.pressed_key) + " key presssed")
                return

    def isArmEnabled(self):
        return self.en_arm == 1

    def resetKeyPressed(self):
        self.pressed_key = ""

    def startTrial(self):
        if self.method == 'offline+pendant':
            rospy.wait_for_service('/offline_pendant/set_teach_state', timeout=2.0)
            set_teach_state = rospy.ServiceProxy('/offline_pendant/set_teach_state', SetTeachState)
            set_teach_state(Bool(False))
        elif self.method == 'offline+omni':
            rospy.wait_for_service('/trajectory_teaching/set_teach_state', timeout=2.0)
            set_teach_state = rospy.ServiceProxy('/trajectory_teaching/set_teach_state', SetTeachStateOmni)
            set_teach_state(Bool(False))

        # self.operator_gui_text_pub.publish(String("CHECK CHEK 112"))
        self.openGripper()
        self.goToInitialPose()
        time.sleep(5)

        self.setDishwasherPosition()
        self.setObjectPosition()
        
        time.sleep(4)

        self.getContext()
        
        self.setDataLoggerParameters()
        self.predict()
        
        self.visualize('prediction')
        
        self.traffic_light_updater.update('red')
        self.text_updater.update("AUTONOMOUS EXECUTION")
        obstacle_hit, object_reached, object_kicked_over = self.executeTrajectory(self.prediction)
        
        # store prediction along with failure
        self.storeData(prediction=1, obstacle_hit=obstacle_hit, object_missed = not object_reached, object_kicked_over=object_kicked_over)
        self.saveData()
        
        # update text in operator gui
        if obstacle_hit or not object_reached or object_kicked_over:
            self.text_updater.update("FAILURE:")
        else:
            self.text_updater.update("SUCCESS!")
            
            # last object position reached
            if self.current_object_position >= self.num_object_positions:
                # build initial model again
                try:
                    rospy.wait_for_service('build_initial_model', timeout=2.0)
                    build_init_model = rospy.ServiceProxy('build_initial_model', BuildInitialModel)
                    build_init_model()
                except (rospy.ServiceException, rospy.ROSException) as e:
                    print("Service call failed: %s" %e)
                
                # shift to next model
                # 1st object, 1st trial
                self.current_model += 1
                self.current_object_position = 1
                self.current_trial = 1
            
            # last position not reached
            else:
                self.current_object_position += 1
                self.current_trial = 1

            # for going to the next loop iteration in start() function
            return 1
    
        if obstacle_hit:
            self.text_updater.append("OBSTACLE HIT")
        if not object_reached:
            self.text_updater.append("OBJECT MISSED")
        if object_kicked_over:
            self.text_updater.append("OBJECT KICKED OVER")

        time.sleep(2)
        # loop the refinement until max refinements has reached
        # or the last refinement was successful
        number_of_refinements = 0
        
        if self.method == 'online+pendant':
            while (obstacle_hit or not object_reached or object_kicked_over) and number_of_refinements <= self.max_refinements-1: # -1 to get 5 instead of 6 max refinements
                print("Trajectory failure!")

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
                
                elif self.refineRefinement():

                    # we only need to start the timer if it is equal to zero, else just keep the timer running
                    if self.start_time == 0:
                        # start timer
                        self.startTimer()
                    else: pass

                    resp = refine_trajectory(self.refined_trajectory, self.T_desired)

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
                    self.text_updater.update("FAILURE:")
                else:
                    self.text_updater.update("SUCCESS!")

                if obstacle_hit:
                    self.text_updater.append("OBSTACLE HIT")
                if not object_reached:
                    self.text_updater.append("OBJECT MISSED")
                if object_kicked_over:
                    self.text_updater.append("OBJECT KICKED OVER")

                time.sleep(2)
                # store refinement along with if it failed or not
                self.storeData(refinement=1, obstacle_hit=obstacle_hit, object_missed = not object_reached, object_kicked_over=object_kicked_over)
               
                number_of_refinements += 1

                # increment number of refinements
                rospy.wait_for_service('set_number_of_refinements', timeout=2.0)
                
                set_nr_refinement = rospy.ServiceProxy('set_number_of_refinements', SetNumberOfRefinements)
                set_nr_refinement(Byte(self.participant_number), Byte(number_of_refinements))

                rospy.loginfo("Got a refined trajectory")

                self.visualize('both')
                print("number of refinement = " + str(number_of_refinements))
                self.number_of_refinements_updater.update(str(number_of_refinements))

                if number_of_refinements >= self.max_refinements:
                    self.text_updater.update("MAX REFINEMENT AMOUNT REACHED!")
        
        elif self.method == 'offline+pendant':

            while (obstacle_hit or not object_reached or object_kicked_over) and number_of_refinements <= self.max_refinements-1: # -1 to get 5 instead of 6 max refinements
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

                self.stopTimer()
                obstacle_hit, object_reached, object_kicked_over = self.executeTrajectory(self.refined_trajectory)
                self.startTimer()
                
                with open('/home/fmeccanici/Documents/thesis/thesis_workspace/src/experiment/debug/refined_trajectory.txt', 'w+') as f:
                    f.write(str(self.refined_trajectory))
                    
                print("\n")

                rospy.loginfo("object missed: " + str(not object_reached))
                rospy.loginfo("obstacle hit: " + str(obstacle_hit))
                rospy.loginfo("object kicked over: " + str(object_kicked_over))

                print("\n")

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

                time.sleep(2)
                # store refinement along with if it failed or not
                self.storeData(refinement=1, obstacle_hit=obstacle_hit, object_missed = not object_reached, object_kicked_over=object_kicked_over)
               
                # increment number of refinements
                rospy.wait_for_service('set_number_of_refinements', timeout=2.0)
                
                number_of_refinements += 1
                set_nr_refinement = rospy.ServiceProxy('set_number_of_refinements', SetNumberOfRefinements)
                set_nr_refinement(Byte(self.participant_number), Byte(number_of_refinements))

                rospy.loginfo("Got a refined trajectory")

                print("number of refinement = " + str(number_of_refinements))
                self.number_of_refinements_updater.update(str(number_of_refinements))

                if number_of_refinements >= self.max_refinements:
                    self.text_updater.update("MAX REFINEMENT AMOUNT REACHED!")
        
            rospy.wait_for_service('offline_pendant/clear_waypoints', timeout=2.0)
            clear_waypoints = rospy.ServiceProxy('offline_pendant/clear_waypoints', ClearWaypoints)
            clear_waypoints()
        
        elif self.method == 'online+omni':
            while (obstacle_hit or not object_reached or object_kicked_over) and number_of_refinements <= self.max_refinements-1: # -1 to get 5 instead of 6 max refinements
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
                    self.text_updater.update("FAILURE:")
                else:
                    self.text_updater.update("SUCCESS!")

                if obstacle_hit:
                    self.text_updater.append("OBSTACLE HIT")
                if not object_reached:
                    self.text_updater.append("OBJECT MISSED")
                if object_kicked_over:
                    self.text_updater.append("OBJECT KICKED OVER")

                time.sleep(2)
                # store refinement along with if it failed or not
                self.storeData(refinement=1, obstacle_hit=obstacle_hit, object_missed = not object_reached, object_kicked_over=object_kicked_over)
               
                number_of_refinements += 1

                # increment number of refinements
                rospy.wait_for_service('set_number_of_refinements', timeout=2.0)
                
                set_nr_refinement = rospy.ServiceProxy('set_number_of_refinements', SetNumberOfRefinements)
                set_nr_refinement(Byte(self.participant_number), Byte(number_of_refinements))

                rospy.loginfo("Got a refined trajectory")

                self.visualize('both')
                print("number of refinement = " + str(number_of_refinements))
                self.number_of_refinements_updater.update(str(number_of_refinements))

                if number_of_refinements >= self.max_refinements:
                    self.text_updater.update("MAX REFINEMENT AMOUNT REACHED!")
        
        elif self.method == 'offline+omni':

            while (obstacle_hit or not object_reached or object_kicked_over) and number_of_refinements <= self.max_refinements-1: # -1 to get 5 instead of 6 max refinements
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

                obstacle_hit, object_reached, object_kicked_over = self.executeTrajectory(self.refined_trajectory)
                

                with open('/home/fmeccanici/Documents/thesis/thesis_workspace/src/experiment/debug/refined_trajectory.txt', 'w+') as f:
                    f.write(str(self.refined_trajectory))
                    
                print("\n")

                rospy.loginfo("object missed: " + str(not object_reached))
                rospy.loginfo("obstacle hit: " + str(obstacle_hit))
                rospy.loginfo("object kicked over: " + str(object_kicked_over))

                print("\n")

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

                time.sleep(2)
                # store refinement along with if it failed or not
                self.storeData(refinement=1, obstacle_hit=obstacle_hit, object_missed = not object_reached, object_kicked_over=object_kicked_over)
               
                # increment number of refinements
                rospy.wait_for_service('set_number_of_refinements', timeout=2.0)
                
                number_of_refinements += 1
                set_nr_refinement = rospy.ServiceProxy('set_number_of_refinements', SetNumberOfRefinements)
                set_nr_refinement(Byte(self.participant_number), Byte(number_of_refinements))

                rospy.loginfo("Got a refined trajectory")

                print("number of refinement = " + str(number_of_refinements))
                self.number_of_refinements_updater.update(str(number_of_refinements))
                
                rospy.wait_for_service('trajectory_teaching/clear_trajectory', timeout=2.0)
                clear_trajectory = rospy.ServiceProxy('trajectory_teaching/clear_trajectory', ClearTrajectory)
                resp = clear_trajectory()

                if number_of_refinements >= self.max_refinements:
                    self.text_updater.update("MAX REFINEMENT AMOUNT REACHED!")
        
            rospy.wait_for_service('trajectory_teaching/clear_trajectory', timeout=2.0)
            clear_trajectory = rospy.ServiceProxy('trajectory_teaching/clear_trajectory', ClearTrajectory)
            clear_trajectory()
        
        self.number_of_refinements_updater.update(str(0))
        
        ####### update model #######
        if number_of_refinements == 0:
            pass
        else:
            print('adding refinement to model')
            self.addToModel()
        
        self.stopTimer()
        
        # store time
        self.storeData(time=True)
        
        ###### save data ######
        self.saveData()
        self.zeroTimer()

        self.current_trial += 1

        # logic to go to next model and object position
        # last trial reached
        if self.current_trial >= self.num_trials+1:

            # last object position reached
            if self.current_object_position >= self.num_object_positions:
                
                if not self.current_model == self.num_models:
                    # build initial model again 
                    try:
                        rospy.wait_for_service('build_initial_model', timeout=2.0)
                        build_init_model = rospy.ServiceProxy('build_initial_model', BuildInitialModel)
                        build_init_model()
                    except (rospy.ServiceException, rospy.ROSException) as e:
                        print("Service call failed: %s" %e)
                    
                    # shift to next model
                    # 1st object, 1st trial
                    self.current_model += 1
                    self.current_object_position = 1
                    self.current_trial = 1
            
            # last position not reached
            else:
                self.current_object_position += 1
                self.current_trial = 1

        self.setDataLoggerParameters()
        
        return 0

    def start(self):
        self.loadParticipant()
        self.initializeHeadLiftJoint()

        for model in self.models:
            print('model = ' + str(model))
            for object_position in self.object_positions:
                self.getContext()
                self.setDataLoggerParameters()
                self.y_position = self.determineYPosition()

                for trial in self.trials:
                    print('object position = ' + str(object_position))
                    print('trial = ' + str(trial))

                    success = self.startTrial()
                    if success:
                        break


            # reset y position after adapting a model
            self.y_position_step_dict = copy.deepcopy(self.experiment_variables.y_position_step_dict)


if __name__ == "__main__":
    experiment = ExperimentNode()
    experiment.start()