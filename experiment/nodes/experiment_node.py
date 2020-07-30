#!/usr/bin/env python

import rospy, rospkg, roslaunch, time
from learning_from_demonstration_python.trajectory_parser import trajectoryParser
from learning_from_demonstration_python.trajectory_resampler import trajectoryResampler
from data_logger_python.text_updater import TextUpdater

# import ros messages
from sensor_msgs.msg import JointState
from data_logger.msg import TrajectoryData, OperatorGUIinteraction
from std_msgs.msg import Byte, Bool, Float32, String
from geometry_msgs.msg import Point, Pose

from gazebo_msgs.msg import ModelState 
from trajectory_visualizer.msg import TrajectoryVisualization

# import ros services
from data_logger.srv import (CreateParticipant, AddRefinement,
                                SetPrediction, ToCsv, SetObstaclesHit,
                                SetObjectMissed, IncrementNumberOfRefinements,
                                SetParameters, SetTime)

from learning_from_demonstration.srv import (GoToPose, MakePrediction, 
                                                GetContext, GetObjectPosition,
                                                WelfordUpdate, ExecuteTrajectory)
from gazebo_msgs.srv import SetModelState
from trajectory_visualizer.srv import VisualizeTrajectory, ClearTrajectories
from trajectory_refinement.srv import RefineTrajectory, CalibrateMasterPose
from execution_failure_detection.srv import GetExecutionFailure, SetExpectedObjectPosition
from experiment.srv import SetText

class ExperimentNode(object):
    def __init__(self):
        rospy.init_node('experiment')
        self._getParameters()
        self.rospack = rospkg.RosPack()
        self.parser = trajectoryParser()
        self.resampler = trajectoryResampler()
        self.text_updater = TextUpdater()

        self.nodes = {}
        self.method_mapping = {'online+omni':1, 'offline+omni':2, 'online+pendant':3, 'offline+pendant':4}

        self.num_trials = 5
        self.num_object_positions = 6
        self.trials = range(1,self.num_trials)
        self.object_positions = range(1,self.num_object_positions+1)
        self.max_refinements = 5
        self.elapsed_time = 0
        self.elapsed_time_prev = 0

        self.T_desired = 10.0
        self.start_time = 0
        self.set_data = False

        self.current_trial = 1
        self.current_object_position = 1
        
        # ros stuff
        self.lift_goal_pub = rospy.Publisher('/lift_controller_ref', JointState, queue_size=10)
        self.head_goal_pub = rospy.Publisher('/head_controller_ref', JointState, queue_size=10)
        self.operator_gui_interaction_sub = rospy.Subscriber('/operator_gui_interaction', OperatorGUIinteraction, self._operatorGuiInteraction)
        # self.operator_gui_text_pub = rospy.Publisher('/operator_gui/text', String, queue_size=10)

    def _operatorGuiInteraction(self, data):    
        if data.refine_prediction.data:
            self.refine = 'prediction'
        elif data.refine_refinement.data:
            self.refine = 'refinement'
        
        if self.set_data == False:
            self.participant_number = data.number.data
            self.age = data.age.data
            self.gender = data.gender.data
            self.set_data = True

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
    
    def setObjectPosition(self):
        try:
            object_position = ModelState()
            object_position.model_name = 'aruco_cube'

            step = 0.07
            x = 0.83
            # dishwasher moved backwards    
            if self.current_object_position == 1:
                object_position.pose.position.x = x
                object_position.pose.position.y = 0.25
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 2:
                object_position.pose.position.x = x
                object_position.pose.position.y = 0.25 - step
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 3:
                object_position.pose.position.x = x
                object_position.pose.position.y = 0.25 - 2*step
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 4:
                object_position.pose.position.x = x
                object_position.pose.position.y = 0.25 - 3*step
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 5:
                object_position.pose.position.x = x
                object_position.pose.position.y = 0.25 - 4*step
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 6:
                object_position.pose.position.x = x
                object_position.pose.position.y = 0.25 - 5*step
                object_position.pose.position.z = 0.7
            

            # dishwasher
            """
            if self.current_object_position == 1:
                object_position.pose.position.x = 0.58
                object_position.pose.position.y = 0.23
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 2:
                object_position.pose.position.x = 0.58
                object_position.pose.position.y = 0.06
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 3:
                object_position.pose.position.x = 0.58
                object_position.pose.position.y = -0.1
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 4:
                object_position.pose.position.x = 0.68
                object_position.pose.position.y = 0.23
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 5:
                object_position.pose.position.x = 0.68
                object_position.pose.position.y = 0.06
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 6:
                object_position.pose.position.x = 0.68
                object_position.pose.position.y = -0.1
                object_position.pose.position.z = 0.7
            """
            # table
            """
            if self.current_object_position == 1:
                object_position.pose.position.x = 0.82
                object_position.pose.position.y = 0.3
                object_position.pose.position.z = 0.9
            elif self.current_object_position == 2:
                object_position.pose.position.x = 0.82
                object_position.pose.position.y = 0.0
                object_position.pose.position.z = 0.9
            elif self.current_object_position == 3:
                object_position.pose.position.x = 0.65

                # 0.29 instead of 0.30 because otherwise object is on the edge of not
                # being detected
                object_position.pose.position.y = 0.29
                object_position.pose.position.z = 0.9
            elif self.current_object_position == 4:
                object_position.pose.position.x = 0.65
                object_position.pose.position.y = 0.0
                object_position.pose.position.z = 0.9
            """

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

            """
            # initial pose dishwasher
            pose.position.x = 0.472
            pose.position.y = 0.069
            pose.position.z = 0.433

            pose.orientation.x = 0.997
            pose.orientation.y = -0.033
            pose.orientation.z = -0.056
            pose.orientation.w = 0.041
            """

            # initial pose dishwasher moved backwards
            """
            pose.position.x = 0.579
            pose.position.y = 0.09
            pose.position.z = 0.481

            pose.orientation.x = 0.992
            pose.orientation.y = -0.033
            pose.orientation.z = -0.121
            pose.orientation.w = 0.016
            """
            
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
        resp = set_parameters(number_msg, object_position_msg, trial_msg, method_msg)

    def loadOrCreateParticipant(self):
        try: 
            rospy.wait_for_service('create_participant', timeout=2.0)
            create_participant = rospy.ServiceProxy('create_participant', CreateParticipant)
            number_msg = Byte()
            
            # wait until we have operator interaction
            text = "FILL IN FORM"
            self.text_updater.update(text)
            
            rospy.wait_for_message('operator_gui_interaction', OperatorGUIinteraction)
            number_msg.data = self.participant_number
            self.text_updater.empty()

            # dummy variable names --> not necessary when data exists
            gender_msg = Bool(self.gender)
            age_msg = Byte(self.age)

            resp = create_participant(number_msg, gender_msg, age_msg)     

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
        amount = 2

        try:
            rospy.wait_for_service('welford_update', timeout=2.0)
            rospy.wait_for_service('get_object_position', timeout=2.0)

            reference_frame = String()
            reference_frame.data = 'base'
            get_object = rospy.ServiceProxy('get_object_position', GetObjectPosition)

            resp = get_object(reference_frame)
            object_wrt_base = resp.object_position

            refined_trajectory, dt = self.parser.promptraj_msg_to_execution_format(self.refined_trajectory)
            refined_trajectory_wrt_object = self.parser.get_trajectory_wrt_context(refined_trajectory, self.parser.point_to_list(object_wrt_base))

            welford_update = rospy.ServiceProxy('welford_update', WelfordUpdate)
            refined_trajectory_wrt_object_msg = self.parser.predicted_trajectory_to_prompTraj_message(refined_trajectory_wrt_object, self.parser.point_to_list(self.context))
            
            for i in range(amount):
                resp = welford_update(refined_trajectory_wrt_object_msg)
            
            rospy.loginfo("Added " + str(amount) + " trajectories to model using Welford")


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
    
    def startTrial(self):
        # self.operator_gui_text_pub.publish(String("CHECK CHEK 112"))

        self.goToInitialPose()
        time.sleep(2)

        self.setObjectPosition()
        time.sleep(4)

        self.getContext()
        
        self.setDataLoggerParameters()
        self.predict()
        self.visualize('prediction')

        if self.method == 'online+pendant':
            self.text_updater.update("AUTONOMOUS EXECUTION")

            obstacle_hit, object_reached, object_kicked_over = self.executeTrajectory(self.prediction)
            
            # print(not object_reached)
            # print(obstacle_hit)
            # print(object_kicked_over)

            # store prediction along with failure
            self.storeData(prediction=1, obstacle_hit=obstacle_hit, object_missed = not object_reached, object_kicked_over=object_kicked_over)
            self.saveData()
            
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

            # loop the refinement until max refinements has reached
            # or the last refinement was successful
            number_of_refinements = 0

            while (obstacle_hit or not object_reached or object_kicked_over) and number_of_refinements <= self.max_refinements-1: # -1 to get 5 instead of 6 max refinements
                print("Trajectory failure!")

                self.goToInitialPose()
                self.setObjectPosition()
                
                # wait until the operator clicked the red or green button
                self.text_updater.update("REFINE RED OR GREEN?")
                rospy.wait_for_message('operator_gui_interaction', OperatorGUIinteraction)
                refine_trajectory = rospy.ServiceProxy('refine_trajectory', RefineTrajectory)
                
                if self.refine == 'prediction':

                    # we only need to start the timer if it is equal to zero, else just keep the timer running
                    if self.start_time == 0:
                        # start timer
                        self.startTimer()
                    else: pass

                    resp = refine_trajectory(self.prediction, self.T_desired)
                
                elif self.refine == 'refinement':

                    # we only need to start the timer if it is equal to zero, else just keep the timer running
                    if self.start_time == 0:
                        # start timer
                        self.startTimer()
                    else: pass

                    resp = refine_trajectory(self.refined_trajectory, self.T_desired)


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
                rospy.wait_for_service('increment_number_of_refinements', timeout=2.0)
                
                increment_refinement = rospy.ServiceProxy('increment_number_of_refinements', IncrementNumberOfRefinements)
                increment_refinement(Byte(self.participant_number))
                number_of_refinements += 1

                rospy.loginfo("Got a refined trajectory")

                self.visualize('both')
                print("number of refinement = " + str(number_of_refinements))

                if number_of_refinements >= self.max_refinements:
                    self.text_updater.update("MAX REFINEMENT AMOUNT REACHED!")

            ####### update model #######
            self.goToInitialPose()
            self.addToModel()
            self.stopTimer()
            
            # store time
            self.storeData(time=True)
            
            ###### save data ######
            self.saveData()
            self.zeroTimer()

            self.current_trial += 1

            if self.current_trial >= self.num_trials:
                self.current_object_position += 1
                self.current_trial = 1
            
            self.setDataLoggerParameters()

    def start(self):
        self.loadOrCreateParticipant()
        self.initializeHeadLiftJoint()

        for object_position in self.object_positions:
            self.getContext()
            self.setDataLoggerParameters()
            for trial in self.trials:
                self.startTrial()


if __name__ == "__main__":
    experiment = ExperimentNode()
    experiment.start()