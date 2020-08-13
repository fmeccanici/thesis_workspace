#! /usr/bin/env/ python2.7

import rospy, copy, time, random
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from learning_from_demonstration.srv import GetObjectPosition, GoToPose, MakePrediction, AddDemonstration, GetContext
from teach_pendant.srv import AddWaypoint, GetTeachState, GetDemonstrationPendant, SetTeachState
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState, SetModelConfiguration
from execution_failure_detection.srv import GetExecutionFailure, SetExpectedObjectPosition
from trajectory_visualizer.srv import VisualizeTrajectory, ClearTrajectories
from trajectory_visualizer.msg import TrajectoryVisualization
from learning_from_demonstration_python.trajectory_parser import trajectoryParser
from learning_from_demonstration_python.trajectory_resampler import trajectoryResampler

from experiment_variables.experiment_variables import ExperimentVariables

class TeachPendantTester(object):
    def __init__(self):
        rospy.init_node('teach_pendant_tester')
        self.parser = trajectoryParser()
        self.resampler = trajectoryResampler()

        self.experiment_variables = ExperimentVariables()
        self.y_position_step_dict = copy.deepcopy(self.experiment_variables.y_position_step_dict)
        self.lift_goal_pub = rospy.Publisher('/lift_controller_ref', JointState, queue_size=10)
        self.head_goal_pub = rospy.Publisher('/head_controller_ref', JointState, queue_size=10)
        self.num_object_positions = self.experiment_variables.num_object_positions
        self.num_trials = self.experiment_variables.num_trials

        self.trials = range(1,self.num_trials+1)

        self.object_positions = range(1,self.num_object_positions+1)

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
    
    def determineYPosition(self):
        random_object_position = random.choice(list(self.y_position_step_dict.keys()))
        
        step = self.y_position_step_dict[random_object_position]

        del self.y_position_step_dict[random_object_position]

        y0 = self.experiment_variables.y0

        return y0 - step
    
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

    def getContext(self):
        try:
            rospy.wait_for_service('get_context', timeout=2.0)

            get_context = rospy.ServiceProxy('get_context', GetContext)
            resp = get_context()
            self.context = resp.context

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)  
    
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
    
    def addToModel(self):
        # 2 was too much, later trajectories had too little influence
        amount = self.experiment_variables.num_updates

        try:

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
    
    def startTrial(self):
        rospy.wait_for_service('/offline_pendant/set_teach_state', timeout=2.0)
        set_teach_state = rospy.ServiceProxy('/offline_pendant/set_teach_state', SetTeachState)
        set_teach_state(Bool(False))

        self.goToInitialPose()
        time.sleep(5)
        self.setObjectPosition()
        time.sleep(4)
        self.getContext()
        self.predict()
        self.visualize('prediction')


        rospy.wait_for_service('/offline_pendant/add_waypoint', timeout=2.0)
        add_waypoint = rospy.ServiceProxy('/offline_pendant/add_waypoint', AddWaypoint)
        add_waypoint()

        set_teach_state(Bool(True))
        rospy.wait_for_service('offline_pendant/get_teach_state', timeout=2.0)
        get_teach_state = rospy.ServiceProxy('offline_pendant/get_teach_state', GetTeachState)
        resp = get_teach_state()
        isTeachingOffline = resp.teach_state.data      
        
        # use teach_pendant node to teach offline
        while isTeachingOffline:
            resp = get_teach_state()
            isTeachingOffline = resp.teach_state.data 
        
        rospy.wait_for_service('get_demonstration_pendant', timeout=2.0)
        get_demo_pendant = rospy.ServiceProxy('get_demonstration_pendant', GetDemonstrationPendant)
        resp = get_demo_pendant()

        set_teach_state(Bool(False))
        # self.goToInitialPose()
        self.setObjectPosition()
        self.refined_trajectory = resp.demo

        self.visualize('both')
        self.addToModel()

    def run(self):
        self.initializeHeadLiftJoint()

        for object_position in self.object_positions:
            self.getContext()
            self.y_position = self.determineYPosition()

            for trial in self.trials:
                print('object position = ' + str(object_position))
                print('trial = ' + str(trial))

                self.startTrial()

if __name__ == "__main__":
    tester = TeachPendantTester()
    tester.run()