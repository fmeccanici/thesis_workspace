#! /usr/bin/env python

import rospy, copy, time, ast, roslaunch, rospkg, os
from subprocess import call
from experiment_variables.experiment_variables import ExperimentVariables

from execution_failure_detection.srv import GetExecutionFailure
from learning_from_demonstration.srv import ExecuteTrajectory, GoToPose, GetContext, MakePrediction
from gazebo_msgs.msg import ModelState, LinkState
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState, SetLinkState
from trajectory_visualizer.msg import TrajectoryVisualization
from trajectory_visualizer.srv import VisualizeTrajectory, ClearTrajectories
from learning_from_demonstration_python.trajectory_parser import trajectoryParser

from learning_from_demonstration.srv import WelfordUpdate, AddDemonstration, BuildInitialModel
from execution_failure_detection.srv import GetExecutionFailure, SetExpectedObjectPosition
import numpy as np
import random

from trajectory_visualizer.msg import TrajectoryVisualization
from trajectory_visualizer.srv import VisualizeTrajectory, ClearTrajectories

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class DataCreator(object):
    def __init__(self):
        self.experiment_variables = ExperimentVariables()
        self.num_object_positions = self.experiment_variables.num_object_positions
        self.num_trials = self.experiment_variables.num_trials

        self.num_methods = self.experiment_variables.num_methods
        self.object_positions = self.experiment_variables.object_positions
        self.num_models = self.experiment_variables.num_models

        self.methods = {}
        self.parser = trajectoryParser()
        self.nodes = {}
        self._rospack = rospkg.RosPack()

        self.trials = {}
        self.data = {}

        self.predicted_trajectory = {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [], 'object_missed': False, 'obstacle_hit': False,'object_kicked_over': False,  'success': True}
        self.refined_trajectory = {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [], 'object_missed': False, 'obstacle_hit': False, 'object_kicked_over': False, 'success': True}
        
        self.debug_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/data/debug/'

        rospy.init_node('data_creator')
        self.lift_goal_pub = rospy.Publisher('/lift_controller_ref', JointState, queue_size=10)
        self.head_goal_pub = rospy.Publisher('/head_controller_ref', JointState, queue_size=10)

    def executeTrajectory(self, traj):
        rospy.wait_for_service('execute_trajectory', timeout=2.0)
        execute_trajectory = rospy.ServiceProxy('execute_trajectory', ExecuteTrajectory)
        self.T_desired = 20
        resp = execute_trajectory(traj, self.T_desired)

        return resp.obstacle_hit.data, resp.object_reached.data, resp.object_kicked_over.data
    
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

    def setObjectPosition(self, before_or_after):
        try:
            object_position = ModelState()
            object_position.model_name = 'aruco_cube'

            step = 0.1

            if before_or_after == 'after':
                x = 0.8 
            elif before_or_after == 'before':
                # 0.81 when dishwasher1
                x = 0.8

            y0 = self.experiment_variables.y0
            
            # dishwasher moved backwards    
            if self.current_object_position == 1:
                object_position.pose.position.x = x
                object_position.pose.position.y = y0
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 2:
                object_position.pose.position.x = x
                object_position.pose.position.y = y0 - step
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 3:
                object_position.pose.position.x = x
                object_position.pose.position.y = y0 - 2*step
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 4:
                object_position.pose.position.x = x
                object_position.pose.position.y = y0 - 3*step
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 5:
                object_position.pose.position.x = x
                object_position.pose.position.y = y0 - 4*step
                object_position.pose.position.z = 0.7
            elif self.current_object_position == 6:
                object_position.pose.position.x = x
                object_position.pose.position.y = y0 - 5*step
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

    def getContext(self):
        try:
            rospy.wait_for_service('get_context', timeout=2.0)

            get_context = rospy.ServiceProxy('get_context', GetContext)
            resp = get_context()
            self.context = resp.context

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)       

    def makePrediction(self):
        try:
            rospy.wait_for_service('make_prediction', timeout=2.0)

            make_prediction = rospy.ServiceProxy('make_prediction', MakePrediction)
            resp = make_prediction(self.context)
            self.prediction = copy.deepcopy(resp.prediction)
        
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
        
        except AttributeError:
            rospy.loginfo("Context not yet extracted!")
        
    def toTxt(self):
        with open(self.path + 'data.txt', 'w+') as f:
            f.write(str(self.data))

    def setPath(self, path):
        self.path = path

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

    def setDishwasherPosition(self, before_or_after='after'):
        try:
            dishwasher = ModelState()
            dishwasher.model_name = 'dishwasher'

            if before_or_after == 'after':
                dishwasher.pose.position.x = 1.75
            elif before_or_after == 'before':
                # 1.7 when dishwasher1
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

            rospy.wait_for_service('/gazebo/set_link_state')

            # if before_or_after == 'before':
            #     set_link_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
            #     upper_basket = LinkState()
            #     upper_basket.link_name = 'upper_basket_link'
            #     upper_basket.pose.position.x = 0.961216312766
            #     upper_basket.pose.position.y = -0.185586276876
            #     upper_basket.pose.position.z = 0.523004713851
            #     upper_basket.pose.orientation.x = 0.505285214303
            #     upper_basket.pose.orientation.y = -0.49466017036
            #     upper_basket.pose.orientation.z = -0.494658398524
            #     upper_basket.pose.orientation.w = 0.505283323331

            #     resp = set_link_state(upper_basket)

            return resp.success

        except ValueError:
            rospy.loginfo("Invalid value for position!")

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s"%e)

    def createDataBeforeExperiment(self):

        self.setPath('/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/before_experiment/dishwasher2/')
        
        for i in range(self.num_trials + 1):
            self.trials[i+1] = {
                'predicted_trajectory': copy.deepcopy(self.predicted_trajectory), 'context': []}
        
        for j in range(self.num_object_positions + 1):
            self.data[j+1] = {
            'trial': copy.deepcopy(self.trials)}
        
        for position in self.object_positions:
            for trial in range(1, self.num_trials+1):
                self.openGripper()

                self.current_object_position = position
                print("object position: " + str(position))
                print("trial: " + str(trial))

                # move ee to initial pose
                self.goToInitialPose()

                # wait until arm is not in the way of the object
                self.setDishwasherPosition(before_or_after='before')

                time.sleep(2)
                self.setObjectPosition('before')
                
                time.sleep(2)

                # get new context
                self.getContext()
                self.makePrediction()
                
                rospy.wait_for_service('visualize_trajectory', timeout=2.0)
                
                # visualize prediction
                visualize_trajectory = rospy.ServiceProxy('visualize_trajectory', VisualizeTrajectory)
                visualization_msg = TrajectoryVisualization()
                visualization_msg.pose_array = self.prediction.poses
                visualization_msg.r = 1.0
                visualization_msg.g = 0.0
                visualization_msg.b = 0.0

                resp = visualize_trajectory(visualization_msg)
                
                obstacle_hit, object_reached, object_kicked_over = self.executeTrajectory(self.prediction)

                x = []
                y = []
                z = []
                qx = []
                qy = []
                qz = []
                qw = []
                t = []

                for i,data in enumerate(self.prediction.poses):
                    x.append(data.position.x)
                    y.append(data.position.y)
                    z.append(data.position.z)
                    qx.append(data.orientation.x)
                    qy.append(data.orientation.y)
                    qz.append(data.orientation.z)
                    qw.append(data.orientation.w)
                    t.append(self.prediction.times[i])

                self.data[position]['trial'][trial]['predicted_trajectory']['x'] = x
                self.data[position]['trial'][trial]['predicted_trajectory']['y'] = y
                self.data[position]['trial'][trial]['predicted_trajectory']['z'] = z
                self.data[position]['trial'][trial]['predicted_trajectory']['qx'] = qx
                self.data[position]['trial'][trial]['predicted_trajectory']['qy'] = qy
                self.data[position]['trial'][trial]['predicted_trajectory']['qz'] = qz
                self.data[position]['trial'][trial]['predicted_trajectory']['qw'] = qw
                self.data[position]['trial'][trial]['predicted_trajectory']['t'] = t
                self.data[position]['trial'][trial]['predicted_trajectory']['object_missed'] = not object_reached
                self.data[position]['trial'][trial]['predicted_trajectory']['obstacle_hit'] = obstacle_hit
                self.data[position]['trial'][trial]['predicted_trajectory']['object_kicked_over'] = object_kicked_over

                if not object_reached or obstacle_hit or object_kicked_over:
                    self.data[position]['trial'][trial]['predicted_trajectory']['success'] = False
                
                self.data[position]['trial'][trial]['context'] = [self.context.x, self.context.y, self.context.z]

                self.toTxt()
                clear_trajectories = rospy.ServiceProxy('clear_trajectories', ClearTrajectories)
                resp = clear_trajectories()

                time.sleep(1)

    def loadData(self):
        data_path = self.path + 'data.txt'
        
        with open(data_path, "r") as infile:
            outfile = ast.literal_eval(infile.read())

            self.gender = outfile['gender']
            self.number = outfile['number']
            self.age = outfile['age']
            
            for i in range(self.num_methods):
                self.methods[i+1] = outfile['method'][i+1]

    def dictToList(self, trajectory_dict):
        x = trajectory_dict['x']
        y = trajectory_dict['y']
        z = trajectory_dict['z']
        qx = trajectory_dict['qx']
        qy = trajectory_dict['qy']
        qz = trajectory_dict['qz']
        qw = trajectory_dict['qw']
        t = trajectory_dict['t']

        trajectory_list = []

        for i in range(len(x)):
            trajectory_list.append([x[i], y[i], z[i], qx[i], qy[i], qz[i], qw[i], t[i]])

        return trajectory_list

    def getTrajectory(self, which, participant_number, method, model, object_position, trial):

        if which == 'refinement':
            trajectory = 'refined_trajectory'
        elif which == 'prediction':
            trajectory == 'predicted_trajectory'

        traj_wrt_base = copy.deepcopy(self.methods[method]['model'][model]['object_position'][object_position]['trial'][trial][trajectory])
        traj_wrt_base = self.dictToList(traj_wrt_base)
        context = copy.deepcopy(self.methods[method]['model'][model]['object_position'][object_position]['trial'][trial]['context'])
        
        r_object_wrt_ee = [context[0] / 10, context[1] / 10, context[2] / 10] 
        
        # calculate object wrt base
        r_object_wrt_base = np.asarray(r_object_wrt_ee) + np.asarray([traj_wrt_base[0][0], traj_wrt_base[0][1], traj_wrt_base[0][2]])

        # store data for debugging
        file_name = 'traj_wrt_base_trial_' + str(trial) + '.txt'

        with open(self.debug_path + file_name, 'w+') as f:
            f.write(str(traj_wrt_base))
        
        file_name = 'context_trial_' + str(trial) 
        
        with open(self.debug_path + file_name, 'w+') as f:
            f.write(str(context))

        traj_wrt_object = self.parser.get_trajectory_wrt_context(traj_wrt_base, r_object_wrt_base)

        file_name = 'traj_wrt_object_trial_' + str(trial) + '.txt'

        with open(self.debug_path + file_name, 'w+') as f:
            f.write(str(traj_wrt_object))

        print("Found refinement")
        return traj_wrt_object, context

    def addToModel(self, trajectory, context):
        # 2 was too much, later trajectories had too little influence
        amount = self.experiment_variables.num_updates

        try:
            if self.method == 3 or self.method == 1:
                rospy.wait_for_service('welford_update', timeout=2.0)
                add_to_model = rospy.ServiceProxy('welford_update', WelfordUpdate)
                logmessage = "Added " + str(amount) + " trajectories to model using Welford"
            elif self.method == 4 or self.method == 2:
                rospy.wait_for_service('add_demonstration', timeout=2.0)
                add_to_model = rospy.ServiceProxy('add_demonstration', AddDemonstration)
                logmessage = "Added " + str(amount) + " trajectories to model using AddDemonstration"
                
            trajectory_wrt_object_msg = self.parser.predicted_trajectory_to_prompTraj_message(trajectory, context)
            
            for i in range(amount):
                resp = add_to_model(trajectory_wrt_object_msg)
            
            rospy.loginfo(logmessage)


        except (AttributeError, ValueError) as e:
                rospy.loginfo("Problem with adding trajectory: %s" %e)

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)  

    def visualizeExperimentData(self, participant_number, method, model):
        self.setPath('/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/participant_' + str(participant_number)+ '/')
        self.loadData()
        
        # adapt model using experiment data
        for object_position in self.object_positions:
            for trial in range(1, self.num_trials+1):
                
                # get trajectory wrt context
                try:
                    # trajectory_for_learning, context = self.getTrajectory('refinement', participant_number, method, object_position, trial)
                    traj_wrt_base = self.methods[method]['model'][model]['object_position'][object_position]['trial'][trial]['refined_trajectory']
                    traj_wrt_base = self.dictToList(traj_wrt_base)
                    context = self.methods[method]['model'][model]['object_position'][object_position]['trial'][trial]['context']
                    
                    # print("context = " + str(context))                    

                    demo = self.parser.predicted_trajectory_to_prompTraj_message(traj_wrt_base, context)

                    # visualize prediction
                    visualize_trajectory = rospy.ServiceProxy('visualize_trajectory', VisualizeTrajectory)
                    visualization_msg = TrajectoryVisualization()
                    visualization_msg.pose_array = demo.poses

                    visualization_msg.r = random.random()
                    visualization_msg.g = random.random()
                    visualization_msg.b = random.random()

                    resp = visualize_trajectory(visualization_msg)
            
                except (rospy.ServiceException, rospy.ROSException) as e:
                    print("Service call failed: %s" %e)
                
                except KeyError:
                    continue
    
    def createDataAfterExperiment(self, participant_number, method, model):
        # voor elke trial moet ik de refined trajectory uitlezen samen met de bijbehorende context
        # dan update ik het model met deze context en trajectory

        self.setPath('/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/participant_' + str(participant_number)+ '/')

        self.loadData()
        path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/participant_' + str(participant_number) + '/after_experiment/' + str(self.experiment_variables.method_mapping_number_to_str[method]) + '/model_' + str(model) + '/'
        
        if not os.path.exists(path):
            os.makedirs(path)

        self.setPath(path)
        

        # adapt model using experiment data
        for object_position in self.object_positions:
            for trial in range(1, self.num_trials+1):

                # get trajectory wrt context
                try:
                    trajectory_for_learning, context = self.getTrajectory('refinement', participant_number, method, model, object_position, trial)
                    file_name = 'traj_for_learning_' + str(trial) + '.txt'

                    with open(self.debug_path + file_name, 'w+') as f:
                        f.write(str(trajectory_for_learning))

                    self.addToModel(trajectory_for_learning, context)
                # when there was no refinement (already successful prediction)
                # just continue
                except KeyError as e:
                    print(e)
                    continue

        self.num_evaluation_trials = 1

        for i in range(self.num_evaluation_trials+1):
            self.trials[i+1] = {
                'predicted_trajectory': copy.deepcopy(self.predicted_trajectory), 'context': []}
        
        for j in range(self.num_object_positions+1):
            self.data[j+1] = {
            'trial': copy.deepcopy(self.trials)}

        y0 = 0.2
        y_position_step_dict = {1: 0.0, 2: 0.1, 3: 2*0.1}
        object_positions = { 1: [0.8, y0 - y_position_step_dict[1], 0.7], 2: [0.8, y0 - y_position_step_dict[2], 0.7]}


        # loop over the object positions again and make predictions
        for position in object_positions:
            for trial in range(1, self.num_evaluation_trials+1):
                self.openGripper()

                self.current_object_position = position
                print("object position: " + str(position))
                print("trial: " + str(trial))

                # move ee to initial pose
                self.goToInitialPose()

                # wait until arm is not in the way of the object
                self.setDishwasherPosition(before_or_after='after')
                time.sleep(5)

                self.setObjectPosition('after')
                
                time.sleep(5)

                # get new context
                self.getContext()
                self.makePrediction()
                
                rospy.wait_for_service('visualize_trajectory', timeout=2.0)
                
                # visualize prediction
                visualize_trajectory = rospy.ServiceProxy('visualize_trajectory', VisualizeTrajectory)
                visualization_msg = TrajectoryVisualization()
                visualization_msg.pose_array = self.prediction.poses
                visualization_msg.r = 1.0
                visualization_msg.g = 0.0
                visualization_msg.b = 0.0

                resp = visualize_trajectory(visualization_msg)
                
                obstacle_hit, object_reached, object_kicked_over = self.executeTrajectory(self.prediction)

                print('obstacle_hit = ' + str(obstacle_hit))
                print('object_reached = ' + str(object_reached))
                print('object_kicked_over = ' + str(object_kicked_over))

                x = []
                y = []
                z = []
                qx = []
                qy = []
                qz = []
                qw = []
                t = []

                for i,data in enumerate(self.prediction.poses):
                    x.append(data.position.x)
                    y.append(data.position.y)
                    z.append(data.position.z)
                    qx.append(data.orientation.x)
                    qy.append(data.orientation.y)
                    qz.append(data.orientation.z)
                    qw.append(data.orientation.w)
                    t.append(self.prediction.times[i])

                self.data[position]['trial'][trial]['predicted_trajectory']['x'] = x
                self.data[position]['trial'][trial]['predicted_trajectory']['y'] = y
                self.data[position]['trial'][trial]['predicted_trajectory']['z'] = z
                self.data[position]['trial'][trial]['predicted_trajectory']['qx'] = qx
                self.data[position]['trial'][trial]['predicted_trajectory']['qy'] = qy
                self.data[position]['trial'][trial]['predicted_trajectory']['qz'] = qz
                self.data[position]['trial'][trial]['predicted_trajectory']['qw'] = qw
                self.data[position]['trial'][trial]['predicted_trajectory']['t'] = t
                self.data[position]['trial'][trial]['predicted_trajectory']['object_missed'] = not object_reached
                self.data[position]['trial'][trial]['predicted_trajectory']['obstacle_hit'] = obstacle_hit
                self.data[position]['trial'][trial]['predicted_trajectory']['object_kicked_over'] = object_kicked_over

                if not object_reached or obstacle_hit or object_kicked_over:
                    self.data[position]['trial'][trial]['predicted_trajectory']['success'] = False
                
                self.data[position]['trial'][trial]['context'] = [self.context.x, self.context.y, self.context.z]

                self.toTxt()
                clear_trajectories = rospy.ServiceProxy('clear_trajectories', ClearTrajectories)
                resp = clear_trajectories()

                time.sleep(1)
    def run(self):
        self.initializeHeadLiftJoint()

        participant_number = int(rospy.get_param('~participant_number'))
        method = int(rospy.get_param('~method')) # 3 = online + keyboard
        self.method = method
        visualize = bool(rospy.get_param('~visualize'))
        before_or_after_experiment = rospy.get_param('~before_or_after')
        
        for model in range(1, self.num_models+1):
            if before_or_after_experiment == 'before':
                self.createDataBeforeExperiment()        
            elif before_or_after_experiment == 'after' and visualize == True:
                self.visualizeExperimentData(participant_number, method, model)
                self.createDataAfterExperiment(participant_number, method, model)
            elif before_or_after_experiment == 'after' and visualize == False:
                self.createDataAfterExperiment(participant_number, method, model)
            
            try:
                rospy.wait_for_service('build_initial_model', timeout=2.0)
                build_init_model = rospy.ServiceProxy('build_initial_model', BuildInitialModel)
                build_init_model()
            except (rospy.ServiceException, rospy.ROSException) as e:
                print("Service call failed: %s" %e)
                
if __name__ == "__main__":
    data_creator = DataCreator()
    data_creator.run()
