#!usr/bin/env python
import rospy, copy, time, ast, roslaunch, rospkg
from execution_failure_detection.srv import GetExecutionFailure
from learning_from_demonstration.srv import ExecuteTrajectory, GoToPose, GetContext, MakePrediction
from gazebo_msgs.msg import ModelState 
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
from trajectory_visualizer.msg import TrajectoryVisualization
from trajectory_visualizer.srv import VisualizeTrajectory, ClearTrajectories
from learning_from_demonstration_python.trajectory_parser import trajectoryParser

from learning_from_demonstration.srv import WelfordUpdate
from execution_failure_detection.srv import GetExecutionFailure, SetExpectedObjectPosition
import numpy as np
import random

from trajectory_visualizer.msg import TrajectoryVisualization
from trajectory_visualizer.srv import VisualizeTrajectory, ClearTrajectories

from std_msgs.msg import Float32

class DataCreator(object):
    def __init__(self):
        self.num_object_positions = 6
        self.num_trials = 5
        self.num_methods = 4
        self.methods = {}
        self.parser = trajectoryParser()
        self.nodes = {}
        self._rospack = rospkg.RosPack()

        # self.object_positions = { 1: [0.8, 0.25, 0.7], 2: [0.8, 0.18, 0.7], 3: [0.8, 0.11, 0.7], 4: [0.8, 0.04, 0.7], 5: [0.8, -0.03, 0.7], 6: [0.8, -0.1, 0.7]}
        self.object_positions = { 1: [0.8, 0.25, 0.7]}

        self.trials = {}
        self.data = {}

        self.predicted_trajectory = {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [], 'object_missed': False, 'obstacle_hit': False,'object_kicked_over': False,  'success': True}
        self.refined_trajectory = {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [], 'object_missed': False, 'obstacle_hit': False, 'object_kicked_over': False, 'success': True}
        self.debug_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/data/debug/'

        rospy.init_node('data_creator')

    def executeTrajectory(self, traj):
        rospy.wait_for_service('execute_trajectory', timeout=2.0)
        execute_trajectory = rospy.ServiceProxy('execute_trajectory', ExecuteTrajectory)
        self.T_desired = 10
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

    def setObjectPosition(self):
        try:
            object_position = ModelState()
            object_position.model_name = 'aruco_cube'

            step = 0.07
            x = 0.8
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

    def createDataBeforeExperiment(self):

        self.setPath('/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/before_experiment/dishwasher2/')
        
        for i in range(self.num_trials):
            self.trials[i+1] = {
                'predicted_trajectory': copy.deepcopy(self.predicted_trajectory), 'context': []}
        
        for j in range(self.num_object_positions):
            self.data[j+1] = {
            'trial': copy.deepcopy(self.trials)}

        for position in self.object_positions:
            for trial in range(1, self.num_trials+1):
                self.current_object_position = position
                print("object position: " + str(position))
                print("trial: " + str(trial))

                # move ee to initial pose
                self.goToInitialPose()

                # wait until arm is not in the way of the object
                time.sleep(2)
                
                self.setObjectPosition()
                
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

    def getTrajectory(self, which, participant_number, method, object_position, trial):

        if which == 'refinement':
            trajectory = 'refined_trajectory'
        elif which == 'prediction':
            trajectory == 'predicted_trajectory'

        traj_wrt_base = copy.deepcopy(self.methods[method]['object_position'][object_position]['trial'][trial][trajectory])
        traj_wrt_base = self.dictToList(traj_wrt_base)
        context = copy.deepcopy(self.methods[method]['object_position'][object_position]['trial'][trial]['context'])
        
        r_object_wrt_ee = [context[0] / 10, context[1] / 10, context[2] / 10] 
        
        print('obj wrt ee: ' + str(r_object_wrt_ee))

        # calculate object wrt base
        r_object_wrt_base = np.asarray(r_object_wrt_ee) + np.asarray([traj_wrt_base[0][0], traj_wrt_base[0][1], traj_wrt_base[0][2]])

        print('obj wrt base: ' + str(r_object_wrt_base))


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

        return traj_wrt_object, context

    def addToModel(self, trajectory, context):
        # 2 was too much, later trajectories had too little influence
        amount = 2

        try:
            rospy.wait_for_service('welford_update', timeout=2.0)

            welford_update = rospy.ServiceProxy('welford_update', WelfordUpdate)
            trajectory_wrt_object_msg = self.parser.predicted_trajectory_to_prompTraj_message(trajectory, context)
            
            for i in range(amount):
                resp = welford_update(trajectory_wrt_object_msg)
            
            rospy.loginfo("Added " + str(amount) + " trajectories to model using Welford")


        except (AttributeError, ValueError) as e:
                rospy.loginfo("Problem with adding trajectory: %s" %e)

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)  

    def visualizeExperimentData(self, participant_number, method):
        self.setPath('/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/participant_' + str(participant_number)+ '/')
        self.loadData()
        
        # adapt model using experiment data
        for object_position in self.object_positions:
            for trial in range(1, self.num_trials+1):
                
                # get trajectory wrt context
                try:
                    # trajectory_for_learning, context = self.getTrajectory('refinement', participant_number, method, object_position, trial)
                    traj_wrt_base = self.methods[method]['object_position'][object_position]['trial'][trial]['refined_trajectory']
                    traj_wrt_base = self.dictToList(traj_wrt_base)
                    context = self.methods[method]['object_position'][object_position]['trial'][trial]['context']
                    
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
    
    def createDataAfterExperiment(self, participant_number, method):
        # voor elke trial moet ik de refined trajectory uitlezen samen met de bijbehorende context
        # dan update ik het model met deze context en trajectory

        self.setPath('/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/participant_' + str(participant_number)+ '/')

        self.loadData()

        self.setPath('/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/participant_' + str(participant_number) + '/after_experiment/')

        # adapt model using experiment data
        for object_position in self.object_positions:
            for trial in range(1, self.num_trials+1):
                
                # get trajectory wrt context
                try:
                    trajectory_for_learning, context = self.getTrajectory('refinement', participant_number, method, object_position, trial)
                    file_name = 'traj_for_learning_' + str(trial) + '.txt'

                    with open(self.debug_path + file_name, 'w+') as f:
                        f.write(str(trajectory_for_learning))

                    self.addToModel(trajectory_for_learning, context)

                # when there was no refinement (already successful prediction)
                # just continue
                except KeyError:
                    continue

        # for i in range(self.num_trials):
        #     self.trials[i+1] = {
        #         'predicted_trajectory': copy.deepcopy(self.predicted_trajectory), 'context': []}
        
        # for j in range(self.num_object_positions):
        #     self.data[j+1] = {
        #     'trial': copy.deepcopy(self.trials)}

        # # loop over the object positions again and make predictions
        # for position in self.object_positions:
        #     for trial in range(1, self.num_trials+1):
        #         self.current_object_position = position
        #         print("object position: " + str(position))
        #         print("trial: " + str(trial))

        #         # move ee to initial pose
        #         self.goToInitialPose()

        #         # wait until arm is not in the way of the object
        #         time.sleep(2)
                
        #         self.setObjectPosition()
                
        #         time.sleep(2)

        #         # get new context
        #         self.getContext()
        #         self.makePrediction()
                
        #         rospy.wait_for_service('visualize_trajectory', timeout=2.0)
                
        #         # visualize prediction
        #         visualize_trajectory = rospy.ServiceProxy('visualize_trajectory', VisualizeTrajectory)
        #         visualization_msg = TrajectoryVisualization()
        #         visualization_msg.pose_array = self.prediction.poses
        #         visualization_msg.r = 1.0
        #         visualization_msg.g = 0.0
        #         visualization_msg.b = 0.0

        #         resp = visualize_trajectory(visualization_msg)
                
        #         obstacle_hit, object_reached, object_kicked_over = self.executeTrajectory(self.prediction)

        #         x = []
        #         y = []
        #         z = []
        #         qx = []
        #         qy = []
        #         qz = []
        #         qw = []
        #         t = []

        #         for i,data in enumerate(self.prediction.poses):
        #             x.append(data.position.x)
        #             y.append(data.position.y)
        #             z.append(data.position.z)
        #             qx.append(data.orientation.x)
        #             qy.append(data.orientation.y)
        #             qz.append(data.orientation.z)
        #             qw.append(data.orientation.w)
        #             t.append(self.prediction.times[i])

        #         self.data[position]['trial'][trial]['predicted_trajectory']['x'] = x
        #         self.data[position]['trial'][trial]['predicted_trajectory']['y'] = y
        #         self.data[position]['trial'][trial]['predicted_trajectory']['z'] = z
        #         self.data[position]['trial'][trial]['predicted_trajectory']['qx'] = qx
        #         self.data[position]['trial'][trial]['predicted_trajectory']['qy'] = qy
        #         self.data[position]['trial'][trial]['predicted_trajectory']['qz'] = qz
        #         self.data[position]['trial'][trial]['predicted_trajectory']['qw'] = qw
        #         self.data[position]['trial'][trial]['predicted_trajectory']['t'] = t
        #         self.data[position]['trial'][trial]['predicted_trajectory']['object_missed'] = not object_reached
        #         self.data[position]['trial'][trial]['predicted_trajectory']['obstacle_hit'] = obstacle_hit
        #         self.data[position]['trial'][trial]['predicted_trajectory']['object_kicked_over'] = object_kicked_over

        #         if not object_reached or obstacle_hit or object_kicked_over:
        #             self.data[position]['trial'][trial]['predicted_trajectory']['success'] = False
                
        #         self.data[position]['trial'][trial]['context'] = [self.context.x, self.context.y, self.context.z]

        #         self.toTxt()
        #         clear_trajectories = rospy.ServiceProxy('clear_trajectories', ClearTrajectories)
        #         resp = clear_trajectories()

        #         time.sleep(1)

if __name__ == "__main__":
    participant_number = 89
    method = 3 # online + keyboard

    data_creator = DataCreator()
    data_creator.visualizeExperimentData(participant_number, method)
    data_creator.createDataAfterExperiment(participant_number, method)
    # data_creator.createDataBeforeExperiment()