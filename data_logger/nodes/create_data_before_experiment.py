#!usr/bin/env python
import rospy, copy, time
from execution_failure_detection.srv import GetExecutionFailure
from learning_from_demonstration.srv import ExecuteTrajectory, GoToPose, GetContext, MakePrediction
from gazebo_msgs.msg import ModelState 
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
from trajectory_visualizer.msg import TrajectoryVisualization
from trajectory_visualizer.srv import VisualizeTrajectory, ClearTrajectories

class CreateDataBeforeExperiment(object):
    def __init__(self):
        self.num_object_positions = 6
        self.num_trials = 5
        self.object_positions = {1:[0.82, 0.3, 0.9], 2:[0.82, 0.0, 0.9], 3:[0.65,0.29,0.9],4:[0.65,0.0,0.9]}
        self.trials = {}
        self.data = {}
        
        self.path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/before_experiment/'
        self.predicted_trajectory = {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [], 'object_missed': False, 'obstacle_hit': False, 'success': True}
        
        for i in range(self.num_trials):
            self.trials[i+1] = {
                'predicted_trajectory': copy.deepcopy(self.predicted_trajectory), 'context': []}
        
        for j in range(self.num_object_positions):
            self.data[j+1] = {
            'trial': copy.deepcopy(self.trials)}

        rospy.init_node('create_data_before_experiment')

    def executeTrajectory(self, traj):
        rospy.wait_for_service('execute_trajectory', timeout=2.0)
        execute_trajectory = rospy.ServiceProxy('execute_trajectory', ExecuteTrajectory)
        self.T_desired = 10.0
        resp = execute_trajectory(traj, self.T_desired)

        return resp.obstacle_hit.data, resp.object_reached.data

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
    
    def setObjectPosition(self, x, y, z):
        object_position = ModelState()
        object_position.model_name = 'aruco_cube'

        object_position.pose.position.x = x
        object_position.pose.position.y = y
        object_position.pose.position.z = z
        object_position.pose.orientation.x = 0
        object_position.pose.orientation.y = 0
        object_position.pose.orientation.z = 0
        object_position.pose.orientation.w = 1
            
        rospy.wait_for_service('/gazebo/set_model_state')
        set_object = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_object(object_position)
        return resp.success

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

    def run(self):
        for position in self.object_positions:
            for trial in range(1, self.num_trials+1):
                x,y,z = self.object_positions[position][0], self.object_positions[position][1], self.object_positions[position][2]

                print("object position: " + str(self.object_positions[position]))
                print("trial: " + str(trial))

                # move ee to initial pose
                self.goToInitialPose()

                # wait until arm is not in the way of the object
                time.sleep(2)
                
                self.setObjectPosition(x,y,z)
                
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
                
                obstacle_hit, object_reached = self.executeTrajectory(self.prediction)

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
                
                if not object_reached or obstacle_hit:
                    self.data[position]['trial'][trial]['predicted_trajectory']['success'] = False
                
                self.data[position]['trial'][trial]['context'] = [self.context.x, self.context.y, self.context.z]


                self.toTxt()
                clear_trajectories = rospy.ServiceProxy('clear_trajectories', ClearTrajectories)
                resp = clear_trajectories()

                time.sleep(1)

if __name__ == "__main__":
    data_creator = CreateDataBeforeExperiment()
    data_creator.run()