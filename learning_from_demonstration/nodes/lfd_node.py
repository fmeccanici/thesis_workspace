#!/usr/bin/env python3.5

# import ros related packages
import rospy, rospkg
from trajectory_visualizer.msg import TrajectoryVisualization
from geometry_msgs.msg import PoseStamped, Pose
from aruco_msgs.msg import MarkerArray
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from learning_from_demonstration.srv import (AddDemonstration, AddDemonstrationResponse, MakePrediction,
                                            MakePredictionResponse, SetObject, SetObjectResponse,
                                            GetContext, GetContextResponse, GoToPose, GoToPoseResponse,
                                            ExecuteTrajectory, ExecuteTrajectoryResponse)

from promp_context_ros.msg import prompTraj
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

# import my own classes
from learning_from_demonstration_python.learning_from_demonstration import learningFromDemonstration
from trajectory_visualizer_python.trajectory_visualizer_python import trajectoryVisualizer
from learning_from_demonstration_python.trajectory_parser import trajectoryParser
from learning_from_demonstration_python.trajectory_resampler import trajectoryResampler

# import other python classes
from scipy.interpolate import interp1d
import numpy as np
import time
import matplotlib

# use agg to avoid gui from over his nek gaan 
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class lfdNode():
    def __init__(self):
        # #initialize ros related
        rospy.init_node('lfd_node')
        self._rospack = rospkg.RosPack()

        self._get_parameters()

        self._traj_vis_pub = rospy.Publisher('trajectory_visualizer/trajectory', TrajectoryVisualization, queue_size=10)
        self._end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)
        self._end_effector_pose_sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self._end_effector_pose_callback)
        self._marker_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self._marker_detection_callback)
        
        # ros services
        self._add_demo_service = rospy.Service('add_demonstration', AddDemonstration, self._add_demonstration)
        self._predict_service = rospy.Service('make_prediction', MakePrediction, self._make_prediction)
        self._set_object_service = rospy.Service('set_object', SetObject, self._set_object_position)
        self._get_context_service = rospy.Service('get_context', GetContext, self._get_context_marker)
        self._go_to_pose_service = rospy.Service('go_to_pose', GoToPose, self._go_to_pose)
        self._execute_trajectory_service = rospy.Service('execute_trajectory', ExecuteTrajectory, self._execute_trajectory)

        # initialize other classes
        self.lfd = learningFromDemonstration()
        self.visualizer = trajectoryVisualizer()
        self.parser = trajectoryParser()
        self.resampler = trajectoryResampler()

        # initialize class variables
        self.marker_pose = Pose()
        self.add_demo_success = Bool()


    def _end_effector_pose_callback(self,data):
        self.current_slave_pose = data.pose

    def _marker_detection_callback(self, data):
        # rospy.loginfo("marker pose = " + str(self.object_marker_pose.position))
        for marker in data.markers:
            if marker.id == 582:
                # flip x and y as in training data
                self.marker_pose.position.x = marker.pose.pose.position.y
                self.marker_pose.position.y = marker.pose.pose.position.x
                self.marker_pose.position.z = marker.pose.pose.position.z

                self.marker_pose.orientation.x = marker.pose.pose.orientation.x
                self.marker_pose.orientation.y = marker.pose.pose.orientation.y
                self.marker_pose.orientation.z = marker.pose.pose.orientation.z
                self.marker_pose.orientation.w = marker.pose.pose.orientation.w

            else: continue
    
    def _get_parameters(self):
        raw_folder = rospy.get_param('~raw_folder')
        self.raw_path = self._rospack.get_path('learning_from_demonstration') + "/data/raw/" + str(raw_folder) + "/"
        print("Set raw trajectory path to " + str(self.raw_path))

    def executeTrajectory(self, traj, dt):
        rospy.loginfo("Executing trajectory...")
        slave_goal = PoseStamped()
        for datapoint in traj:
            slave_goal.pose.position.x = datapoint[0]
            slave_goal.pose.position.y = datapoint[1]
            slave_goal.pose.position.z = datapoint[2]

            slave_goal.pose.orientation.x = datapoint[3]
            slave_goal.pose.orientation.y = datapoint[4]
            slave_goal.pose.orientation.z = datapoint[5]
            slave_goal.pose.orientation.w = datapoint[6]

            slave_goal.header.seq = 0
            slave_goal.header.frame_id = "/base_footprint"

            slave_goal.header.stamp = (rospy.Time.now())
            self._end_effector_goal_pub.publish(slave_goal)
            
            time.sleep(dt)

    def context_to_msg(self, context):
        point = Point()
        point.x = context[0]
        point.y = context[1]
        point.z = context[2]

        return point

    def get_relative_context(self):
        
        object_wrt_base = [self.marker_pose.position.x, self.marker_pose.position.y, self.marker_pose.position.z]
        ee_pos_wrt_base = [self.current_slave_pose.position.x, self.current_slave_pose.position.y, self.current_slave_pose.position.z]
        
        object_wrt_ee = list(np.subtract(object_wrt_base, ee_pos_wrt_base))
        
        # times 10 to get the model to work
        x = round(object_wrt_ee[0]*10, 2)
        y = round(object_wrt_ee[1]*10, 2)
        z = round(object_wrt_ee[2]*10, 2)

        return [x, y, z]

    def get_marker_wrt_base(self):
        x = round(self.marker_pose.position.x, 2)
        y = round(self.marker_pose.position.y, 2)
        z = round(self.marker_pose.position.z, 2)

        return [x, y, z]

    def get_context(self):
        # times 10 to get the model to work
        x = round(self.marker_pose.position.x*10, 2)
        y = round(self.marker_pose.position.y*10, 2)
        z = round(self.marker_pose.position.z*10, 2)

        return [x, y, z]
    
    def get_current_slave_position(self):
        x = self.current_slave_pose.position.x
        y = self.current_slave_pose.position.y
        z = self.current_slave_pose.position.z

        return [x, y, z]

    def go_to_pose(self, pose):
        rospy.wait_for_message('/end_effector_pose', PoseStamped)

        T = 2
        x = [self.current_slave_pose.position.x, pose.position.x]
        y = [self.current_slave_pose.position.y, pose.position.y]
        z = [self.current_slave_pose.position.z, pose.position.z]

        t = [rospy.Time.now(), rospy.Time.now() + rospy.Duration(T)]

        t = list(self.parser.secs_nsecs_to_float_vector(self.parser.durationVector2secsNsecsVector(t)))
        fx = interp1d(t, x, fill_value="extrapolate")
        fy = interp1d(t, y, fill_value="extrapolate")
        fz = interp1d(t, z, fill_value="extrapolate")

        dt = 0.1
        tnew = np.arange(t[0],t[-1],dt)
        xnew = fx(tnew)
        ynew = fy(tnew)
        znew = fz(tnew)

        for i in range(len(ynew)):
            slave_goal = PoseStamped()

            slave_goal.pose.position.x = xnew[i]
            slave_goal.pose.position.y = ynew[i]
            slave_goal.pose.position.z = znew[i]

            slave_goal.pose.orientation.x = pose.orientation.x
            slave_goal.pose.orientation.y = pose.orientation.y
            slave_goal.pose.orientation.z = pose.orientation.z
            slave_goal.pose.orientation.w = pose.orientation.w

            slave_goal.header.seq = 0
            slave_goal.header.frame_id = "/base_footprint"

            slave_goal.header.stamp = (rospy.Time.now())
            self._end_effector_goal_pub.publish(slave_goal)
            
            time.sleep(dt)
                
    def goToInitialPose(self):
        rospy.loginfo("Moving to initial pose")
        rospy.wait_for_message('/end_effector_pose', PoseStamped)

        T = 2
        x = [self.current_slave_pose.position.x, 0.401946359213]
        y = [self.current_slave_pose.position.y, -0.0230769199229]
        z = [self.current_slave_pose.position.z, 0.840896642238]

        # x = [self.current_slave_pose.position.x, 0.403399335619]
        # y = [self.current_slave_pose.position.y, -0.430007534239]
        # z = [self.current_slave_pose.position.z, 1.16269467394]
        
        # x = [self.current_slave_pose.position.x, 0.353543514402]
        # y = [self.current_slave_pose.position.y, 0.435045131507]
        # z = [self.current_slave_pose.position.z, 0.760080619348]

        t = [rospy.Time.now(), rospy.Time.now() + rospy.Duration(T)]

        t = list(self.parser.secs_nsecs_to_float_vector(self.parser.durationVector2secsNsecsVector(t)))
        fx = interp1d(t, x, fill_value="extrapolate")
        fy = interp1d(t, y, fill_value="extrapolate")
        fz = interp1d(t, z, fill_value="extrapolate")

        dt = 0.1
        tnew = np.arange(t[0],t[-1],dt)
        xnew = fx(tnew)
        ynew = fy(tnew)
        znew = fz(tnew)

        for i in range(len(ynew)):
            slave_goal = PoseStamped()

            slave_goal.pose.position.x = xnew[i]
            slave_goal.pose.position.y = ynew[i]
            slave_goal.pose.position.z = znew[i]

            slave_goal.pose.orientation.x = 0.980837824843
            slave_goal.pose.orientation.y = -0.00365989846539
            slave_goal.pose.orientation.z = -0.194791016723
            slave_goal.pose.orientation.w = -0.194791016723

            slave_goal.header.seq = 0
            slave_goal.header.frame_id = "/base_footprint"

            slave_goal.header.stamp = (rospy.Time.now())
            self._end_effector_goal_pub.publish(slave_goal)
            
            time.sleep(dt)

    def initialize_lfd_model(self):
        self.base_frame = 'base_footprint'
        self.lfd.load_trajectories_from_folder(self.raw_path)

        desired_datapoints = 100
        self.lfd.prepare_for_learning(desired_datapoints)
        
        plt.figure()
        for traj in self.lfd.trajectories_for_learning:
            plt.plot([x[0:3] for x in traj])
            plt.plot([x[7:10] for x in traj])
            plt.xlabel("datapoints [-]")
            plt.ylabel("position [m]")
            plt.title("Trajectories used as input")
        plt.show()
        self.lfd.build_initial_promp_model()

    def visualize_trajectory(self, traj, r, g, b):
        for i in range(50):
            # print(self.visualizer.trajToVisMsg(traj, r=r, g=g, b=b, frame_id=self.base_frame))
            self._traj_vis_pub.publish(self.visualizer.trajToVisMsg(traj, r=r, g=g, b=b, frame_id=self.base_frame))

    def clear_trajectories_rviz(self):
        empty_traj = np.zeros((1, 7))
        for i in range(10):
            self._traj_vis_pub.publish(self.visualizer.trajToVisMsg(list(empty_traj), r=0, g=0, b=0))
    
    def set_aruco_position(self, x=0.7, y=-0.43, z=1):
        state_msg = ModelState()
        state_msg.model_name = 'aruco_cube'
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = z
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def _execute_trajectory(self, req):
        traj, dt = self.lfd.parser.promptraj_msg_to_execution_format(req.trajectory)

        ndesired = 75

        # resample trajectory so that it can successfully be executed
        if len(traj) < ndesired:
            traj, dt = self.resampler.interpolate_learned_keypoints(traj, ndesired)

        self.executeTrajectory(traj, dt)

    def _go_to_pose(self, req):
        rospy.loginfo("Executing trajectory using service...")
        pose = req.pose

        self.go_to_pose(pose)

        response = GoToPoseResponse()
        suc = Bool()
        suc.data = True
        response.success = suc

        return response

    def _set_object_position(self, req):
        print("Setting Aruco position using service...")
        x = req.position.x
        y = req.position.y
        z = req.position.z

        self.set_aruco_position(x, y, z)

        response = SetObjectResponse()
        suc = Bool()
        suc.data = True
        response.success = suc

        return response        
    
    def _get_context_marker(self, req):
        rospy.loginfo("Get context service...")

        response = GetContextResponse()
        context = self.get_relative_context()
        context_msg = self.context_to_msg(context)
        response.context = context_msg

        return response

    def _make_prediction(self, req):
        rospy.loginfo("Making prediction using service...")
        goal = [req.context.x, req.context.y, req.context.z]

        object_wrt_base = self.get_marker_wrt_base()
        prediction = self.lfd.generalize(goal)
        relative_prediction = self.parser.traj_wrt_base(prediction, object_wrt_base)

        # traj_pred_message = self.predicted_trajectory_to_prompTraj_message(prediction, goal)
        traj_pred_message = self.lfd.parser.predicted_trajectory_to_prompTraj_message(relative_prediction, goal)

        response = MakePredictionResponse()
        response.prediction = traj_pred_message

        return response

    def _add_demonstration(self, req):
        print("Adding demonstration using service...")

        trajectory, context = self.lfd.parser.prompTrajMessage_to_demonstration_format(req.demo)

        self.lfd.add_trajectory_to_promp_model(trajectory, context)
        self.add_demo_success.data = True

        response = AddDemonstrationResponse()
        response.success = self.add_demo_success

        return response

    # for debugging
    def predict(self):
        object_wrt_base = self.get_context()
        print("goal = " + str(object_wrt_base))
        ee_wrt_base = self.get_current_slave_position()
        object_wrt_ee = self.lfd.parser.object_wrt_ee(ee_wrt_base, object_wrt_base)

        prediction = self.lfd.generalize(object_wrt_ee)

        rospy.loginfo(object_wrt_base)
        object_wrt_base1 = [round(0*10,2), round(0.78*10, 2), round(0.68*10,2)]

        prediction = self.lfd.generalize(object_wrt_base1)

        plt.figure()
        plt.plot([x[0] for x in prediction], label='x')
        plt.plot([x[1] for x in prediction], label='y')
        plt.plot([x[2] for x in prediction], label='z')
        plt.legend()
        
        # plt.plot([x[7:10] for x in prediction])

        plt.xlabel("datapoints [-]")
        plt.ylabel("position [m]")
        plt.title("Predicted relative trajectory")

        # trajectory_wrt_base = self.lfd.trajectory_wrt_base(prediction, object_wrt_base)
        trajectory_wrt_base = prediction
        
        plt.figure()
        plt.plot([x[0] for x in trajectory_wrt_base], label='x')
        plt.plot([x[1] for x in trajectory_wrt_base], label='y')
        plt.plot([x[2] for x in trajectory_wrt_base], label='z')

        plt.xlabel("datapoints [-]")
        plt.ylabel("position [m]")
        plt.title("Final executed predicted trajectory")
        plt.grid()
        plt.legend()

        plt.show()
        return trajectory_wrt_base

    def run(self):
        pass
        # self.goToInitialPose()
        # x = float(input("x: "))

        # # x = 0.94
        # y = -0.0231
        # self.set_aruco_position(x, y)

        # if input("Is the object placed at the desired location? 1/0") == "0":
        #     x = float(input("x: "))
        # else:
        #     # self.clear_trajectories_rviz()

        #     print("Making prediction...")
        #     # self.lfd.promp_model.plot_conditioned_joints()

        #     traj_pred = self.predict()

        #     plt.plot([ x[0] for x in traj_pred])
        #     plt.show()
        #     # print(traj_pred)
        #     n = 100
        #     # traj_pred_resampled, dt = self.resampler.interpolate_learned_keypoints(traj_pred, n)
        #     self.visualize_trajectory(traj_pred, 1, 0, 0)
        #     # self.visualize_trajectory(traj_pred_resampled, 0, 0, 1)

        #     # dt = 0.1
        #     # self.executeTrajectory(traj_pred_resampled, dt)
        #     time.sleep(5)
        

            
if __name__ == "__main__":
    node = lfdNode()
    node.initialize_lfd_model()
    node.goToInitialPose()
    
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        node.run()
        r.sleep()
    # except Exception as e: rospy.loginfo(e)
