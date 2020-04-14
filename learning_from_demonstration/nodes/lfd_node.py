#!/usr/bin/env python3.5

# import ros related packages
import rospy, rospkg
from trajectory_visualizer.msg import TrajectoryVisualization
from geometry_msgs.msg import PoseStamped, Pose
from aruco_msgs.msg import MarkerArray
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

# import my own classes
from learning_from_demonstration.learning_from_demonstration import learningFromDemonstration
from trajectory_visualizer_python.trajectory_visualizer_python import trajectoryVisualizer
from learning_from_demonstration.trajectory_parser import trajectoryParser
from learning_from_demonstration.trajectory_resampler import trajectoryResampler

# import other python classes
from scipy.interpolate import interp1d
import numpy as np
import time
import matplotlib.pyplot as plt

class lfdNode():
    def __init__(self):
        # initialize ros related
        rospy.init_node('lfd_node')
        self._rospack = rospkg.RosPack()

        self._get_parameters()

        self._traj_vis_pub = rospy.Publisher('trajectory_visualizer/trajectory', TrajectoryVisualization, queue_size=10)
        self._end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)
        self._end_effector_pose_sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self._end_effector_pose_callback)
        self._marker_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self._marker_detection_callback)

        # initialize other classes
        self.lfd = learningFromDemonstration()
        self.visualizer = trajectoryVisualizer()
        self.parser = trajectoryParser()
        self.resampler = trajectoryResampler()

        # initialize class variables
        self.marker_pose = Pose()

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

    def get_current_marker_position(self):
        x = self.marker_pose.position.x
        y = self.marker_pose.position.y
        z = self.marker_pose.position.z

        return [x, y, z]
    
    def get_current_slave_position(self):
        x = self.current_slave_pose.position.x
        y = self.current_slave_pose.position.y
        z = self.current_slave_pose.position.z

        return [x, y, z]

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
        print(t)
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
            slave_goal.pose.orientation.w = 0.000475714270521

            slave_goal.header.seq = 0
            slave_goal.header.frame_id = "/base_footprint"

            slave_goal.header.stamp = (rospy.Time.now())
            self._end_effector_goal_pub.publish(slave_goal)
            
            time.sleep(dt)

    def initialize_lfd_model(self):
        self.base_frame = 'base_footprint'
        self.lfd.load_trajectories_from_folder(self.raw_path)

        # for traj in self.lfd.raw_trajectories:
        #     print(traj[0][7:10])
        # time.sleep(30)
        desired_datapoints = 10
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
    
    def predict(self):
        object_wrt_base = self.get_current_marker_position()
        print("goal = " + str(object_wrt_base))
        ee_wrt_base = self.get_current_slave_position()
        object_wrt_ee = self.lfd.parser.object_wrt_ee(ee_wrt_base, object_wrt_base)

        # prediction = self.lfd.generalize(object_wrt_ee)

        prediction = self.lfd.generalize(object_wrt_base)


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
        plt.legend()

        plt.show()
        return trajectory_wrt_base

    def run(self):
        self.goToInitialPose()
        x = 0.8
        y = -0.0231
        self.set_aruco_position(x, y)

        if input("Is the object placed at the desired location? 1/0"):
            self.clear_trajectories_rviz()

            print("Making prediction...")
            traj_pred = self.predict()

            n = 100
            traj_pred_resampled, dt = self.resampler.interpolate_learned_keypoints(traj_pred, n)
            
            self.visualize_trajectory(traj_pred, 1, 0, 0)
            self.visualize_trajectory(traj_pred_resampled, 0, 0, 1)

            # dt = 0.1
            self.executeTrajectory(traj_pred_resampled, dt)
            time.sleep(5)
        

            
if __name__ == "__main__":
    node = lfdNode()
    node.initialize_lfd_model()
    

    while not rospy.is_shutdown():
        node.run()
    # except Exception as e: rospy.loginfo(e)
