#!/usr/bin/env python3.5

# import ros related packages
import rospy 
from trajectory_visualizer.msg import TrajectoryVisualization

# import my own classes
from learning_from_demonstration.learning_from_demonstration import learningFromDemonstration
from trajectory_visualizer.trajectory_visualizer import trajectoryVisualizer

class lfdNode():
    def __init__(self):
        # initialize ros related
        rospy.init_node('lfd_node')
        self.traj_vis_pub_ = rospy.Publisher('trajectory_visualizer/trajectory', TrajectoryVisualization, queue_size=10)

        # initialize other classes
        self.lfd = learningFromDemonstration()
        self.visualizer = trajectoryVisualizer()

    def run(self):
        raw_path = "/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/data/raw/both_wrt_base4/"
        base_frame = 'base_footprint'
        self.lfd.load_trajectories_from_folder(raw_path)

        desired_datapoints = 100
        self.lfd.prepare_for_learning(desired_datapoints)
        self.lfd.build_initial_promp_model()

        object_wrt_base = [0.24, 0.81, 0.68]
        ee_wrt_base = [0.41, -0.42, 1.14]
        object_wrt_ee = self.lfd.parser.object_wrt_ee(ee_wrt_base, object_wrt_base)

        prediction = self.lfd.generalize(object_wrt_ee)
        trajectory_wrt_base = self.lfd.trajectory_wrt_base(prediction, object_wrt_base)

        for i in range(50):
            self.traj_vis_pub_.publish(self.visualizer.trajToVisMsg(trajectory_wrt_base, r=0, g=0, b=1, frame_id=base_frame))

if __name__ == "__main__":
    node = lfdNode()
    try:
        while not rospy.is_shutdown():
            node.run()
    except Exception as e: rospy.loginfo(e)
