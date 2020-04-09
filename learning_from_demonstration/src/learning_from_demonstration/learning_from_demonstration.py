#!/usr/bin/env python2.7

# import my own classes
from promp_python import ProMPContext
from dynamic_time_warping import DTW
from trajectory_parser import trajectoryParser
from trajectory_resampling import trajectoryResampler

# import other external classes
import ast

class learningFromDemonstration():
    def __init__(self):
        # raw trajectory layout: 
        # [ee_x, ee_y, ee_z, ee_qx, ee_qy, ee_qz, ee_qw,
        #  pos_x, pos_y, pos_z, pos_qx, pos_qy, pos_qz, pos_qw,
        #  time_as_float]
        
        self.raw_trajectories = []
        self.trajectories_for_learning = []

        self.parser = trajectoryParser()
        self.resampler = trajectoryResampler()
        self.dtw = DTW()

    def load_trajectories_form_folder(self, path):
        traj_files = [name for name in os.listdir(path) if os.path.isfile(os.path.join(path, name))]

    def load_trajectory_from_folder(self, path, traj_file):
        with open(path+traj_file, "r") as traj:
            raw_trajectory = ast.literal_eval(traj.read())    
            self.add_raw_trajectory(raw_trajectory)

    def add_raw_trajectory(self, raw_traj):
        self.raw_trajectories.append(raw_traj)

    def is_correct_raw_format(self, raw_traj):
        if len(raw_traj) == 15:
            return True
        else:
            return False
    
    def get_relevant_learning_data(self, traj):
        traj_relevant_data = []

        # T doesnt work properly --> chose dt as output
        dt = self.parser.get_dt(traj)
        ee_pose = data[0:7]
        object_positions = data[7:10]

        for data in traj:
            traj_relevant_data.append(ee_pose + object_positions + [dt] )

        return traj_relevant_data

    def convert_raw_to_correct_format(self, raw_traj):
        if self.is_correct_raw_format(raw_traj):
            print("Raw trajectory already in correct format")
        else:
            try:
                # check if time format is incorrect (secs, nsecs) instead of float
                if isinstance(raw_traj[14], int):
                    # time format is (secs, nsecs)
                    print("Raw trajectory contains secs/nsecs values")
                    print("Converting to float...")
                    return self.parser.convert_raw_secs_nsecs_to_float(raw_traj)
                else:
                    print("No secs/nsecs value detected")

    def prepare_for_learning(self, desired_datapoints):
        
        # resample trajectories
        resampled_trajectories = []
        for traj in self.raw_trajectories():
            resampled_trajectories.append(self.resampler.interpolate_raw_trajectory(traj, desired_datapoints))

        # get relevant learning data
        relevant_data_trajectories = []
        for traj in resampled_trajectories:
            relevant_traj = self.get_relevant_learning_data(traj)
            relevant_data_trajectories.append(relevant_traj)

        # apply dynamic time warping
        traj_aligned_for_learning = self.dtw.align_necessary_trajectories(relevant_data_trajectories)

        # convert trajectory to relative trajectory wrt ee
        # and change object pose wrt base to wrt ee
        for traj in traj_aligned_for_learning:
            traj_wrt_object = self.parser.get_trajectory_wrt_object(traj)
            self.trajectories_for_learning.append(traj_wrt_object)
    
    def build_initial_promp_model(self):
        # in promp package, input and output are all called joints
        # if name is different, then it won't plot them
        joints = ["joint_x", "joint_y","joint_z", "qx", "qy", "qz", "qw", "object_x", "object_y", "object_z", "dt" ]
        
        # how to select the number of points??
        # if other trajectories are added to the model
        # this will probably be wrongly compared to the existing
        # model, since the trajs will be squeezed/stretched
        num_points = len(self.trajectories_for_learning[0])
        
        # default value
        num_basis = 20

        # default value
        sigma = 0.1

        self.promp_model = ProMPContext(joints, num_points=num_points, num_basis=num_basis, sigma=sigma)
        for traj in self.trajectories_for_learning:
            print('Adding trajectories to model...')
            self.promp_model.add_demonstration(traj)
    
    def add_trajectory_to_promp_model(self, traj):
        self.promp_model.add_demonstration(traj)
        
if __name__ == "__main__":
    lfd = learningFromDemonstration()
