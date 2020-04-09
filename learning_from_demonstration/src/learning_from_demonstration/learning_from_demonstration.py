#!/usr/bin/env python3.5

# import my own classes
from promp_python import ProMPContext
from dynamic_time_warping import DTW
from trajectory_parser import trajectoryParser
from trajectory_resampler import trajectoryResampler

# import other external classes
import ast, os
import numpy as np

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

    def load_trajectories_from_folder(self, path):
        traj_files = [name for name in os.listdir(path) if os.path.isfile(os.path.join(path, name))]
        
        for traj_file in traj_files:
            self.load_trajectory_from_folder(path, traj_file)
        
    def load_trajectory_from_folder(self, path, traj_file):
        with open(path+traj_file, "r") as traj:
            raw_trajectory = ast.literal_eval(traj.read())    
            self.add_raw_trajectory(raw_trajectory)

    def add_raw_trajectory(self, raw_traj):
        
        # check if in correct format
        if not self.is_correct_raw_format(raw_traj):
            raw_traj = self.convert_raw_to_correct_format(raw_traj)
            # print(raw_traj)
        else:
            pass

        self.raw_trajectories.append(raw_traj)

    def is_correct_raw_format(self, raw_traj):
        if len(raw_traj[0]) == 15:
            return True
        else:
            return False
    
    def parse_relevant_learning_data(self, traj):
        traj_relevant_data = []
        
        # T doesnt work properly --> chose dt as output
        for data in traj:
            dt = data[-1]
            ee_pose = data[0:7]
            object_positions = data[7:10]

            traj_relevant_data.append(ee_pose + object_positions + [dt] )

        return traj_relevant_data

    def convert_raw_to_correct_format(self, raw_traj):
        if self.is_correct_raw_format(raw_traj):
            print("Raw trajectory already in correct format")
        else:
            try:
                # check if time format is incorrect (secs, nsecs) instead of float
                if isinstance(raw_traj[0][14], int):
                    # time format is (secs, nsecs)
                    print("Raw trajectory contains secs/nsecs values")
                    print("Converting to float...")
                    return self.parser.convert_raw_secs_nsecs_to_float(raw_traj)
                else:
                    print("No secs/nsecs value detected")
            except IndexError:
                print("Raw trajectory has incorrect length, check if it contains essential paramaters")
    
    def prepare_for_learning(self, desired_datapoints):
        print("Preparing raw trajectories for learning...")
        
        # normalize time
        print("Normalizing trajectories wrt time...")
        for traj in self.raw_trajectories:
            traj = self.parser.normalize_trajectory_time_float(traj)

        # resample trajectories
        print("Resample trajectories to all have " + str(desired_datapoints) + " datapoints...")
        resampled_trajectories = []
        for traj in self.raw_trajectories:
            resampled_trajectories.append(self.resampler.interpolate_raw_trajectory(traj, desired_datapoints))

        # get relevant learning data
        print("Extracting relevant learning data: [ee_x, ee_y, ee_z, ee_qx, ee_qy, ee_qz, obj_x, obj_y, obj_z, dt]...")
        relevant_data_trajectories = []
        for traj in resampled_trajectories:
            relevant_traj = self.parse_relevant_learning_data(traj)
            relevant_data_trajectories.append(relevant_traj)

        # apply dynamic time warping
        print("Applying DTW...")
        traj_aligned_for_learning = self.dtw.align_necessary_trajectories(relevant_data_trajectories)

        # convert trajectory to relative trajectory wrt ee
        # and change object pose wrt base to wrt ee
        print("Converting to relative trajectory...")
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
        print('Adding trajectories to ProMP model...')

        for i,traj in enumerate(self.trajectories_for_learning):
            self.add_trajectory_to_promp_model(traj)
            print("Added trajectory " + str(i+1))

    def add_trajectory_to_promp_model(self, traj):
        self.promp_model.add_demonstration(np.asarray(traj))

    def get_raw_trajectories(self):
        return self.raw_trajectories

    def get_trajectories_for_learning(self):
        return self.trajectories_for_learning

if __name__ == "__main__":
    lfd = learningFromDemonstration()
    raw_path = "/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/data/raw/both_wrt_base4/"

    lfd.load_trajectories_from_folder(raw_path)

    desired_datapoints = 100
    lfd.prepare_for_learning(desired_datapoints)
    lfd.build_initial_promp_model()
