#!/usr/bin/env python

from trajectory_parser.trajectory_parser import *
# import trajectory_parser
from dynamic_time_warping.dynamic_time_warping import *
import numpy as np

class prepareForLearning():
    def __init__(self, input_path, output_path, time_step):
        self.input_path = input_path
        self.output_path = output_path
        self.parser = trajectoryParser()
        self.dtw = DTW()
        self.dt = time_step

    def ee_wrt_object(self, ee_wrt_base, object_wrt_base):
        return np.subtract(ee_wrt_base, object_wrt_base)

    def object_wrt_ee(self, ee_wrt_base, object_wrt_base):
        return np.subtract(object_wrt_base, ee_wrt_base)

    def parse_to_relative_traj(self, traj):

        # initialize information needed for relative calculations
        object_wrt_base = traj[0][8:]
        ee_wrt_base_0 = traj[0][0:3]
        object_wrt_ee_0 = list(self.object_wrt_ee(ee_wrt_base_0, object_wrt_base))

        parsed_traj = []
        # calculate relative vectors
        for data in traj:
            ee_wrt_base = data[0:3]

            ee_wrt_object = list(self.ee_wrt_object(ee_wrt_base, object_wrt_base))
            ee_ori =  list(data[3:7])
            dt = [data[7]]
            parsed_traj.append(ee_wrt_object + ee_ori + dt + object_wrt_ee_0)
        return parsed_traj

    def prepare_for_learning(self):
        # downsample
        trajectories, trajectories_lengths = self.parser.load_trajectories_from_folder_and_downsample(input_path, self.dt)
        # print(trajectories[-1][0])

        ## resample
        # traj_min_length = trajectories[np.argmin(trajectories_lengths)]
        
        # del trajectories[np.argmin(trajectories_lengths)]
        
        # traj_res = self.parser.resample_trajectories(trajectories, traj_min_length)
        traj_res = []
        for traj in trajectories:
            traj_res.append(self.parser.interpolate_raw_trajectory(traj, 20))
        # traj_res = self.parser.resample_trajectories(trajectories, traj_min_length)

        traj_for_learning = []
        for traj in traj_res:
            # print(len(traj))
            print(traj[0])
            # check if in right format
            
            # if not self.parser.tIsFloat(traj):
            #     t_secs_nsecs = self.parser._getTimeVector(traj)
            #     t_float = self.parser._secsNsecsToFloat(t_secs_nsecs)
            #     traj_wo_t = self.parser._removeTmatrix(traj)
            #     traj = self.parser._addTmatrix(traj_wo_t, t_float)

            traj_for_learning.append(self.parser.get_relevant_learning_data(traj))
        
        # apply dtw
        traj_aligned_for_learning = self.dtw.align_necessary_trajectories(traj_for_learning)
        
        # print(traj_aligned_for_learning[-1][0])

        parsed_trajs = []
        # parse positions to get relative vectors
        for traj in traj_aligned_for_learning:
            parsed_traj = []
            # initialize information needed for relative calculations
            object_wrt_base = traj[0][8:]
            ee_wrt_base_0 = traj[0][0:3]
            object_wrt_ee_0 = list(self.object_wrt_ee(ee_wrt_base_0, object_wrt_base))

            # calculate relative vectors
            for data in traj:
                ee_wrt_base = data[0:3]

                ee_wrt_object = list(self.ee_wrt_object(ee_wrt_base, object_wrt_base))
                ee_ori =  list(data[3:7])
                dt = [data[7]]
                parsed_traj.append(ee_wrt_object + ee_ori + dt + object_wrt_ee_0)

            parsed_trajs.append(parsed_traj)

        # print(parsed_trajs[-1][0])
        return parsed_trajs
    
    def store_trajectories(self, traj, output_path):
        for i in range(len(traj)):
            traj_file = open(output_path + "resampled_" + str(i) + ".txt", "w+")
            traj_file.write(str(traj[i]))
            traj_file.close()

    def prepare_and_store(self):
        prep_trajs = self.prepare_for_learning()
        self.store_trajectories(prep_trajs, self.output_path)

if __name__ == "__main__":
    input_path = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_teaching/data/both_wrt_base/'
    output_path = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_refinement/data/resampled/'
    dt = 0.1

    prep = prepareForLearning(input_path, output_path, dt)
    prep.prepare_and_store()
