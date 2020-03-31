#!/usr/bin/env python

from fastdtw import fastdtw
import numpy as np

from trajectory_parser.trajectory_parser import *
import matplotlib.pyplot as plt
import os, os.path

class DTW():
    def set_trajectories(self, demonstrations):
        self.trajectories = demonstrations

    # input structure list in list [[x_1, y_1, z_1]]
    @classmethod
    def apply_dtw(self, x, y):

        # check if we need to transpose the vectors
        # length of first array should be 11 (posx, posy, posz, etc...)
        # if this is too large then it should be transposed
        if len(x[0]) > 30:
            x = np.asarray(x).transpose()
            y = np.asarray(y).transpose()
        else:
            x = np.asarray(x)
            y = np.asarray(y)


        distance, path = fastdtw(x, y)

        x_aligned = []
        y_aligned = []

        for i in range(len(path)):
            x_aligned.append(x[path[i][0]])
            y_aligned.append(y[path[i][1]])

        return x_aligned, y_aligned

    # from Kyrarini et al.
    def determine_reference(self, demonstrations):
        similarity = np.zeros(len(demonstrations))
        for i in range(len(demonstrations)):
            for j in range(len(demonstrations)):
                distance, path = fastdtw(demonstrations[i], demonstrations[j])
                similarity[i] += distance
        
        return np.argmin(similarity)
   
    def align_necessary_trajectories(self, input_path, output_path, dt):
        traj_files = [name for name in os.listdir(input_path) if os.path.isfile(os.path.join(input_path, name))]
        num_traj = len(traj_files)
        trajectories, trajectories_lengths = parser.load_trajectories_from_folder_and_downsample(input_path, dt)
        traj_min_length = trajectories[np.argmin(trajectories_lengths)]
        
        del trajectories[np.argmin(trajectories_lengths)]
        traj_res = parser.resample_trajectories(trajectories, traj_min_length)
        traj_for_learning = []
        for traj in traj_res:
            traj_for_learning.append(parser.get_relevant_learning_data(traj))


        ix_for_dtw = parser.get_trajectories_ix_for_dtw(traj_for_learning)
        ix_for_dtw_copy = ix_for_dtw

        # counter = 1 
        while len(ix_for_dtw_copy) > 0:

            to_dtw = []
            to_dtw.append(ix_for_dtw_copy[0][0])
            to_dtw.append(ix_for_dtw_copy[0][1])
            del ix_for_dtw_copy[0]

            for i in range(len(ix_for_dtw_copy)):

                if ix_for_dtw_copy[i][0] in to_dtw and ix_for_dtw_copy[i][1] not in to_dtw:
                    to_dtw.append(traj_for_learning[ix_for_dtw_copy[i][1]])
                    ix_for_dtw_copy = ix_for_dtw

                    del ix_for_dtw[i]
                elif ix_for_dtw_copy[i][1] in to_dtw and ix_for_dtw_copy[i][0] not in to_dtw:
                    to_dtw.append(traj_for_learning[ix_for_dtw_copy[i][0]])
                    ix_for_dtw_copy = ix_for_dtw

                    del ix_for_dtw[i]

                elif ix_for_dtw_copy[i][1] not in to_dtw and ix_for_dtw_copy[i][0] not in to_dtw:
                    pass
                elif ix_for_dtw_copy[i][1] in to_dtw and ix_for_dtw_copy[i][0] in to_dtw:
                    pass

            traj_to_dtw = []
            for i in range(len(to_dtw)):
                traj_to_dtw.append(traj_for_learning[to_dtw[i]])
            
            reference = dtw.determine_reference(traj_to_dtw)
            reference = to_dtw[reference]

            for i in range(len(to_dtw)):

                if to_dtw[i] != reference:

                    x,y = dtw.apply_dtw(traj_for_learning[reference], traj_for_learning[to_dtw[i]])
                    y = parser.arrays_in_list_to_list_in_list(y)
                    x = parser.arrays_in_list_to_list_in_list(x)

                    # if counter == 1:
                    #     # plt.plot(parser.getCartesianPositions(traj_for_learning[reference]))
                    #     # plt.plot(parser.getCartesianPositions(traj_for_learning[to_dtw[i]]))
                    #     plt.plot(parser.getCartesianPositions(x))
                    #     plt.plot(parser.getCartesianPositions(y))
                    #     counter += 1
                    #     print(parser.getCartesianPositions(x)[0])
                    del traj_for_learning[to_dtw[i]]
                    del traj_for_learning[reference]

                    traj_for_learning.append(y)
                    traj_for_learning.append(x)

        return traj_for_learning
if __name__ == "__main__":
    parser = trajectoryParser()
    dtw = DTW()

    input_path = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_teaching/data/with_object_wrt_optical/'
    output_path = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_parser/data/'
    dt = 0.01

    traj_aligned_for_learning = dtw.align_necessary_trajectories(input_path, output_path, dt)

 

