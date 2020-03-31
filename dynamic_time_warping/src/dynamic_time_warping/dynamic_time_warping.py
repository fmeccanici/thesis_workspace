#!/usr/bin/env python

from fastdtw import fastdtw
import numpy as np

from trajectory_parser.trajectory_parser import *

class DTW():
    def set_trajectories(self, demonstrations):
        self.trajectories = demonstrations

    # input structure list in list [[x_1, y_1, z_1]]
    @classmethod
    def apply_dtw(self, x, y):
        x = np.asarray(x).transpose()
        y = np.asarray(y).transpose()
        
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


if __name__ == "__main__":
    parser = trajectoryParser()
    dtw = DTW()

    input_path = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_teaching/data/with_object_wrt_optical/'
    dt = 0.01

    trajectories, trajectories_lengths = parser.load_trajectories_from_folder_and_downsample(input_path, dt)


    traj_min_length = trajectories[np.argmin(trajectories_lengths)]
    del trajectories[np.argmin(trajectories_lengths)]

    output_path = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_parser/data/'
    # traj_res = parser.resample_and_store_trajectories(trajectories, traj_min_length, output_path)
    traj_res = parser.resample_trajectories(trajectories, traj_min_length)

    traj_for_learning = []
    for traj in traj_res:
        traj_for_learning.append(parser.get_relevant_learning_data(traj))


    ix_for_dtw = parser.get_trajectories_ix_for_dtw(traj_for_learning)
    ix_for_dtw_copy = ix_for_dtw

while len(ix_for_dtw_copy) > 0:

    to_dtw = []
    # print(ix_for_dtw_copy)
    to_dtw.append(ix_for_dtw_copy[0][0])
    to_dtw.append(ix_for_dtw_copy[0][1])
    del ix_for_dtw_copy[0]

    for i in range(len(ix_for_dtw_copy)):
        # print(to_dtw)
        # print(ix_for_dtw_copy[i][0])
        # print(len(ix_for_dtw_copy))
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
        print(to_dtw[i])
        traj_to_dtw.append(traj_for_learning[to_dtw[i]])
    
    reference = dtw.determine_reference(traj_to_dtw)
    y_aligned = []
    for i in range(len(to_dtw)):
        x,y = dtw.apply_dtw(traj_for_learning[reference], traj_for_learning[to_dtw[i]])
        y_aligned.append(y)
    
