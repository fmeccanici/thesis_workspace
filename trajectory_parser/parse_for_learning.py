#!/usr/bin/env python

from trajectory_parser.trajectory_parser import *
import numpy as np
import matplotlib.pyplot as plt

parser = trajectoryParser()

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
    plt.plot(parser.getCartesianPositions(traj))
    traj_for_learning.append(parser.get_relevant_learning_data(traj))
    plt.xlabel('datapoint [-]')
    plt.ylabel('position [m]')
    plt.title('Cartesian end effector positions after resampling')
plt.show()
ix_for_dtw = parser.get_trajectories_ix_for_dtw(traj_for_learning)

