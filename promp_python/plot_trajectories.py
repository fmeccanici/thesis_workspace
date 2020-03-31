from promp_python.promp_python import *
from trajectory_parser.trajectory_parser import *

import os, os.path
import numpy as np

import matplotlib.pyplot as plt

parser = trajectoryParser()

def resample_trajectory(traj1, traj2):

    n1 = len(traj1)
    n2 = len(traj2)
    dt1 = parser.getTimeInterval(parser._normalize(traj1))
    dt2 = parser.getTimeInterval(parser._normalize(traj2))

    traj1 = parser._normalize(traj1)
    traj2 = parser._normalize(traj2)



    dt_new = n2 / (n1 / dt1)

    traj2_pos = parser.getCartesianPositions(traj2)
    traj2_time = parser._getTimeVector(traj2)

    # we want to downsample to the smallest trajectory, which is traj1
    l = len(traj1)
    xvals2 = np.linspace(dt_new, l*dt_new, l)

    traj2_time = np.asarray(parser._secsNsecsToFloat(traj2_time))
    traj2_pos_x = np.asarray(parser.getXpositions(traj2_pos)).reshape(len(traj2_pos), 1)
    traj2_pos_y = np.asarray(parser.getYpositions(traj2_pos)).reshape(len(traj2_pos), 1)
    traj2_pos_z = np.asarray(parser.getZpositions(traj2_pos)).reshape(len(traj2_pos), 1)

    yinterp_traj2_x = interp1d((traj2_time), np.transpose(traj2_pos_x), axis=1, fill_value="extrapolate")
    yinterp_traj2_y = interp1d((traj2_time), np.transpose(traj2_pos_y), axis=1, fill_value="extrapolate")
    yinterp_traj2_z = interp1d((traj2_time), np.transpose(traj2_pos_z), axis=1, fill_value="extrapolate")
    
    y_traj2_new_x = yinterp_traj2_x(xvals2)
    y_traj2_new_y = yinterp_traj2_y(xvals2)
    y_traj2_new_z = yinterp_traj2_z(xvals2)
    
    qstart = traj2[0][3:7]
    qend = traj2[-1][3:7]
    
    object_info = traj2[0][7:14]

    traj2 = []

    # print(object_info)

    for i,q in enumerate(parser.interpolateQuaternions(qstart, qend, l, False)):
        traj2.append([y_traj2_new_x[0][i], y_traj2_new_y[0][i], y_traj2_new_z[0][i]] + [q[1], q[2], q[3], q[0]] + object_info + [xvals2[i]])
    
    return traj2


dt = 0.01

DIR = '../trajectory_teaching/data/with_object_wrt_optical/'
traj_files = [name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))]
num_traj = len(traj_files)

trajectories = []
trajectories_lengths = []

for traj in traj_files:
    trajectory = parser.openTrajectoryFile(traj, DIR)
    trajectory = parser._normalize(trajectory)
    trajectory = parser.downsample(trajectory, dt)
    print(len(trajectory))

    # plt.plot(parser.getCartesianPositions(trajectory))

    trajectories.append(trajectory)
    trajectories_lengths.append(len(trajectory))

traj_min_length = trajectories[np.argmin(trajectories_lengths)]
print(traj_min_length)
del trajectories[np.argmin(trajectories_lengths)]

trajectories_resampled = []

for traj in trajectories:
    traj_res = resample_trajectory(traj_min_length, traj)
    plt.plot(parser.getCartesianPositions(traj_res))
    
    x = np.ones((len(traj_res), 1)) * parser.get_context(traj_res[0])[0]
    y = np.ones((len(traj_res), 1)) * parser.get_context(traj_res[0])[1]
    z = np.ones((len(traj_res), 1)) * parser.get_context(traj_res[0])[2]
    # plt.plot(x)
    # plt.plot(y)
    # plt.plot(z)
    trajectories_resampled.append(traj_res)

plt.show()