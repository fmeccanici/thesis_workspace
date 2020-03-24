
from promp_python.promp_python import *
from trajectoryParser.trajectoryParser import *

import os, os.path
import numpy as np

parser = trajectoryParser()

# resample trajectory to the one with the smallest lentgh --> downsample the other
# traj1 is smaller than traj2
def resample_trajectory(traj1, traj2):

    n1 = len(traj1)
    n2 = len(traj2)
    dt1 = parser.getTimeInterval(parser._normalize(traj1))
    dt2 = parser.getTimeInterval(parser._normalize(traj2))

    traj1 = parser._normalize(traj1)
    traj2 = parser._normalize(traj2)

    # traj = [traj1, traj2]

    n = [n1, n2]
    dt = [dt1, dt2]

    # i_nmin = np.argmin(n)
    # i_nmax = np.argmax(n)

    # n[i_nmin] / dt[i_nmin] = n[i_nmax] / dt[i_nmax]

    dt_new = n2 / (n1 / dt1)

    # traj1_pos = parser.getCartesianPositions(traj1)
    # traj1_time = parser._getTimeVector(traj1)

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


    for i,q in enumerate(parser.interpolateQuaternions(qstart, qend, l, False)):
        traj2.append([y_traj2_new_x[0][i], y_traj2_new_y[0][i], y_traj2_new_z[0][i]] + object_info + [q[1], q[2], q[3], q[0], xvals2[i]])
    
    return traj2

def get_relevant_data(traj):

    traj_new = []

    for data in traj:

        # add only the cartesian path, object location and time
        traj_new.append(data[0:3] + data[7:10] + [data[-1]])
        
    return traj_new

dt = 0.1

DIR = '../trajectory_teaching/data/with_object/'
traj_files = [name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))]
num_traj = len(traj_files)

trajectories = []
trajectories_lengths = []

for traj in traj_files:
    trajectory = parser.openTrajectoryFile(traj, DIR)
    trajectories.append(trajectory)
    trajectories_lengths.append(len(trajectory))

traj_min_length = trajectories[np.argmin(trajectories_lengths)]
del trajectories[np.argmin(trajectories_lengths)]

trajectories_resampled = []

for traj in trajectories:
    trajectories_resampled.append(resample_trajectory(traj_min_length, traj))

joints = ["cartesian_x", "cartesian_y", "cartesian_z", "object_x", "object_y", "object_z", "time"]

promp = ProMPContext(joints, len(traj_min_length))

for i in range(0, len(trajectories_resampled)):
    traj = get_relevant_data(trajectories_resampled[i])
    
    promp.add_demonstration(traj)
# print(traj)
# promp.plot_unconditioned_joints()