#!/usr/bin/env python3
from promp_python.promp_python import *
from trajectoryParser.trajectoryParser import *
import numpy as np
import os

parser = trajectoryParser()


DIR = './data/'
traj_files = [name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))]

trajectories = []

# joints = ["cartesian_x", "cartesian_y", "cartesian_z"]
joints = ["joint_x", "joint_y", "joint_z", "object_x", "object_y", "object_z"]

i=0
for traj in traj_files:
    # print(i)
    trajectory = parser.openTrajectoryFile(traj, DIR)
    trajectory = np.array(trajectory)
    trajectories.append(trajectory)
    # i += 1
promp = ProMPContext(joints, len(trajectories[0]))

for traj in trajectories:
    print(traj[0])
#     promp.add_demonstration(traj)

# promp.plot_unconditioned_joints()
# plt.show()
