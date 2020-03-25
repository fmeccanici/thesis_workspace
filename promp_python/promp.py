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

for traj in traj_files:
    trajectory = parser.openTrajectoryFile(traj, DIR)
    trajectory = np.array(trajectory)
    trajectories.append(trajectory)

num_points = len(trajectories[0])

promp = ProMPContext(joints, num_points)

sigma_noise=0.03
for traj in trajectories:
    # print(traj[0])
    promp.add_demonstration(traj)

promp.plot_unconditioned_joints()


goal = np.zeros(len(joints))
goal[0:3] = [0.6225886721571389, 0.1337757571231239, 0.8079378816574874]
goal[3:6] = [-0.2162276804447174, 0.04950021952390671, 0.7742748260498047]
# goal[3] = -0.3

promp.clear_viapoints()
promp.set_goal(goal, sigma=1e-6)
promp.plot_conditioned_joints()
plt.show()


# print(goal)

# promp.clear_viapoints()
# promp.set_goal(goal, sigma=1e-6)
# generated_trajectory = promp.generate_trajectory(sigma_noise)
# plt.figure()
# for joint_id, joint_name in enumerate(joints):
#     print(joint_id)
#     plt.plot(generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0], label=joint_name)
# plt.legend()
# plt.show()
