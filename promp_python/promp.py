#!/usr/bin/env python2.7

from promp_python.promp_python import *
from trajectory_parser.trajectory_parser import *
import numpy as np
import os
import rospy

parser = trajectoryParser()


DIR = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_refinement/data/resampled/'
# DIR = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_teaching/data/'
traj_files = [name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))]

trajectories = []

# joints = ["cartesian_x", "cartesian_y", "cartesian_z"]
# joints = ["joint_x", "joint_y", "joint_z", "context"]
# joints = ["joint_x", "joint_y", "joint_z", "dt", "object_x", "object_y", "object_z"]

joints = ["joint_x", "joint_y", "joint_z", "qx", "qy", "qz", "qw",  "dt", "object_x", "object_y", "object_z"]


# joints = ["joint_x", "context"]

for traj in traj_files:
    # print([t[8:] for t in traj])
    # plt.plot([t[8:] for t in traj])
    trajectory = parser.openTrajectoryFile(traj, DIR)
    trajectory = np.array(trajectory)
    trajectories.append(trajectory)

num_points = len(trajectories[0])

# "num_points = num_points" --> very important otherwise it wont work
# first had "num_points" as argument which causes the trajectories to be weird

promp = ProMPContext(joints, num_points=num_points)

for traj in trajectories:
    # print(traj[0])
    promp.add_demonstration(traj)

promp.plot_unconditioned_joints()

# plt.show()

goal = np.zeros(len(joints))
# goal[4:] = [0.38, 0.81, 0.68]
goal[8:] = [0.12, 0.0, 0.74]

# promp.clear_viapoints()
# promp.set_goal(goal, sigma=1e-6)
# promp.plot_conditioned_joints()
# plt.show()


# print(goal)

## alternative
sigma_noise=0.03

promp.clear_viapoints()
promp.set_goal(goal, sigma=1e-6)
generated_trajectory = promp.generate_trajectory(sigma_noise)

"""joint_id = 0
pred_traj_x = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

joint_id = 1
pred_traj_y = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

joint_id = 2
pred_traj_z = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

joint_id = 3
pred_traj_qx = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

joint_id = 4
pred_traj_qy = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

joint_id = 5
pred_traj_qz = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

joint_id = 6
pred_traj_qw = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

joint_id = 7
pred_traj_dt = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

"""
joint_id = 0
pred_traj_x = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

joint_id = 1
pred_traj_y = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

joint_id = 2
pred_traj_z = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])

joint_id = 3
pred_traj_dt = (generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0])


pred_traj = []
dt = pred_traj_dt[0]
t = 0
for i in range(len(pred_traj_x)):
    pred_traj.append([pred_traj_x[i], pred_traj_y[i], pred_traj_z[i], t])
    t += dt
# for i in range(len(pred_traj_x)):
#     pred_traj.append([pred_traj_x[i], pred_traj_y[i], pred_traj_z[i], pred_traj_qx[i], pred_traj_qy[i], pred_traj_qz[i], pred_traj_qw[i], t])
#     t += dt

print('pred_traj' + str(pred_traj))

plt.figure()
for joint_id, joint_name in enumerate(joints):
    # print(joint_id)
    plt.plot(generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0], label=joint_name)
    plt.xlabel("datapoint [-]")

plt.legend()
plt.show()
