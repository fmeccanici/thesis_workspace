import numpy as np
from contextual_promp.contextual_promp import *

num_traj = 20
sigma_noise=0.03
x = np.arange(0,1,0.01)
A = np.array([.2, .2, .01, -.05])
X = np.vstack((np.sin(5*x), x**2, x, np.ones((1,len(x)))))
# X = np.vstack((x, x, x, np.ones((1,len(x)))))


# we use one joint and one context
# feel free to use more joints or contexts
joints = ["joint_x", "context"]

Y = np.zeros((num_traj, len(x)))
samples = []
for traj in range(0, num_traj):
    sample = np.dot(A + sigma_noise * np.random.randn(1,4), X)[0]

    sample = np.vstack((sample, np.tile(sigma_noise * np.random.randn() + 0.15, len(x))))

    samples.append(sample)

samples = np.array(samples)
num_points=len(x)
plt.figure(figsize=(6, 4))
plt.title("Added samples")
for i in range(0, num_traj):
    if i == 0:
        plt.plot(np.arange(0, len(samples[i, 0, :].T)) / num_points, samples[i, 0, :].T, color='blue', label='joint_x')
        plt.plot(np.arange(0, len(samples[i, 1, :].T)) / num_points, samples[i, 1, :].T, '-.', color='blue', label='context')
    elif i == 1:
        plt.plot(np.arange(0, len(samples[i, 0, :].T)) / num_points, samples[i, 0, :].T, color='green')
        plt.plot(np.arange(0, len(samples[i, 1, :].T)) / num_points, samples[i, 1, :].T, '-.', color='green')
    elif i == 2:
        plt.plot(np.arange(0, len(samples[i, 0, :].T)) / num_points, samples[i, 0, :].T, color='cyan')
        plt.plot(np.arange(0, len(samples[i, 1, :].T)) / num_points, samples[i, 1, :].T, '-.', color='cyan')
    elif i == 3:
        plt.plot(np.arange(0, len(samples[i, 0, :].T)) / num_points, samples[i, 0, :].T, color='magenta')
        plt.plot(np.arange(0, len(samples[i, 1, :].T)) / num_points, samples[i, 1, :].T, '-.', color='magenta')

    else:
        plt.plot(np.arange(0, len(samples[i, 0, :].T)) / num_points, samples[i, 0, :].T, color='black')
        plt.plot(np.arange(0, len(samples[i, 1, :].T)) / num_points, samples[i, 1, :].T, '-.', color='black')
plt.xlabel('t [s]')
plt.ylabel('joint position [rad]')
plt.grid()
plt.legend()

pmp = ProMPContext(joints, num_points=num_points)

plt.figure(figsize=(6, 4))
# add demonstrations to ProMP
for demo_id in range(0, num_traj):
    pmp.add_demonstration(samples[demo_id, :, :].T)

# plot mean and standard deviation of demonstrations
pmp.plot_unconditioned_joints()

# condition on context=0.1

goal = np.zeros(2)
print(samples[0][-1][-1])
goal[1] = samples[0][-1][-1]
pmp.clear_viapoints()
pmp.set_goal(goal, sigma=1e-6)
pmp.plot_conditioned_joints()
plt.grid()
# plt.show()

# alternatively
# goal = np.zeros(2)
# goal[1] = 0.1
# pmp.clear_viapoints()
# pmp.set_goal(goal, sigma=1e-6)
generated_trajectory = pmp.generate_trajectory(sigma_noise)
plt.figure(figsize=(6, 4))
plt.title("Prediction")
for joint_id, joint_name in enumerate(joints):
    if joint_name == "context":
        print(len(np.ones(len(x))*goal[1]))
        plt.plot(np.ones(len(x))*goal[1], '-.', label=joint_name)
    else:
        print(len(generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0]))
        plt.plot(generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0], label=joint_name)
plt.legend()
plt.grid()
plt.show()
