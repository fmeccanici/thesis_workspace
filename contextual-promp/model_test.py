import numpy as np
from contextual_promp.contextual_promp import *
import random
import matplotlib.pyplot as plt

num_traj = 5

sigma_noise=0.03
x = np.arange(0,1,0.01)

joints = ['output', 'input']
num_points = len(x)
pmp = ProMPContext(joints, num_points=num_points)

samples = []


# make samples without random numbers
y1 = list(map(lambda x: x, x))
y2 = list(map(lambda x: x + 1, x))
y3 = list(map(lambda x: x + 2, x))
y4 = list(map(lambda x: x + 1, x))
y5 = list(map(lambda x: x + 2, x))

context1 = list(np.ones(len(x)) * 0)
context2 = list(np.ones(len(x)) * 1)
context3 = list(np.ones(len(x)) * 2)
context4 = list(np.ones(len(x)) * 1)
context5 = list(np.ones(len(x)) * 2)

y = []
context = []

y.append(y1)
y.append(y2)
y.append(y3)
# y.append(y4)
# y.append(y5)

context.append(context1)
context.append(context2)
context.append(context3)
# context.append(context4)
# context.append(context5)

plt.figure()
# create samples
for i in range(num_traj):
    y = list(map(lambda x: x + i, x))
    # context = list(np.ones(len(x)) * np.random.randint(1,4))
    context = list(np.ones(len(x)) * i)
    
    # generate random colors
    r = round(random.random(), 1)
    b = round(random.random(), 1)
    g = round(random.random(), 1)
    color = (r, g, b)
    

    # plt.plot(x, y[i], c=color, label='input ' + str(i+1))
    # plt.plot(x, context[i], '-.', c=color, label='context ' + str(i+1))

    plt.plot(x, y, c=color, label='output ' + str(i+1))
    plt.plot(x, context, '-.', c=color, label='input ' + str(i+1))


    plt.title("Demonstrations")
    # sample = list(map(lambda x, y: [x, y], y[i], context[i]))

    sample = list(map(lambda x, y: [x, y], y, context))

    samples.append(sample)


plt.legend()
plt.grid()

# add samples to promp model
for sample in samples:
    pmp.add_demonstration(np.asarray(sample))

pmp.plot_unconditioned_joints()

goal = np.zeros(len(joints))
goal[1] = 1.0

pmp.clear_viapoints()
pmp.set_goal(goal, sigma=1e-6)
# start = [1.0, 1.0]

# pmp.set_start(start)

pmp.plot_conditioned_joints()

generated_trajectory = pmp.generate_trajectory(sigma_noise)
plt.figure(figsize=(6, 4))
plt.title("Prediction")
for joint_id, joint_name in enumerate(joints):
    if joint_name == "input":
        plt.plot(x, np.ones(len(x))*goal[1], '-.', label=joint_name)
    else:
        plt.plot(x, generated_trajectory[joint_id*num_points:(joint_id+1)*num_points, 0], label=joint_name)
plt.legend()
plt.grid()
plt.show()

print(pmp.get_mean(0))