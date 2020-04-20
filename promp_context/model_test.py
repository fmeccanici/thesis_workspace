import numpy as np
from promp_context.promp_context import ProMPContext
import random
import matplotlib.pyplot as plt

def get_cmap(n, name='hsv'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct 
    RGB color; the keyword argument name must be a standard mpl colormap name.'''
    return plt.cm.get_cmap(name, n)

num_traj = 5

cmap = get_cmap(num_traj)

sigma_noise=0.03
x = np.arange(0,1,0.01)

joint_names = ['output', 'context']

num_samples = len(x)
pmp = ProMPContext(joint_names, num_samples=num_samples)

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

goal = np.zeros(len(joint_names))


# goal[1] = 1.0
# goal[0] = float('nan')
# pmp.set_goal(goal)

# start = [0.0, 1.0]
# pmp.set_start(start)

context = 1.0
pred = pmp.generate_trajectory(context)

plt.figure()
plt.plot(x, [y[1] for y in pred], '-.', color='blue')    
plt.plot(x, [y[0] for y in pred], color='orange')    
plt.savefig('pred.png')


plt.figure()
pmp.plot(x, output_randomess=-1)
plt.savefig('mean_std.png')

plt.show()