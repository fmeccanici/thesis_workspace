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

output_names = ['output1']
context_names = ['context1', 'context2', 'context3']
# context_names = ['context1']

num_samples = len(x)
promps = [ProMPContext(output_name, context_names, num_samples=num_samples,num_basis=20) for output_name in output_names]


samples1 = []
samples2 = []

plt.figure()
# create samples
for i in range(num_traj):
    # y = list(map(lambda x: [x + i, x + i, x + i], x))
    # y = list(map(lambda x: [x + i], x))
    y1 = list(map(lambda x: [np.sin(10*x) + i], x))
    y2 = list(map(lambda x: [np.sin(10*x) + 2*i], x))
    y3 = list(map(lambda x: [np.sin(10*x) + 3*i], x))

    context1 = [i, i, i]
    # context2 = 3*i

    # context = [i, i, i]
    # context = [ [ x[0], x[1], x[2] ] for x in context ]
    # print(context)
    # generate random colors
    r = round(random.random(), 1)
    b = round(random.random(), 1)
    g = round(random.random(), 1)
    color = (r, g, b)
    

    # plt.plot(x, y[i], c=color, label='input ' + str(i+1))
    # plt.plot(x, context[i], '-.', c=color, label='context ' + str(i+1))

    plt.plot(x, y1, c=color, label='output ' + str(1))
    # plt.plot(x, y2, c=color, label='output ' + str(2))

    for i in range(len(context1)):

        plt.plot(x, np.ones(len(x)) * context1[i], '-.', c=color, label='input ' + str(i))
    # plt.plot(x, np.ones(len(x)) * context1, '-.', c=color, label='input ' + str(i+1))


    plt.title("Demonstrations")
    # sample = list(map(lambda x, y: [x, y], y[i], context[i]))

    # print(context)
    # sample = list(map(lambda x: [x, i, i, i], y))
    sample1 = (y1, context1)
    # sample2 = (y2, context2)

    samples1.append(sample1)
    # samples2.append(sample2)

# plt.legend()
plt.grid()
plt.show()
plt.savefig('model.png')

# add samples to promp model
for sample in samples1:
    
    # pmp.add_demonstration(np.asarray(sample))
    promps[0].add_demonstration(sample)

# for sample in samples2:
#     promps[1].add_demonstration(sample)

# goal = np.zeros(len(joint_names))


# goal[1] = 1.0
# goal[0] = float('nan')
# pmp.set_goal(goal)

# start = [0.0, 1.0]
# pmp.set_start(start)

context = [1.0, 1.0, 1.0]
# context = 1.0

plt.figure()
# for promp in promps:
pred = promps[0].generate_trajectory(context)
plt.plot(x, [y for y in pred], color='blue', label='prediction')    
plt.plot(x, np.ones(pred.shape) * context[0], '-',  color='orange', label='context1')   
plt.plot(x, np.ones(pred.shape) * context[1], '-.',  color='black', label='context2')   
plt.plot(x, np.ones(pred.shape) * context[2], '--',  color='green', label='context3')   

# plt.figure()
# plt.plot(x, [y for y in pred], color='blue')    
# plt.plot(x, np.ones(pred.shape) * context, '-.',  color='orange')    
plt.title("Prediction")
plt.grid()
plt.legend()
plt.savefig('pred.png')


# pmp.plot_mean_variance()
# plt.savefig('mean_variance.png')

# plt.figure()
# pmp.plot(x, output_randomess=-1)
# plt.savefig('mean_std.png')

# plt.show()