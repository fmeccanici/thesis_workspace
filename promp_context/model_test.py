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
context_names = ['context1']

num_samples = len(x)
pmp = ProMPContext(output_names, context_names, num_samples=num_samples,num_basis=20)


samples = []

plt.figure()
# create samples
for i in range(num_traj):
    # y = list(map(lambda x: [x + i, x + i, x + i], x))
    y = list(map(lambda x: [x + i], x))

    context = i
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

    # plt.plot(x, y, c=color, label='output ' + str(i+1))
    # plt.plot(x, context, '-.', c=color, label='input ' + str(i+1))


    plt.title("Demonstrations")
    # sample = list(map(lambda x, y: [x, y], y[i], context[i]))

    # print(context)
    # sample = list(map(lambda x: [x, i, i, i], y))
    sample = (y, context)
    samples.append(sample)

plt.legend()
plt.grid()

# add samples to promp model
for sample in samples:
    
    # pmp.add_demonstration(np.asarray(sample))
    pmp.add_demonstration(sample)

# goal = np.zeros(len(joint_names))


# goal[1] = 1.0
# goal[0] = float('nan')
# pmp.set_goal(goal)

# start = [0.0, 1.0]
# pmp.set_start(start)

context = 1.0
pred = pmp.generate_trajectory(context)

print("prediction = " + str(pred))
print("prediction = " + str(pred.shape))

plt.figure()
plt.plot(x, [y for y in pred], color='blue')    
plt.plot(x, np.ones(pred.shape) * context, '-.',  color='orange')    
plt.title("Prediction")
plt.savefig('pred.png')


pmp.plot_mean_variance()
plt.savefig('mean_variance.png')

# plt.figure()
# pmp.plot(x, output_randomess=-1)
# plt.savefig('mean_std.png')

# plt.show()