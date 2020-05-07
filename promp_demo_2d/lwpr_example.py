from numpy import *
from matplotlib import pyplot as plt
from lwpr import LWPR
import os, ast
import numpy as np
from random import *

demonstrations = []
R = Random()
y_min = -2
y_max = 2

def load_demonstration_from_folder(path, traj_file):
    with open(path+traj_file, "r") as traj:
        demo = ast.literal_eval(traj.read())    
        demonstrations.append(demo)

def load_demonstrations_from_folder(path):
    files = [name for name in os.listdir(path) if os.path.isfile(os.path.join(path, name))]
    
    # get numbers from these files
    numbers = [int(os.path.splitext(f)[0].split('_')[-1]) for f in files]
    
    # sort them in ascending order
    numbers.sort()

    # make list of these files
    files = ["demonstration_" + str(number) + ".txt" for number in numbers]

    for demo in files:
        load_demonstration_from_folder(path, demo)

path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/promp_demo_2d/data/good/'
load_demonstrations_from_folder(path)

# dimension of outputs
n_out = len(demonstrations[0][0])

# dimension of inputs
n_in = len(demonstrations[0][1])

# n_out = 1

# create x vector for plotting
x = np.linspace(0, 4, n_out)

plt.figure()
plt.ylim([y_min, y_max])
plt.title("Demonstrated trajectories")

for demonstration in demonstrations:
    plt.plot(x, demonstration[0], 'o-')
    context = np.asarray(demonstration[1])

    plt.title("Predicted trajectory")
    plt.ylim([y_min, y_max])
    context1 = [2.0, context[0]]
    context2 = [3.6, context[1]]

    circle1 = plt.Circle((context1[0], context1[1]), 0.1, color='b', fill=False)
    circle2 = plt.Circle((context2[0], context2[1]), 0.1, color='b', fill=False)
    ax = plt.gca()
    ax.add_artist(circle1)
    ax.add_artist(circle2)
plt.grid()
plt.savefig('/home/fmeccanici/Documents/thesis/thesis_workspace/src/promp_demo_2d/figures/lwpr/lwpr_demos.png')
plt.clf()



# initialize lwpr model
model = LWPR(n_in, n_out)
model.init_D = 100*eye(n_in)
model.init_alpha = 10 * eye(n_in)
# model.kernel = 'BiSquare'

for i in range(10):
    for demonstration in demonstrations:
        output = np.asarray(demonstration[0])
        context = np.asarray(demonstration[1])

        # print("added output: " + str(output))
        # print("added context: " + str(context))
        
        model.update(context, output)

# generalize
y = [-1.0, 0.0, 1.0]

for y1 in y:
    plt.figure()
    for y2 in y: 
        context = np.asarray([y1, y2])
        output, conf = model.predict_conf(context)

        # print("predicted output: " + str(output))

        plt.title("Predicted trajectory")
        plt.ylim([y_min, y_max])
        plt.plot(x, output, 'ro-')
        context1 = [2.0, context[0]]
        context2 = [3.6, context[1]]

        circle1 = plt.Circle((context1[0], context1[1]), 0.1, color='b', fill=False)
        circle2 = plt.Circle((context2[0], context2[1]), 0.1, color='b', fill=False)
        ax = plt.gca()
        ax.add_artist(circle1)
        ax.add_artist(circle2)
        plt.grid()
        plt.savefig('/home/fmeccanici/Documents/thesis/thesis_workspace/src/promp_demo_2d/figures/lwpr/lwpr_prediction_' + str(int(y1)) + str(int(y2)) + '.png' )
        circle1.remove()
        circle2.remove()
        plt.clf()
