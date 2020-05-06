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

path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/promp_demo_2d/data/'
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
    plt.plot(x, demonstration[0])
plt.grid()
plt.legend()

# initialize lwpr model
model = LWPR(n_in, n_out)
model.init_D = 50*eye(n_in)
model.init_alpha = 40 * eye(n_in)

for i in range(100):
    for demonstration in demonstrations:
        output = np.asarray(demonstration[0])
        context = np.asarray(demonstration[1])

        print("added output: " + str(output))
        print("added context: " + str(context))
        
        model.update(context, output)

# generalize
context = np.asarray([-1.0, 1.5])
output = model.predict(context)

print("predicted output: " + str(output))

plt.figure()
plt.title("Predicted trajectory")
plt.ylim([y_min, y_max])
plt.plot(x, output, 'r-', label='context = ' + str(context))
plt.grid()
plt.legend()
plt.show()
