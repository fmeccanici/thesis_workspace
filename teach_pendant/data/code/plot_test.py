import ast
import matplotlib.pyplot as plt

path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/teach_pendant/data/'
traj_file = 'raw_trajectory_25.txt'
with open(path+traj_file, "r") as traj:
    raw_trajectory = ast.literal_eval(traj.read())    
x = []
y = []
z = []

for data in raw_trajectory:
    x.append(data[0])
    y.append(data[1])
    z.append(data[2])

plt.plot(x, label='x')
plt.plot(y, label='y')
plt.plot(z, label='z')
plt.legend()
plt.show()