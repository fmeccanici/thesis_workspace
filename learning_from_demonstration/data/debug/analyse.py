import ast
import matplotlib.pyplot as plt
from learning_from_demonstration_python.trajectory_parser import trajectoryParser

output_names = ['ee_x', 'ee_y', 'ee_z', 'ee_qx', 'ee_qy', 'ee_qz', 'ee_qw'] 
path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/data/debug/'
text_file = 'x.txt'
plt.figure()
plt.xlabel("Timestep [-]")
plt.ylabel("Position [m]")

x = []

with open(path + text_file, 'r') as f:
    lists = f.read().split('&')
    
    for l in lists:
        try:
            x.append(ast.literal_eval(l))
        except SyntaxError:
            continue

for output in output_names:
    demos = []
    with open(path + output + '.txt', 'r') as f:
        lists = f.read().split('&')
        
        for l in lists:
            try:
                demo = ast.literal_eval(l)
                plt.plot(x[0], demo, label='demonstration')
            except SyntaxError:
                continue

    with open(path + 'prediction_' + output + '.txt', 'r') as f:
        lists = f.read().split('&')
        
        for l in lists:

            try:
                prediction = ast.literal_eval(l)

                plt.plot(x[0], prediction, '--k', label='prediction ' + str(output))

               

            except SyntaxError:
                continue
            

plt.grid()
plt.legend()

traj_wrt_base_data = range(1,5)

parser = trajectoryParser()

plt.figure()
plt.title('Trajectories wrt base')

text_file = 'interp_waypoints.txt'

waypoints = []

with open(path + text_file, 'r') as f:
    waypoints = ast.literal_eval(f.read())

print(waypoints)
x = parser.getXpositions(waypoints)    
y = parser.getYpositions(waypoints)    
z = parser.getZpositions(waypoints)    

qx = parser.get_qx(waypoints)
qy = parser.get_qy(waypoints)
qz = parser.get_qz(waypoints)
qw = parser.get_qw(waypoints)

plt.plot(x)
plt.plot(y)
plt.plot(z)

plt.plot(qx)
plt.plot(qy)
plt.plot(qz)
plt.plot(qw)

for i in traj_wrt_base_data:

    try:
        text_file = 'traj_wrt_base_trial_' + str(i) + '.txt'
        with open(path + text_file, 'r') as f:
            traj = ast.literal_eval(f.read())
            x = parser.getXpositions(traj)
            y = parser.getYpositions(traj)
            z = parser.getZpositions(traj)
            qx = parser.get_qx(waypoints)
            qy = parser.get_qy(waypoints)
            qz = parser.get_qz(waypoints)
            qw = parser.get_qw(waypoints)
            
            plt.plot(x)
            plt.plot(y)
            plt.plot(z)
    
            plt.plot(qx)
            plt.plot(qy)
            plt.plot(qz)
            plt.plot(qw)

    except IOError:
        continue

text_file = 'prediction_wrt_base.txt'
with open(path + text_file, 'r') as f:
    traj = ast.literal_eval(f.read())
    x = parser.getXpositions(traj)
    y = parser.getYpositions(traj)
    z = parser.getZpositions(traj)
    qx = parser.get_qx(waypoints)
    qy = parser.get_qy(waypoints)
    qz = parser.get_qz(waypoints)
    qw = parser.get_qw(waypoints)

    plt.plot(x, '--k')
    plt.plot(y, '--k')
    plt.plot(z, '--k')
    plt.plot(qx)
    plt.plot(qy)
    plt.plot(qz)
    plt.plot(qw)


plt.figure()
plt.title('Trajectories wrt object')
for i in traj_wrt_base_data:

    try:
        text_file = 'traj_wrt_object_trial_' + str(i) + '.txt'
        with open(path + text_file, 'r') as f:
            traj = ast.literal_eval(f.read())
            x = parser.getXpositions(traj)
            y = parser.getYpositions(traj)
            z = parser.getZpositions(traj)

            plt.plot(x)
            plt.plot(y)
            plt.plot(z)
    except IOError:
        continue

text_file = 'prediction_wrt_object.txt'
with open(path + text_file, 'r') as f:
    traj = ast.literal_eval(f.read())
    x = parser.getXpositions(traj)
    y = parser.getYpositions(traj)
    z = parser.getZpositions(traj)

    plt.plot(x, '--k')
    plt.plot(y, '--k')
    plt.plot(z, '--k')

plt.grid()
plt.show()
