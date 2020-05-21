import numpy as np
from promp_context.promp_context import ProMPContext
import random
import matplotlib.pyplot as plt


class ModelTest(object):
    def __init__(self, num_outputs, num_contexts, num_trajs):
        self.sigma_noise=0.03
        self.x = np.arange(0,1,0.01)
        self.num_outputs = num_outputs
        self.num_contexts = num_contexts

        output_names = []
        context_names = []

        for i in range(num_outputs):
            output_names.append("output_" + str(i))
        
        for i in range(num_contexts):
            context_names.append("context_" + str(i))

        self.num_samples = len(self.x)
        self.promps = [ProMPContext(output_name, context_names, num_samples=self.num_samples,num_basis=20) for output_name in output_names]

        self.samples = []
        self.num_trajs = num_trajs

    def generate_plots(self, case='linear'):
        plt.figure()
        for i in range(self.num_trajs):
            context = self.num_contexts * [i]
            
            # generate random colors
            r = round(random.random(), 1)
            b = round(random.random(), 1)
            g = round(random.random(), 1)
            color = (r, g, b)

            if case == 'linear':
                y = []
                for j in range(1, self.num_outputs+1):
                    if self.num_outputs > 1:
                        y_temp = [ [x + j*(i+1)] for x in self.x] 
                    else:
                        y_temp = [ [x + j*i] for x in self.x] 

                    plt.plot(self.x, y_temp, c=color, label='output ' + str(i+1))
                    y.append(y_temp)

                print(str(y) + '\n')

            elif case == 'non-linear':
                y = []
                for j in range(1, self.num_outputs+1):
                    y_temp = list(map(lambda x: [np.sin(10*x) + j*i], self.x))
                    plt.plot(self.x, y_temp, c=color, label='output ' + str(i+1))
                    y.append(y_temp)

            for i in range(len(context)):
                plt.plot(self.x, np.ones(len(self.x)) * context[i], '-.', c=color, label='input ' + str(i+1))

            sample = (y, context)
            self.samples.append(sample)
        
        plt.legend()
        plt.title("Demonstrations")
        plt.xlabel("x [-]")
        plt.ylabel("y [-]")
        plt.grid()
        plt.show()
        plt.savefig('./figures/demonstrations_' + case + '_num_inputs_' + str(self.num_contexts) + 'num_outputs_' + str(self.num_outputs) + '.png')


        # add samples to promp model
        print("Adding demonstrations")
        for sample in self.samples:
            for i in range(self.num_outputs):
                print(sample[0][i])
                # list comprehension to add only one parameter array to each promp 
                self.promps[i].add_demonstration( (sample[0][i], sample[1]) )

        if case == 'linear':
            context = self.num_contexts * [1.0]
        
        plt.figure()
        for i in range(self.num_outputs):
            pred = self.promps[i].generate_trajectory(context)
            plt.plot(self.x, [y for y in pred], color='blue', label='prediction')    
        
        for i in range(self.num_contexts):
            plt.plot(self.x, np.ones(pred.shape) * context[i], '-',  color='orange', label='context', linewidth=2)   

        plt.title("Prediction")
        plt.xlabel("x [-]")
        plt.ylabel("y [-]")
        plt.grid()
        plt.legend()
        
        plt.savefig('./figures/pred_' + case + '_num_inputs_' + str(self.num_contexts) + '_num_outputs_' + str(self.num_outputs) + '.png')


if __name__ == "__main__":
    
    model_tester4 = ModelTest(3, 1, 2)
    model_tester4.generate_plots(case='linear')

    model_tester1 = ModelTest(1, 1, 5)
    model_tester1.generate_plots(case='linear')

    model_tester2 = ModelTest(1, 1, 5)
    model_tester2.generate_plots(case='non-linear')

    model_tester3 = ModelTest(1, 3, 5)
    model_tester3.generate_plots(case='linear')


    

"""# create samples
for i in range(num_traj):
    # y = list(map(lambda x: [x + i, x + i, x + i], x))
    y = list(map(lambda x: [x + i], x))
    # y1 = list(map(lambda x: [np.sin(10*x) + i], x))
    # y2 = list(map(lambda x: [np.sin(10*x) + 2*i], x))
    # y3 = list(map(lambda x: [np.sin(10*x) + 3*i], x))

    
    # context1 = [i, i, i]
    context = [i]

    # context = [i, i, i]
    # context = [ [ x[0], x[1], x[2] ] for x in context ]
    # print(context)
    # generate random colors
    r = round(random.random(), 1)
    b = round(random.random(), 1)
    g = round(random.random(), 1)
    color = (r, g, b)
    


    plt.plot(x, y, c=color, label='input ' + str(i+1))
    # plt.plot(x, context[i], '-.', c=color, label='context ' + str(i+1))

    # plt.plot(x, y1, c=color, label='output ' + str(1))
    # plt.plot(x, y2, c=color, label='output ' + str(2))

    for i in range(len(context)):

        plt.plot(x, np.ones(len(x)) * context[i], '-.', c=color, label='input ' + str(i))
    # plt.plot(x, np.ones(len(x)) * context1, '-.', c=color, label='input ' + str(i+1))


    plt.title("Demonstrations")
    # sample = list(map(lambda x, y: [x, y], y, context))
    sample = (y, context)
    # print(context)
    # sample = list(map(lambda x: [x, i, i, i], y))
    # sample1 = (y1, context1)
    # sample2 = (y2, context2)

    # samples1.append(sample1)
    samples1.append(sample)

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

# context = [1.0, 1.0, 1.0]
context = 1.0

# for promp in promps:
pred = promps[0].generate_trajectory(context)
plt.figure()

plt.plot(x, [y for y in pred], color='blue', label='prediction')    
plt.plot(x, np.ones(pred.shape) * context, '-',  color='orange', label='context', linewidth=2)   
print(np.ones(pred.shape) * context)
# plt.plot(x, np.ones(pred.shape) * context[0], '-',  color='orange', label='context1')   
# plt.plot(x, np.ones(pred.shape) * context[1], '-.',  color='black', label='context2')   
# plt.plot(x, np.ones(pred.shape) * context[2], '--',  color='green', label='context3')   

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

# plt.show()"""