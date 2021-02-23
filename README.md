# Thesis workspace
This is the src folder of the ROS workspace used in my thesis. In this README, small descriptions of the most important packages are given. The Conditioned-ProMP and online learning framework are implemented according to [1], so you should reference this paper when reading the code. This thesis is written to work within the HIT Docker, and changes should be made inside this docker to make it work. The telemanip_master and telemanip_slave from within this GIT repository should be copied inside the docker, or put in a ROS workspace and sourced. 

## promp_context_ros
This contains the code for the Conditioned Probabilistic Movement Primitives, where condition is called context in the code. This class is able to set the input/output names (input=condition), add demonstrations and generate a trajectory using a new context. Adding demonstrations can be done normally, by appending the weight matrix, or by using Welford's method for updating the covariance matrix incrementally.

### Example code
Copy the promp_context from within the src folder, this is the Python class only. The following code is able to make an initial model and make new predictions towards other contexts/conditions/inputs: 
```python
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
```
## learning_from_demonstration
This contains the code for applying learning from demonstration using Conditioned Probabilistic Movement Primitives (C-ProMP) on the robot. The ROS node contains services to create an initial model, add demonstrations and make predictions from other ROS nodes. The Python packages inside the src folder contain all the logic that enables LfD, such as Dynamic Time Warping and trajectory resampling. The learning_from_demonstration.py file contains a high level usage of the C-ProMP package, that enables learning from demonstration on this specific robot. 

## trajectory_refinement
This contains a ROS node that is used to adapt a trajectory during execution time. The refineTrajectory function uses a trajectory as input and gives as output the adapted trajectory. The new trajectory is then the old prediction + the difference between the old and the new prediction (see [1]).
This also contains the node refinement_force_publisher.py that is used to determine the haptic feedback force felt when applying a certain refinement. 

## experiment
This package contains all the nodes used to run the human factors experiment. You should run ```roslaunch experiment experiment.launch``` to run the data logger, learning from demonstration, trajectory refinement, the execution failure detections and the necessary nodes needed to use the different methods (online/offline + omni/teach pendant). 

## nasa_tlx
GUI used for the participants to fill in the Nasa TLX questionaire and calculate the workload. 

# References
[1]: Incremental imitation learning of context-dependent motor skills, Ewerton et al.
