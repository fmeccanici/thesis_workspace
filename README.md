# Thesis workspace
This is the src folder of the ROS workspace used in my thesis. In this README, small descriptions of the most important packages are given. 

## promp_context_ros
This contains the code for the Conditioned Probabilistic Movement Primitives, where condition is called context in the code. This class is able to set the input/output names (input=condition), add demonstrations and generate a trajectory using a new context. Adding demonstrations can be done normally, by appending the weight matrix, or by using Welford's method for updating the covariance matrix incrementally.

## learning_from_demonstration
This contains the code for applying learning from demonstration using Conditioned Probabilistic Movement Primitives (C-ProMP) on the robot. The ROS node contains services to create an initial model, add demonstrations and make predictions from other ROS nodes. The Python packages inside the src folder contain all the logic that enables LfD, such as Dynamic Time Warping and trajectory resampling. The learning_from_demonstration.py file contains a high level usage of the C-ProMP package, that enables learning from demonstration on this specific robot. 

## trajectory_refinement
