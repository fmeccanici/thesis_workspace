# Thesis workspace
This is the src folder of the ROS workspace used in my thesis. In this README, small descriptions of the most important packages are given. The Conditioned-ProMP and online learning framework are implemented according to [1], so you should reference this paper when reading the code. This thesis is written to work within the HIT Docker, and changes should be made inside this docker to make it work. The telemanip_master and telemanip_slave from within this GIT repository should be copied inside the docker, or put in a ROS workspace and sourced. 

## promp_context_ros
This contains the code for the Conditioned Probabilistic Movement Primitives, where condition is called context in the code. This class is able to set the input/output names (input=condition), add demonstrations and generate a trajectory using a new context. Adding demonstrations can be done normally, by appending the weight matrix, or by using Welford's method for updating the covariance matrix incrementally.

## learning_from_demonstration
This contains the code for applying learning from demonstration using Conditioned Probabilistic Movement Primitives (C-ProMP) on the robot. The ROS node contains services to create an initial model, add demonstrations and make predictions from other ROS nodes. The Python packages inside the src folder contain all the logic that enables LfD, such as Dynamic Time Warping and trajectory resampling. The learning_from_demonstration.py file contains a high level usage of the C-ProMP package, that enables learning from demonstration on this specific robot. 

## trajectory_refinement
This contains a ROS node that is used to adapt a trajectory during execution time. The refineTrajectory function uses a trajectory as input and gives as output the adapted trajectory. The new trajectory is then the old prediction + the difference between the old and the new prediction (see [1]).
This also contains the node refinement_force_publisher.py that is used to determine the haptic feedback force felt when applying a certain refinement. 

# References
[1]: Incremental imitation learning of context-dependent motor skills, Ewerton et al.
