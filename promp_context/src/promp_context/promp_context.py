import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

class ProMPContext(object):
    ## Contextualized ProMP as performed in Ewerton et al. 
    ## promplib code used 

    def __init__(self, variable_names, num_basis=11, sigma=0.05, num_samples=100):
        
        self.variables = variable_names
        self.outputs = []
        self.outputs_ix = []

        self.contexts = []
        self.contexts_ix = []

        # detect contexts
        for i, j in enumerate(self.variables):

            # if string starts with c we know it is a context
            if j[0] == 'c':
                self.contexts.append(j)
                self.contexts_ix.append(i)
            # else its an output variable of the trajectory
            else:
                self.outputs.append(j)
                self.outputs_ix.append(i)

        self.num_variables = len(self.variables)
        self.num_outputs = len(self.outputs)
        self.num_contexts = len(self.contexts)

        self.x = np.linspace(0, 1, num_samples)
        self.num_samples = len(self.x)
        self.num_basis = num_basis
        self.sigma = sigma
        self.sigmaSignal = float('inf')  # Noise of signal (float)
        
        ## basis function matrix as function of x (using lambda)
        self.centers = np.arange(0, self.num_basis)/(self.num_basis - 1.0)
        self.Phi = np.exp(-.5 * (np.array(map(lambda x: x - self.centers, np.tile(self.x, (self.num_basis, 1)).T)).T ** 2 
                                    / (self.sigma ** 2)))
        # normalize
        self.Phi /= sum(self.Phi)

        # viapoints
        self.viapoints = []

        # weights
        self.W = np.array([])
        self.mean_w = None
        self.sigma_ww = None

        # contexts
        self.C = np.array([])
        self.mean_c = None
        self.sigma_cc = None

        # covariances
        self.sigma_wc = None
        self.sigma_cw = None


        self.nr_traj = 0

        self.Y = np.empty((0, self.num_samples), float)
        self.C = np.empty((0, 1), float)

    def add_demonstration(self, demonstration):

        demonstration = np.array(demonstration).T
        self.nr_traj += 1

        if len(demonstration) != self.num_variables:
            raise ValueError("The given demonstration has {} joints while num_joints={}".format(len(demonstration), self.num_variables))

        # loop over variables
        for variable_idx, variable in enumerate(demonstration):

            interpolate = interp1d(np.linspace(0, 1, len(demonstration[variable_idx, :])), demonstration[variable_idx, :], kind='cubic')
            stretched_demo = interpolate(self.x)

            # if we have an output variable
            if variable_idx in self.outputs_ix:
                
                # stack Y matrix vertically with this variable
                self.Y = np.vstack((self.Y, stretched_demo))

                # calculate weights (size NxM, N=num_basis, M=num_demonstrations)
                self.W = np.dot(np.linalg.inv(np.dot(self.Phi, self.Phi.T)), np.dot(self.Phi, self.Y.T)).T  # weights for each trajectory

            # if variable is a context
            elif variable_idx in self.contexts_ix:
                ## add the data to the context matrix

                # stack only the context values up until the amount of basis functions
                self.C = np.vstack((self.C, stretched_demo[0]))

        if self.nr_traj > 1:
            # we can only calculate covariance if we have 2 or more demonstrations
            self.sigma_total = np.cov(self.W, self.C, rowvar=0)

            self.sigma_ww = self.sigma_total[:self.num_basis, :self.num_basis]
            self.sigma_cw = self.sigma_total[self.num_basis:, :self.num_basis]
            self.sigma_wc = self.sigma_total[:self.num_basis:, self.num_basis:]
            
            self.sigma_cc = self.sigma_total[self.num_basis:, self.num_basis:]
            
            self.mean_w = np.mean(self.W, 0)                                                            
            self.mean_c = np.mean(self.C, 0)

            print("sigma_cc = " + str(self.sigma_cc))
            print("sigma_ww = " + str(self.sigma_ww))
            print("sigma_wc = " + str(self.sigma_wc))
            print("sigma_cw = " + str(self.sigma_cw))
            print("mean_w = " + str(self.mean_w))
            print("mean_c = " + str(self.mean_c))
            
            self.mean_total = np.append(self.mean_w, self.mean_c)

            print("sigma_total = " + str(self.sigma_total))

        else:
            self.mean_w = self.W[0]
            self.mean_c = self.C[0]

            self.mean_total = np.append(self.mean_w, self.mean_c)

        
    def generate_trajectory(self, context):

        # mu_w|c = mu_w + Sigma_wc * Sigma_cc^-1 * (c - mu_c)
        mu_w_given_c = self.mean_w + np.dot(np.dot(self.sigma_wc, np.linalg.inv(self.sigma_cc)), context - self.mean_c)
        
        # Sigma_w|c = Sigma_ww - Sigma_wc * Sigma^-1 * Sigma_cw
        sigma_w_given_c = self.sigma_ww - np.dot(np.dot(self.sigma_wc, np.linalg.inv(self.sigma_cc)), self.sigma_cw)
        p_w_given_c = np.random.multivariate_normal(mu_w_given_c, sigma_w_given_c)

        # mu_tau|c = Phi * mu_w|c
        mu_traj_given_c = np.dot(self.Phi.T, mu_w_given_c)
        
        # Sigma_tau|c = sigma^2 * I_TxT + Phi * Sigma_w|c * Phi^T
        sigma_traj_given_c = np.dot(self.sigma ** 2, np.eye(self.num_samples)) + np.dot(np.dot(self.Phi.T, sigma_w_given_c), self.Phi)
    
        p_traj_given_c = np.random.multivariate_normal(mu_traj_given_c, sigma_traj_given_c)

        return mu_traj_given_c

    def plot_mean_variance(self, x=None, legend='promp', color=None):
        plt.figure()

        mean = np.dot(self.Phi.T, self.mean_w)
        x = self.x if x is None else x
        plt.plot(x, mean, color=color, label=legend)
        
        # Sigma_tau = sigma^2 I_TxT + Phi * Sigma_w * Phi^T
        # std = sqrt(diag(sigma_tau))
        std = np.sqrt(np.diag(np.dot(self.sigma ** 2, np.eye(self.num_samples)) + np.dot(self.Phi.T, np.dot(self.sigma_ww, self.Phi))))
        
        plt.fill_between(x, mean - std, mean + std, color=color, alpha=0.2)