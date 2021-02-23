import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from welford.welford import Welford
from padasip import *

class ProMPContext(object):
    ## Contextualized ProMP as performed in Ewerton et al. 
    ## promplib code used 

    def __init__(self, output_name, context_names, num_basis=11, sigma=0.05, num_samples=100):
        

        self.output_name = [output_name]
        self.context_names = context_names
        self.num_outputs = len(self.output_name)
        self.num_contexts = len(self.context_names)
        
        self.x = np.linspace(0, 1, num_samples)
        self.num_samples = len(self.x)
        self.num_basis = num_basis
        self.sigma = sigma
        self.sigmaSignal = float('inf')  # Noise of signal (float)
        
        self.centers = np.arange(0, self.num_basis)/(self.num_basis - 1.0)

        self.phi = np.exp(-0.5 * (np.array(list(map(lambda x: x - self.centers, np.tile(self.x, (self.num_basis, 1)).T))).T ** 2
                                  / self.sigma**2))
        self.phi /= np.sum(self.phi, axis=0)
        self.Phi = np.zeros((self.num_basis*self.num_outputs, self.num_samples*self.num_outputs))
        for i in range(0, self.num_outputs):
            self.Phi[i*self.num_basis:(i+1)*self.num_basis, i*self.num_samples:(i+1)*self.num_samples] = self.phi

        # normalize
        self.Phi /= sum(self.Phi)

        # weights
        self.W = np.array([])
        self.mean_w = None
        self.sigma_ww = None

        # contexts = conditions
        self.C = np.array([])
        self.mean_c = None
        self.sigma_cc = None

        # covariances
        self.sigma_wc = None
        self.sigma_cw = None

        self.nr_traj = 0

        self.Y = np.empty((0, self.num_samples), float)
        self.C = np.empty((0, self.num_contexts), float)


    def save_plots(self):
        pass
        # used in other code so kept this function

    # welford update where x = [w_M, c_M]
    # thus containing the weights and context of the refined trajectory
    def welford_update(self, demonstration, amount=1, alpha=0.0):
        
        trajectory = demonstration[0]
        context = demonstration[1]
        trajectory = np.array(trajectory).T
        if len(trajectory) != self.num_outputs:
            raise ValueError("The given demonstration has {} outputs while num_outputs={}".format(len(trajectory), self.num_outputs))

        self.nr_traj += 1

        # first we calculate the weights
        # according to Ewerton et al.
        for variable_idx, variable in enumerate(trajectory):

            # interpolate to fit the Phi matrix
            interpolate = interp1d(np.linspace(0, 1, len(trajectory[variable_idx, :])), trajectory[variable_idx, :], kind='cubic')
            
            # name convention from Ewerton et al.
            tau = interpolate(self.x)

            # calculate weights of Mth demonstration: refinement
            # (size N, N=num_basis, M=num_demonstrations)
            w_M = np.dot(np.linalg.inv(np.dot(self.Phi, self.Phi.T)), np.dot(self.Phi, tau)).T
            c_M = context

            x = np.append(w_M, c_M)

            # do a welford update step and update the mean and variance
            print("Number of trajectories = " + str(self.nr_traj))
            N = self.nr_traj
            # N = 2

            welford = Welford(N=N, Mean=self.mean_total, Sigma=self.sigma_total, alpha=alpha)

            if amount > 1:
                for i in range(amount):
                    mean_total, sigma_total = welford.update(x)
            else: 
                mean_total, sigma_total = welford.update(x)

            self.mean_total = mean_total
            self.sigma_total = sigma_total

            self.sigma_ww = self.sigma_total[:self.num_basis, :self.num_basis]
            self.sigma_cw = self.sigma_total[self.num_basis:, :self.num_basis]
            self.sigma_wc = self.sigma_total[:self.num_basis:, self.num_basis:]
            self.sigma_cc = self.sigma_total[self.num_basis:, self.num_basis:] 

            self.mean_w = self.mean_total[:self.num_basis] 
            self.mean_c = self.mean_total[self.num_basis:] 

    def add_demonstration(self, demonstration):

        trajectory = demonstration[0]
        context = demonstration[1]

        trajectory = np.array(trajectory).T

        
        self.nr_traj += 1

        if len(trajectory) != self.num_outputs:
            raise ValueError("The given demonstration has {} outputs while num_outputs={}".format(len(trajectory), self.num_outputs))


        # loop over variables
        for variable_idx, variable in enumerate(trajectory):
            interpolate = interp1d(np.linspace(0, 1, len(trajectory[variable_idx, :])), trajectory[variable_idx, :], kind='cubic')
            stretched_demo = interpolate(self.x)

            # stack Y matrix vertically with this variable
            self.Y = np.vstack((self.Y, stretched_demo))

            # calculate weights (size NxM, N=num_basis, M=num_demonstrations)
            self.W = np.dot(np.linalg.inv(np.dot(self.Phi, self.Phi.T)), np.dot(self.Phi, self.Y.T)).T  # weights for each trajectory
            self.C = np.vstack((self.C, context))        
            # print("C = " + str(self.C))
            
        if self.nr_traj > 1:
            # we can only calculate covariance if we have 2 or more demonstrations

            self.sigma_total = np.cov(self.W, self.C, rowvar=0)

            self.sigma_ww = self.sigma_total[:self.num_basis, :self.num_basis]
            self.sigma_cw = self.sigma_total[self.num_basis:, :self.num_basis]
            self.sigma_wc = self.sigma_total[:self.num_basis:, self.num_basis:]
            
            self.sigma_cc = self.sigma_total[self.num_basis:, self.num_basis:]
            
            self.mean_w = np.mean(self.W, 0)                                                            
            self.mean_c = np.mean(self.C, 0)
            
            self.mean_total = np.append(self.mean_w, self.mean_c)


        else:
            self.mean_w = self.W[0]
            self.mean_c = self.C[0]

            self.mean_total = np.append(self.mean_w, self.mean_c)
        
        print("Number of trajectories = " + str(self.nr_traj))

    def generate_trajectory(self, context):

        # noise preventing the matrix Sigma_cc to be singular
        noise = np.eye(self.sigma_cc.shape[0]) * self.sigma

        # mu_w|c = mu_w + Sigma_wc * Sigma_cc^-1 * (c - mu_c)
        mu_w_given_c = self.mean_w + np.dot(np.dot(self.sigma_wc, np.linalg.inv(self.sigma_cc + noise)), context - self.mean_c)
        
        # Sigma_w|c = Sigma_ww - Sigma_wc * Sigma^-1 * Sigma_cw
        sigma_w_given_c = self.sigma_ww - np.dot(np.dot(self.sigma_wc, np.linalg.inv(self.sigma_cc + noise)), self.sigma_cw)
        p_w_given_c = np.random.multivariate_normal(mu_w_given_c, sigma_w_given_c)

        # mu_tau|c = Phi * mu_w|c
        mu_traj_given_c = np.dot(self.Phi.T, mu_w_given_c)
        
        # Sigma_tau|c = sigma^2 * I_TxT + Phi * Sigma_w|c * Phi^T
        sigma_traj_given_c = np.dot(self.sigma ** 2, np.eye(self.num_samples)) + np.dot(np.dot(self.Phi.T, sigma_w_given_c), self.Phi)
    
        p_traj_given_c = np.random.multivariate_normal(mu_traj_given_c, sigma_traj_given_c)
            
        return mu_traj_given_c

    def plot_mean_variance(self, x=None, legend='promp', color=None):
        plt.figure(self.figs[1])

        mean = np.dot(self.Phi.T, self.mean_w)
        x = self.x if x is None else x
        plt.plot(x, mean, color=color, label=legend)
        
        # Sigma_tau = sigma^2 I_TxT + Phi * Sigma_w * Phi^T
        # std = sqrt(diag(sigma_tau))
        std = np.sqrt(np.diag(np.dot(self.sigma ** 2, np.eye(self.num_samples)) + np.dot(self.Phi.T, np.dot(self.sigma_ww, self.Phi))))
        
        plt.fill_between(x, mean - std, mean + std, color=color, alpha=0.2)
        
