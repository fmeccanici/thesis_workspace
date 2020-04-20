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


        self.nrTraj = 0

        self.Y = np.empty((0, self.num_samples), float)
        self.C = np.empty((0, self.num_basis), float)

    def add_demonstration(self, demonstration):

        demonstration = np.array(demonstration).T

        if len(demonstration) != self.num_variables:
            raise ValueError("The given demonstration has {} joints while num_joints={}".format(len(demonstration), self.num_variables))

        # loop over variables
        for variable_idx, variable in enumerate(demonstration):

            interpolate = interp1d(np.linspace(0, 1, len(demonstration[variable_idx, :])), demonstration[variable_idx, :], kind='cubic')
            stretched_demo = interpolate(self.x)

            if variable_idx in self.outputs_ix:
                self.Y = np.vstack((self.Y, stretched_demo))

                self.nrTraj = len(self.Y)
                self.W = np.dot(np.linalg.inv(np.dot(self.Phi, self.Phi.T)), np.dot(self.Phi, self.Y.T)).T  # weights for each trajectory
                print("W = " + str(self.W))

                self.mean_w = np.mean(self.W, 0)                                                             # mean of weights
                print("mean_w = " + str(self.mean_w))
                # w1 = np.array(map(lambda x: x - self.meanW.T, self.W))
                self.sigma_ww = np.var(self.W, 0)
                print("sigma_ww = " + str(self.sigma_ww))
                # self.sigma_w = np.dot(w1.T, w1)/self.nrTraj                                                  # covariance of weights
                # self.sigmaSignal = np.sum(np.sum((np.dot(self.W, self.Phi) - self.Y) ** 2)) / (self.nrTraj * self.num_samples)
            
            # if variable is a context
            elif variable_idx in self.contexts_ix:
                ## add the data to the context matrix

                # stack only the context values up until the amount of basis functions
                self.C = np.vstack((self.C, stretched_demo[:self.num_basis]))
                print("C = " + str(self.C))

                self.mean_c = np.mean(self.C, 0)      
                print("mean_c = " + str(self.mean_c))

                self.sigma_cc = np.var(self.C, 0)
                print("sigma_cc = " + str(self.sigma_cc))
                # print(self.C)
                # print(self.sigma_cc)

            if self.C.shape[0] != 0 and self.W.shape[0] != 0:
                self.sigma_wc = np.cov(self.W, self.C, rowvar=0)
                self.sigma_cw = np.cov(self.C, self.W, rowvar=0)

                print("sigma_wc = " + str(self.sigma_wc))
                print("sigma_cw = " + str(self.sigma_cw))

        # stack mean matrix
        self.mu_total = np.vstack((self.mean_w, self.mean_c))
        
        print(np.hstack((self.sigma_cc, self.sigma_wc)))


        # stack covariance matrix
        self.sigma_total = np.vstack((np.hstack((self.sigma_cc, self.sigma_wc)), np.hstack((self.sigma_cw, self.sigma_ww))))

    def generate_trajectory(self, context):

        # mu_w|c = mu_w + Sigma_wc * Sigma_cc^-1 * (c - mu_c)
        mu_w_given_c = self.mean_w + np.dot(np.dot(self.sigma_wc, np.linalg.inv(self.sigma_cc)), context - self.mean_c)
        
        # Sigma_w|c = Sigma_ww - Sigma_wc * Sigma^-1 * Sigma_cw
        sigma_w_given_c = self.sigma_ww - np.dot(np.dot(self.sigma_wc, np.linalg.inv(self.sigma_cc)), self.sigma_cw)
        p_w_given_c = np.random.multivariate_normal(mu_w_given_c, sigma_w_given_c)

        # mu_tau|c = Phi * mu_w|c
        mu_traj_given_c = np.dot(self.Phi, mu_w_given_c)
        
        # Sigma_tau|c = sigma^2 * I_TxT + Phi * Sigma_w|c * Phi^T
        sigma_traj_given_c = np.dot(self.sigma ** 2, np.eye(self.num_samples)) + np.dot(np.dot(self.Phi, self.sigma_w_given_c), self.Phi.T)
    
        return mu_traj_given_c

    def get_bounds(self, t):
        """
        Return the bounds of all joints at time t
        :param t: 0 <= t <= 1
        :return: [(lower boundary joints 0, upper boundary joints 0), (lower boundary joint 1), upper)...]
        """
        return [joint.get_bounds(t) for joint in self.promps]

    def get_means(self, t):
        """
        Return the mean of all joints at time t
        :param t: 0 <= t <= 1
        :return: [mean joint 1, mean joint 2, ...]
        """
        return [joint.get_mean(t) for joint in self.promps]

    def get_stds(self):
        """
        Return the standard deviation of all joints
        :param t: 0 <= t <= 1
        :return: [std joint 1, std joint 2, ...]
        """
        return [joint.get_std() for joint in self.promps]

    def clear_viapoints(self):
        for promp in self.promps:
            promp.clear_viapoints()


    def plot(self, x=None, joint_names=(), output_randomess=0.5):
        """
        Plot the means and variances of gaussians, requested viapoints as well as an output trajectory (dotted)
        :param output_randomess: 0. to 1., -1 to disable output plotting
        """
        if output_randomess >= 0:
            output = self.generate_trajectory(output_randomess).T

        for promp_idx, promp in enumerate(self.promps):
            color = self.colors[promp_idx % len(self.colors)]
            joint_name = "Joint {}".format(promp_idx+1) if len(joint_names) == 0 else joint_names[promp_idx]
            promp.plot(x, joint_name, color)
            if output_randomess >= 0:
                plt.plot(x, output[promp_idx], linestyle='--', label="Out {}".format(joint_name), color=color, lw=2)

class ProMP(object):
    """
    Uni-dimensional probabilistic MP
    """
    def __init__(self, nrBasis=11, sigma=0.05, num_samples=100):
        self.x = np.linspace(0, 1, num_samples)
        self.nrSamples = len(self.x)
        self.nrBasis = nrBasis
        self.sigma = sigma
        self.sigmaSignal = float('inf')  # Noise of signal (float)
        self.C = np.arange(0,nrBasis)/(nrBasis-1.0)
        self.Phi = np.exp(-.5 * (np.array(map(lambda x: x - self.C, np.tile(self.x, (self.nrBasis, 1)).T)).T ** 2 / (self.sigma ** 2)))
        self.Phi /= sum(self.Phi)

        self.viapoints = []
        self.W = np.array([])
        self.nrTraj = 0
        self.meanW = None
        self.sigmaW = None
        self.Y = np.empty((0, self.nrSamples), float)

    def add_demonstration(self, demonstration):
        interpolate = interp1d(np.linspace(0, 1, len(demonstration)), demonstration, kind='cubic')
        stretched_demo = interpolate(self.x)
        self.Y = np.vstack((self.Y, stretched_demo))
        self.nrTraj = len(self.Y)
        self.W = np.dot(np.linalg.inv(np.dot(self.Phi, self.Phi.T)), np.dot(self.Phi, self.Y.T)).T  # weights for each trajectory
        self.meanW = np.mean(self.W, 0)                                                             # mean of weights
        w1 = np.array(map(lambda x: x - self.meanW.T, self.W))
        self.sigmaW = np.dot(w1.T, w1)/self.nrTraj                                                  # covariance of weights
        self.sigmaSignal = np.sum(np.sum((np.dot(self.W, self.Phi) - self.Y) ** 2)) / (self.nrTraj * self.nrSamples)

    def get_bounds(self, t):
        """
        Return the bounds at time t
        :param t: 0 <= t <= 1
        :return: (lower boundary, upper boundary)
        """
        return self._get_bounds(int(self.num_points*t))

    def get_mean(self, t):
        """
        Return the mean at time t
        :param t: 0 <= t <= 1
        :return: scalar
        """
        return self._get_mean(int(self.num_points*t))

    def get_std(self):
        std = 2 * np.sqrt(np.diag(np.dot(self.Phi.T, np.dot(self.sigmaW, self.Phi))))
        return std

    def _get_mean(self, t_index):
        mean = np.dot(self.Phi.T, self.meanW)
        return mean[t_index]

    def _get_bounds(self, t_index):
        mean = self._get_mean(t_index)
        std = self.get_std()
        return mean - std, mean + std

    def clear_viapoints(self):
        del self.viapoints[:]

    def add_viapoint(self, t, obsy, sigmay=1e-6):
        """
        Add a viapoint to the trajectory
        Observations and corresponding basis activations
        :param t: timestamp of viapoint
        :param obsy: observed value at time t
        :param sigmay: observation variance (constraint strength)
        :return:
        """
        self.viapoints.append({"t": t, "obsy": obsy, "sigmay": sigmay})

    def set_goal(self, obsy, sigmay=1e-6):
        self.add_viapoint(1., obsy, sigmay)

    def set_start(self, obsy, sigmay=1e-6):
        self.add_viapoint(0., obsy, sigmay)

    # def generate_trajectory(self, randomness=1e-10):
    #     """
    #     Outputs a trajectory
    #     :param randomness: float between 0. (output will be the mean of gaussians) and 1. (fully randomized inside the variance)
    #     :return: a 1-D vector of the generated points
    #     """
    #     newMu = self.meanW
    #     newSigma = self.sigmaW

    #     for viapoint in self.viapoints:
    #         PhiT = np.exp(-.5 * (np.array(map(lambda x: x - self.C, np.tile(viapoint['t'], (11, 1)).T)).T ** 2 / (self.sigma ** 2)))
    #         PhiT = PhiT / sum(PhiT)  # basis functions at observed time points

    #         # Conditioning
    #         aux = viapoint['sigmay'] + np.dot(np.dot(PhiT.T, newSigma), PhiT)
            
            
    #         newMu = newMu + np.dot(np.dot(newSigma, PhiT) * 1 / aux, (viapoint['obsy'] - np.dot(PhiT.T, newMu)))  # new weight mean conditioned on observations
            
            
    #         newSigma = newSigma - np.dot(np.dot(newSigma, PhiT) * 1 / aux, np.dot(PhiT.T, newSigma))

    #     sampW = np.random.multivariate_normal(newMu, randomness*newSigma, 1).T
    #     return np.dot(self.Phi.T, sampW)

    def plot(self, x=None, legend='promp', color=None):
        mean = np.dot(self.Phi.T, self.meanW)
        x = self.x if x is None else x
        plt.plot(x, mean, color=color, label=legend)
        std = 2*np.sqrt(np.diag(np.dot(self.Phi.T, np.dot(self.sigmaW, self.Phi))))
        plt.fill_between(x, mean - std, mean + std, color=color, alpha=0.2)
        for viapoint_id, viapoint in enumerate(self.viapoints):
            x_index = x[int(round((len(x)-1)*viapoint['t'], 0))]
            plt.plot(x_index, viapoint['obsy'], marker="o", markersize=10, label="Via {} {}".format(viapoint_id, legend), color=color)
