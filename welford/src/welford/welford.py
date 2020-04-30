import math
import numpy as np

class Welford():

    def __init__(self, N, Mean, Sigma, lst=None):
        if Sigma is not None and Mean is not None:
            self.N = N
            self.Mean = Mean
            self.Sigma = Sigma
        else:
            self.N = 0
            self.Mean = 0
            self.Sigma = 0
            self.S = 0
    
    def update(self,x):
        if x is None:
            return
        # self.N += 1

        # forgetting factor
        alpha = 0.99


        # auxiliary matrix (ewerton et al.)
        self.S = (self.N - 1) * self.Sigma

        # initialize np matrix        
        newS = np.zeros((self.S.shape))
        newSigma = np.zeros((self.S.shape))

        newMean = self.Mean + (x - self.Mean)*1./self.N

        # update covariance matrix
        for i in range(len(x)):
            for j in range(len(x)):
                newS[i,j] = (1-alpha)*self.S[i,j] + alpha*(self.N - 1) / self.N * (x[i] - self.Mean[i]) * (x[j] - self.Mean[j])
                newSigma[i,j] = newS[i,j] / (self.N - 1)

        self.Mean, self.Sigma = newMean, newSigma
        
        return self.Mean, self.Sigma
