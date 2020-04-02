import math
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
        self.N += 1

        # auxiliary matrix (ewerton et al.)
        self.S = (self.N - 1) * self.Sigma
        
        newMean = self.Mean + (x - self.Mean)*1./self.N
        newS = self.S + (self.N - 1) / self.N * (x - self.Mean)*(x - self.Mean)
        newSigma = newS / (self.N - 1)
        self.Mean, self.Sigma = newMean, newSigma
        
        return self.Mean, self.Sigma
