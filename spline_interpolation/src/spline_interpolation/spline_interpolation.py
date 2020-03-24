#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from scipy import interpolate

class splineInterpolation():
    def interpolate(self, y, dt, t, T, k=3):
        

        # tck = interpolate.splrep(t, y, s=2, k=3)
        cs = CubicSpline(t, y)


        t = np.arange(t[0], T, dt)

        # ynew = interpolate.splev(t, tck)
        ynew = cs(t)
        # print(t)
        # print(ynew)
        # x = [y_[0] for y_ in ynew]
        # y = [y_[1] for y_ in ynew]
        # z = [y_[2] for y_ in ynew]

        # plt.figure()
        # plt.plot(t, x, 'b')
        # plt.plot(t, y, 'r')
        # plt.plot(t, z, 'g')

        # plt.show()
        return t,ynew
        
if __name__ == "__main__":
    interp = splineInterpolation()

    dt = 0.01

    # y1 = [1, 1]
    # y2 = [2, 0]
    # y3 = [3, 0]
    # y4 = [4, 0]
    y1 = 4
    y2 = 1
    y3 = 1
    y4 = 1
    
    t1 = 0
    t2 = 1
    t3 = t2 + dt
    t4 = t3 + dt

    t = [t1, t2, t3, t4]
    y = [y1, y2, y3, y4]

    # t = [t1, t2]
    # y = [y1, y2]
    
    interp.interpolate(y, dt, t)