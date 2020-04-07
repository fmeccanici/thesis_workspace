#!/usr/bin/env python

from trajectory_parser.trajectory_parser import *
# import trajectory_parser
from dynamic_time_warping.dynamic_time_warping import *
import numpy as np

class learnedToExecuted():
    def __init__(self, pred_traj, object_wrt_base):
        self.pred_traj = pred_traj
        self.object_wrt_base = object_wrt_base
        
    def ee_wrt_base(self, ee_wrt_object):
        return np.add(ee_wrt_object, self.object_wrt_base)
            

    def pred_traj_to_executed(self):
        traj_wrt_base = []

        for data in self.pred_traj:
            ee_wrt_base = list(self.ee_wrt_base(data[0:3]))
            ori = list(data[3:7])
            dt = [data[7]]
            object_wrt_base = list(self.object_wrt_base)
            traj_wrt_base.append(ee_wrt_base + ori + object_wrt_base + dt)

        return traj_wrt_base