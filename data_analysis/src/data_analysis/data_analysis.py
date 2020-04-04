#!/usr/bin/env python3
from trajectory_parser.trajectory_parser import *
import os, os.path
import pandas as pd
import numpy as np

class dataAnalysis():
    def __init__(self, data_path):
        self.data_path = data_path
        self.traj_files = [name for name in os.listdir(self.data_path) if os.path.isfile(os.path.join(self.data_path, name))]
        self.parser = trajectoryParser()

    def set_columns(self, columns=["joint_x", "joint_y", "joint_z", "qx", "qy", "qz", "qw",  "dt", "object_x", "object_y", "object_z"]):
        self.columns = columns
    
    def traj_files_to_csv(self, output_path = '/home/fmeccanici/Documents/thesis/lfd_ws/src/data_analysis/data/'):
        
        for traj in self.traj_files:
            trajectory = self.parser.openTrajectoryFile(traj, self.data_path)
            df = pd.DataFrame(trajectory)
            df.columns = self.columns
            df.to_csv(output_path + traj + '.csv')
            

if __name__ == "__main__":
    DIR = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_teaching/data/with_object_wrt_ee2/'

    col = ["EE_x", "EE_y", "EE_z", "EE_qx", "EE_qy","EE_qz", "EE_qw", "marker_x", "marker_y", "marker_z", "marker_qx", "marker_qy", "marker_qz", "marker_qw", "secs", "nsecs"]
    analys = dataAnalysis(DIR)
    analys.set_columns(col)
    analys.traj_files_to_csv()

