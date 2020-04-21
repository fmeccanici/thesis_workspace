#!/usr/bin/env python3.5

# import my own classes
# from learning_from_demonstration.promp_python import ProMPContext
from promp_context.promp_context import ProMPContext

from learning_from_demonstration.dynamic_time_warping import *
from learning_from_demonstration.trajectory_parser import *
from learning_from_demonstration.trajectory_resampler import *

# import other external classes
import ast, os, time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

class learningFromDemonstration():
    def __init__(self):
        # raw trajectory layout: 
        # [ee_x, ee_y, ee_z, ee_qx, ee_qy, ee_qz, ee_qw,
        #  pos_x, pos_y, pos_z, pos_qx, pos_qy, pos_qz, pos_qw,
        #  time_as_float]
        
        self.raw_trajectories = []
        self.trajectories_for_learning = []

        self.parser = trajectoryParser()
        self.resampler = trajectoryResampler()
        self.dtw = DTW()

    def load_trajectories_from_folder(self, path):
        traj_files = [name for name in os.listdir(path) if os.path.isfile(os.path.join(path, name))]
        
        # get numbers from these files
        numbers = [int(os.path.splitext(f)[0].split('_')[-1]) for f in traj_files]
        
        # sort them in ascending order
        numbers.sort()

        # make list of these files
        traj_files = ["raw_trajectory_" + str(number) + ".txt" for number in numbers]

        for traj_file in traj_files:
            self.load_trajectory_from_folder(path, traj_file)
        
    def load_trajectory_from_folder(self, path, traj_file):
        with open(path+traj_file, "r") as traj:
            raw_trajectory = ast.literal_eval(traj.read())    
            self.add_raw_trajectory(raw_trajectory)

    def add_raw_trajectory(self, raw_traj):
        
        # check if in correct format
        if not self.is_correct_raw_format(raw_traj):
            raw_traj = self.convert_raw_to_correct_format(raw_traj)
            # print(raw_traj)
        else:
            pass

        self.raw_trajectories.append(raw_traj)

    def is_correct_raw_format(self, raw_traj):
        if len(raw_traj[0]) == 15:
            return True
        else:
            return False
    
    def parse_relevant_learning_data(self, traj):
        traj_relevant_data = []
        dt = self.parser.get_time_interval_float(traj)
        object_positions = traj[0][7:10]

        # T doesnt work properly --> chose dt as output
        for data in traj:
            ee_pose = data[0:7]
            traj_relevant_data.append(ee_pose + object_positions + [dt] )

        return traj_relevant_data

    def convert_raw_to_correct_format(self, raw_traj):
        if self.is_correct_raw_format(raw_traj):
            print("Raw trajectory already in correct format")
        else:
            try:
                # check if time format is incorrect (secs, nsecs) instead of float
                if isinstance(raw_traj[0][14], int):
                    # time format is (secs, nsecs)
                    print("Raw trajectory contains secs/nsecs values")
                    print("Converting to float...")
                    return self.parser.convert_raw_secs_nsecs_to_float(raw_traj)
                else:
                    print("No secs/nsecs value detected")
            except IndexError:
                print("Raw trajectory has incorrect length, check if it contains essential paramaters")

    def prepare_for_learning(self, desired_datapoints):
        print("Preparing raw trajectories for learning...")
        
        # normalize time
        print("Normalizing trajectories wrt time...")
        for traj in self.raw_trajectories:
            traj = self.parser.normalize_trajectory_time_float(traj)

        # resample trajectories
        print("Resample trajectories to all have " + str(desired_datapoints) + " datapoints...")
        resampled_trajectories = []
        for traj in self.raw_trajectories:
            resampled_trajectories.append(self.resampler.interpolate_raw_trajectory(traj, desired_datapoints))
        plt.figure()
        for traj in resampled_trajectories:
            plt.plot([x[0:3] for x in traj])
            # plt.plot([x[7:10] for x in traj])
            plt.xlabel("datapoints [-]")
            plt.ylabel("position [m]")
            plt.title("Resampled trajectories")
        plt.grid()
        plt.show()

        # get relevant learning data
        print("Extracting relevant learning data: [ee_x, ee_y, ee_z, ee_qx, ee_qy, ee_qz, obj_x, obj_y, obj_z, dt]...")
        relevant_data_trajectories = []
        for traj in resampled_trajectories:
            relevant_traj = self.parse_relevant_learning_data(traj)
            relevant_data_trajectories.append(relevant_traj)
            # print(traj[0][7:10])
        

        # apply dynamic time warping
        print("Applying DTW...")
        traj_aligned_for_learning = self.dtw.align_necessary_trajectories(relevant_data_trajectories)

        # print(len(traj_aligned_for_learning))
        plt.figure()
        for traj in traj_aligned_for_learning:
            plt.plot([x[0:3] for x in traj])
            # plt.plot([x[0] for x in traj], label="context=" + str([round(traj[0][7]*10, 2), round(traj[0][8]*10, 2), round(traj[0][9]*10, 2)] ))
            # plt.plot([x[1] for x in traj], label='y')
            # plt.plot([x[2] for x in traj], label='z')

            # plt.plot([x[7:10] for x in traj])
            plt.xlabel("datapoints [-]")
            plt.ylabel("position [m]")
            plt.title("Aligned trajectories")
            plt.legend()
        plt.grid()
        plt.show()


        # convert trajectory to relative trajectory wrt ee
        # and change object pose wrt base to wrt ee
        print("Converting to relative trajectory...")
        for traj in traj_aligned_for_learning:
        # for traj in relevant_data_trajectories:
            traj_wrt_object = self.parser.get_trajectory_wrt_object(traj)
            
            self.trajectories_for_learning.append(traj_wrt_object)
            # self.trajectories_for_learning.append(traj)

        print(self.trajectories_for_learning[0][0])

    def build_initial_promp_model(self):
        # in promp package, input and output are all called joints
        # if name is different, then it won't plot them
        self.outputs = ["ee_x", "ee_y","ee_z", "ee_qx", "ee_qy", "ee_qz", "ee_qw", "dt" ]
        self.contexts = ["object_x", "object_y", "object_z"]
        # self.contexts = ["object_y"]

        self.variables = ["ee_x", "ee_y","ee_z", "ee_qx", "ee_qy", "ee_qz", "ee_qw", "object_x", "object_y", "object_z", "dt" ]

        # how to select the number of points??
        # if other trajectories are added to the model
        # this will probably be wrongly compared to the existing
        # model, since the trajs will be squeezed/stretched
        self.num_samples = len(self.trajectories_for_learning[0])
        
        # default value
        num_basis = 10

        # default value
        sigma = 0.1

        # for each output variable create a ProMP
        self.promps = [ProMPContext(output, self.contexts, num_samples=self.num_samples, num_basis=num_basis, sigma=sigma) for output in self.outputs]
        
        print('Adding trajectories to ProMP model...')

        for i,traj in enumerate(self.trajectories_for_learning):
            print("context = " + str(traj[0][7:10]))

            context = [round(traj[0][7]*10, 2), round(traj[0][8]*10, 2), round(traj[0][9]*10, 2)]
            # context = [round(traj[0][8], 2)]

            for idx, name in enumerate(self.variables):
                # check if name is output variable
                if name in self.outputs:

                    output_idx = self.outputs.index(name)
                    traj_one_output = [ [data[idx]] for data in traj]

                    demonstration = (traj_one_output, context)

                    # add relevant trajectories to promp
                    self.promps[output_idx].add_demonstration(demonstration)
                
                else: pass
            
    def generalize(self, context):
        # generate trajectory using ProMP package
        
        pred_traj = self.promps[0].generate_trajectory(context)     
        for promp in self.promps[1:]:
            pred = promp.generate_trajectory(context)
            pred_traj = np.vstack((pred_traj, pred))

        # list format
        pred_traj = [list(x) for x in pred_traj.T]
        
        return pred_traj

    def trajectory_wrt_base(self, trajectory_wrt_object, object_wrt_base):
        # calculating trajectory wrt base instead of wrt object --> needed for execution
        traj_wrt_base = []

        for data in trajectory_wrt_object:
            ee_wrt_base = list(self.parser.ee_wrt_base(data[0:3], object_wrt_base))
            ori = list(data[3:7])
            t = [data[-1]]
            object_wrt_base = list(object_wrt_base)
            traj_wrt_base.append(ee_wrt_base + ori + object_wrt_base + t)

        return traj_wrt_base

    def add_trajectory_to_promp_model(self, traj):
        print("Adding trajectory to promp model...")
        self.promp_model.add_demonstration(np.asarray(traj))

    def get_raw_trajectories(self):
        return self.raw_trajectories

    def get_trajectories_for_learning(self):
        return self.trajectories_for_learning

if __name__ == "__main__":
    lfd = learningFromDemonstration()
    raw_path = "/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/data/raw/one_plane/"

    lfd.load_trajectories_from_folder(raw_path)

    desired_datapoints = 100
    lfd.prepare_for_learning(desired_datapoints)
    lfd.build_initial_promp_model()

    # object_wrt_base = [0.24, 0.85, 0.68]
    object_wrt_base1 = [round(0*10,2), round(0.78*10, 2), round(0.68*10,2)]

    prediction1 = lfd.generalize(object_wrt_base1)

    object_wrt_base2 = [round(0*10,2), round(0.94*10, 2), round(0.68*10,2)]
    # object_wrt_base = [0.94]

    prediction2 = lfd.generalize(object_wrt_base2)

    # ee_wrt_base = [0.41, -0.42, 1.14]
    # object_wrt_ee = lfd.parser.object_wrt_ee(ee_wrt_base, object_wrt_base)

    plt.figure()
    plt.plot([x[0] for x in prediction1 ], label="context=" + str(object_wrt_base1))
    plt.plot([x[0] for x in prediction2 ], label="context=" + str(object_wrt_base2))

    # plt.plot([x[1] for x in prediction1 ])
    # plt.plot([x[2] for x in prediction1 ])
    plt.grid()
    plt.title("Prediction")
    plt.legend()
    plt.show()
    # print("prediction = " + str(prediction))
    # trajectory_wrt_base = lfd.trajectory_wrt_base(prediction, object_wrt_base)
