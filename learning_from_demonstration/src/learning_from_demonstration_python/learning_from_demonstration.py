#!/usr/bin/env python3.5

# import my own classes
# from learning_from_demonstration.promp_python import ProMPContext
from promp_context.promp_context import ProMPContext

from learning_from_demonstration_python.dynamic_time_warping import *
from learning_from_demonstration_python.trajectory_parser import *
from learning_from_demonstration_python.trajectory_resampler import *

# import other external classes
import ast, os, time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
from mpl_toolkits.mplot3d import Axes3D
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
        T = self.parser.get_total_time(traj)
        object_positions = traj[0][7:10]

        for data in traj:
            ee_pose = data[0:7]
            traj_relevant_data.append(ee_pose + object_positions + [T] )

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
    
    def prepare_single_raw(self, raw_traj, desired_datapoints):
        traj = self.convert_raw_to_correct_format(raw_traj)
        traj = self.parser.normalize_trajectory_time_float(traj)

        traj = self.resampler.interpolate_raw_trajectory(traj, desired_datapoints)
        
        traj = self.parse_relevant_learning_data(traj)
        traj = self.parser.get_trajectory_wrt_object(traj)
        traj = self.dtw.align_necessary_trajectories(traj)
    
        return traj
        
    def prepare_for_learning(self, desired_datapoints):
        
        print("Preparing raw trajectories for learning...")
        
        plt.figure(figsize=[20,5])
        plt.title("Demonstrations")
        # ax = fig.gca(projection='3d')
        # plt.subplot(1,3,1)
        # plt.grid()
        # plt.subplot(1,3,2)
        # plt.grid()
        # plt.subplot(1,3,3)
        # plt.grid()
        
        
        # normalize time
        print("Normalizing trajectories wrt time...")
        for traj in self.raw_trajectories:
        
            x = []
            y = []
            z = []
            qx = []
            qy = []
            qz = []
            qw = []

            # due to pointers, this changes the raw trajectories
            traj = self.parser.normalize_trajectory_time_float(traj)
            
            for data in traj:
                x.append(data[0])
                y.append(data[1])
                z.append(data[2])
                qx.append(data[3])
                qy.append(data[4])
                qz.append(data[5])
                qw.append(data[6])

        #     plt.subplot(1,7,1)
        #     plt.plot(x) 

        #     plt.subplot(1,7,2)
        #     plt.plot(y) 
            
        #     plt.subplot(1,7,3)
        #     plt.plot(z) 
            
        #     plt.subplot(1,7,4)
        #     plt.plot(qx)        

        #     plt.subplot(1,7,5)
        #     plt.plot(qy) 

        #     plt.subplot(1,7,6)
        #     plt.plot(qz) 

        #     plt.subplot(1,7,7)
        #     plt.plot(qw) 

        # plt.subplot(1,7,1)
        # plt.title("Cartesian x")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("position [m]")
        # plt.grid()
        
        # plt.subplot(1,7,2)
        # plt.title("Cartesian y")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("position [m]")
        # plt.grid()

        # plt.subplot(1,7,3)
        # plt.title("Cartesian z")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("position [m]")
        # plt.grid()

        # plt.subplot(1,7,4)
        # plt.title("Quaternion x")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("orientation [-]")
        # plt.grid()

        # plt.subplot(1,7,5)
        # plt.title("Quaternion y")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("orientation [-]")
        # plt.grid()

        # plt.subplot(1,7,6)
        # plt.title("Quaternion z")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("orientation [-]")
        # plt.grid()

        # plt.subplot(1,7,7)
        # plt.title("Quaternion w")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("orientation [-]")
        # plt.grid()

        # plt.tight_layout()
        # plt.savefig('/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/figures/raw_demonstrations.png')
        # plt.clf()
               
        # resample trajectories
        print("Resample trajectories to all have " + str(desired_datapoints) + " datapoints...")
        resampled_trajectories = []
        for traj in self.raw_trajectories:
            resampled_trajectories.append(self.resampler.interpolate_raw_trajectory(traj, desired_datapoints))
        
        plt.figure(figsize=[20,5])

        for traj in resampled_trajectories:
            x = []
            y = []
            z = []
            qx = []
            qy = []
            qz = []
            qw = []

            for data in traj:
                x.append(data[0])
                y.append(data[1])
                z.append(data[2])
                qx.append(data[3])
                qy.append(data[4])
                qz.append(data[5])
                qw.append(data[6])
        #     plt.subplot(1,7,1)
        #     plt.plot(x) 

        #     plt.subplot(1,7,2)
        #     plt.plot(y) 
            
        #     plt.subplot(1,7,3)
        #     plt.plot(z) 
            
        #     plt.subplot(1,7,4)
        #     plt.plot(qx)        

        #     plt.subplot(1,7,5)
        #     plt.plot(qy) 

        #     plt.subplot(1,7,6)
        #     plt.plot(qz) 

        #     plt.subplot(1,7,7)
        #     plt.plot(qw) 

        # plt.subplot(1,7,1)
        # plt.title("Cartesian x")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("position [m]")
        # plt.grid()
        
        # plt.subplot(1,7,2)
        # plt.title("Cartesian y")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("position [m]")
        # plt.grid()

        # plt.subplot(1,7,3)
        # plt.title("Cartesian z")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("position [m]")
        # plt.grid()

        # plt.subplot(1,7,4)
        # plt.title("Quaternion x")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("orientation [-]")
        # plt.grid()

        # plt.subplot(1,7,5)
        # plt.title("Quaternion y")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("orientation [-]")
        # plt.grid()

        # plt.subplot(1,7,6)
        # plt.title("Quaternion z")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("orientation [-]")
        # plt.grid()

        # plt.subplot(1,7,7)
        # plt.title("Quaternion w")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("orientation [-]")
        # plt.grid()

        # plt.tight_layout()
        # plt.savefig('/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/figures/resampled_trajectories.png')

        # plt.close()

        # get relevant learning data
        print("Extracting relevant learning data: [ee_x, ee_y, ee_z, ee_qx, ee_qy, ee_qz, obj_x, obj_y, obj_z, T]...")
        relevant_data_trajectories = []
        for traj in resampled_trajectories:
            relevant_traj = self.parse_relevant_learning_data(traj)
            relevant_data_trajectories.append(relevant_traj)
        
        # plt.figure()
        # for traj in relevant_data_trajectories:
            # plt.plot([x[0:3] for x in traj])
            # plt.plot([x[0] for x in traj], label="context=" + str([round(traj[0][7]*10, 2), round(traj[0][8]*10, 2), round(traj[0][9]*10, 2)] ))
            # plt.plot([x[1] for x in traj], label='y')
            # plt.plot([x[2] for x in traj], label='z')

            # plt.plot([x[7:10] for x in traj])
        #     plt.xlabel("datapoints [-]")
        #     plt.ylabel("position [m]")
        #     plt.title("Relevant data trajectories")
        #     plt.legend()
        # plt.grid()
        # plt.savefig('/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/figures/relevant_data_trajectories.png')
        # plt.close()
        
        # convert trajectory to relative trajectory wrt ee
        # and change object pose wrt base to wrt ee
        print("Converting to relative trajectory...")

        relative_trajectories = []
        for traj in relevant_data_trajectories:
        # for traj in relevant_data_trajectories:
            traj_wrt_object = self.parser.get_trajectory_wrt_object(traj)
            
            relative_trajectories.append(traj_wrt_object)
            # self.trajectories_for_learning.append(traj)

        plt.figure(figsize=[20,5])
        for traj in relative_trajectories:
            x = []
            y = []
            z = []
            qx = []
            qy = []
            qz = []
            qw = []

            for data in traj:
                x.append(data[0])
                y.append(data[1])
                z.append(data[2])
                qx.append(data[3])
                qy.append(data[4])
                qz.append(data[5])
                qw.append(data[6])
        #     plt.subplot(1,7,1)
        #     plt.plot(x) 

        #     plt.subplot(1,7,2)
        #     plt.plot(y) 
            
        #     plt.subplot(1,7,3)
        #     plt.plot(z) 
            
        #     plt.subplot(1,7,4)
        #     plt.plot(qx)        

        #     plt.subplot(1,7,5)
        #     plt.plot(qy) 

        #     plt.subplot(1,7,6)
        #     plt.plot(qz) 

        #     plt.subplot(1,7,7)
        #     plt.plot(qw) 

        # plt.subplot(1,7,1)
        # plt.title("Cartesian x")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("position [m]")
        # plt.grid()
        
        # plt.subplot(1,7,2)
        # plt.title("Cartesian y")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("position [m]")
        # plt.grid()

        # plt.subplot(1,7,3)
        # plt.title("Cartesian z")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("position [m]")
        # plt.grid()

        # plt.subplot(1,7,4)
        # plt.title("Quaternion x")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("orientation [-]")
        # plt.grid()

        # plt.subplot(1,7,5)
        # plt.title("Quaternion y")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("orientation [-]")
        # plt.grid()

        # plt.subplot(1,7,6)
        # plt.title("Quaternion z")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("orientation [-]")
        # plt.grid()

        # plt.subplot(1,7,7)
        # plt.title("Quaternion w")
        # plt.xlabel("datapoint [-]")
        # plt.ylabel("orientation [-]")
        # plt.grid()
        # plt.tight_layout()
        # plt.savefig('/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/figures/relative_trajectories.png')
        # plt.close()

        # apply dynamic time warping
        print("Applying DTW...")
        self.trajectories_for_learning = self.dtw.align_necessary_trajectories(relative_trajectories)

        # print(len(traj_aligned_for_learning))
        plt.figure(figsize=[20,5])
        for traj in self.trajectories_for_learning:
            x = []
            y = []
            z = []
            qx = []
            qy = []
            qz = []
            qw = []

            for data in traj:
                x.append(data[0])
                y.append(data[1])
                z.append(data[2])
                qx.append(data[3])
                qy.append(data[4])
                qz.append(data[5])
                qw.append(data[6])
            plt.subplot(1,7,1)
            plt.plot(x) 

            plt.subplot(1,7,2)
            plt.plot(y) 
            
            plt.subplot(1,7,3)
            plt.plot(z) 
            
            plt.subplot(1,7,4)
            plt.plot(qx)        

            plt.subplot(1,7,5)
            plt.plot(qy) 

            plt.subplot(1,7,6)
            plt.plot(qz) 

            plt.subplot(1,7,7)
            plt.plot(qw) 

        plt.subplot(1,7,1)
        plt.title("Cartesian x")
        plt.xlabel("datapoint [-]")
        plt.ylabel("position [m]")
        plt.grid()
        
        plt.subplot(1,7,2)
        plt.title("Cartesian y")
        plt.xlabel("datapoint [-]")
        plt.ylabel("position [m]")
        plt.grid()

        plt.subplot(1,7,3)
        plt.title("Cartesian z")
        plt.xlabel("datapoint [-]")
        plt.ylabel("position [m]")
        plt.grid()

        plt.subplot(1,7,4)
        plt.title("Quaternion x")
        plt.xlabel("datapoint [-]")
        plt.ylabel("orientation [-]")
        plt.grid()

        plt.subplot(1,7,5)
        plt.title("Quaternion y")
        plt.xlabel("datapoint [-]")
        plt.ylabel("orientation [-]")
        plt.grid()

        plt.subplot(1,7,6)
        plt.title("Quaternion z")
        plt.xlabel("datapoint [-]")
        plt.ylabel("orientation [-]")
        plt.grid()

        plt.subplot(1,7,7)
        plt.title("Quaternion w")
        plt.xlabel("datapoint [-]")
        plt.ylabel("orientation [-]")
        plt.tight_layout()
        plt.grid()
        plt.savefig('/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/figures/trajectories_for_learning.png')
        plt.close()

    def build_initial_promp_model(self):
        # in promp package, input and output are all called joints
        # if name is different, then it won't plot them
        self.outputs = ["ee_x", "ee_y","ee_z", "ee_qx", "ee_qy", "ee_qz", "ee_qw", "T" ]
        self.contexts = ["object_x", "object_y", "object_z"]
        # self.contexts = ["object_y"]

        self.variables = ["ee_x", "ee_y","ee_z", "ee_qx", "ee_qy", "ee_qz", "ee_qw", "object_x", "object_y", "object_z", "T" ]

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
        print('Amount of demonstrations = ' + str(len(self.trajectories_for_learning)))
        for i,traj in enumerate(self.trajectories_for_learning):
            # print(traj[0])
            # print("context = " + str(traj[0][7:10]))

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
            
            # if variable is dt --> convert to time vector
            if promp.output_name[0] == 'T':
                # dt = pred[0]
                n = len(pred)
                # T = dt*n

                # if this produces negative results probably the marker is not detected properly
                # this also produces a weird trajectory
                T = np.abs(pred[0])
                # print("T = " + str(T))
                t = np.linspace(0, T, n)
                pred_traj = np.vstack((pred_traj, t))
            else:
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

    def welford_update(self, traj, context):
        # we need to extract each separate variable
        # and add this to the individual promp models
        rospy.loginfo("Welford update...")

        for variable in range(np.asarray(traj).shape[1]):
            single_variable_traj = [ [data[variable]] for data in traj]
            demonstration = (single_variable_traj, context)
            self.promps[variable].welford_update(demonstration, 1)


    def add_trajectory_to_promp_model(self, traj, context):
        # we need to extract each separate variable
        # and add this to the individual promp models
        rospy.loginfo("Adding trajectory to promp model...")

        for variable in range(np.asarray(traj).shape[1]):
            single_variable_traj = [ [data[variable]] for data in traj]
            demonstration = (single_variable_traj, context)
            self.promps[variable].add_demonstration(demonstration)

    def get_raw_trajectories(self):
        return self.raw_trajectories

    def get_trajectories_for_learning(self):
        return self.trajectories_for_learning

if __name__ == "__main__":
    lfd = learningFromDemonstration()
    raw_path = "/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/data/raw/one_plane/"

    lfd.load_trajectories_from_folder(raw_path)

    desired_datapoints = 10
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
