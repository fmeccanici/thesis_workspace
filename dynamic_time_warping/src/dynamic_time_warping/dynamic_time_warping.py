#!/usr/bin/env python

from fastdtw import fastdtw
import numpy as np

from trajectory_parser.trajectory_parser import *
import matplotlib.pyplot as plt
import os, os.path

class DTW():
    def __init__(self):
        self.parser = trajectoryParser()


    def set_trajectories(self, demonstrations):
        self.trajectories = demonstrations

    # input structure list in list [[x_1, y_1, z_1]]
    @classmethod
    def apply_dtw(self, x, y):

        # check if we need to transpose the vectors
        # length of first array should be 11 (posx, posy, posz, etc...)
        # if this is too large then it should be transposed
        if len(x[0]) > 30:
            x = np.asarray(x).transpose()
            y = np.asarray(y).transpose()
        else:
            x = np.asarray(x)
            y = np.asarray(y)


        distance, path = fastdtw(x, y)

        x_aligned = []
        y_aligned = []

        for i in range(len(path)):
            x_aligned.append(x[path[i][0]])
            y_aligned.append(y[path[i][1]])

        return x_aligned, y_aligned

    # from Kyrarini et al.
    def determine_reference(self, demonstrations):
        similarity = np.zeros(len(demonstrations))
        for i in range(len(demonstrations)):
            for j in range(len(demonstrations)):
                distance, path = fastdtw(demonstrations[i], demonstrations[j])
                similarity[i] += distance
        
        return np.argmin(similarity)


    def align_necessary_trajectories(self, input_path, dt):
        traj_files = [name for name in os.listdir(input_path) if os.path.isfile(os.path.join(input_path, name))]
        num_traj = len(traj_files)
        trajectories, trajectories_lengths = self.parser.load_trajectories_from_folder_and_downsample(input_path, dt)
        traj_min_length = trajectories[np.argmin(trajectories_lengths)]
        
        del trajectories[np.argmin(trajectories_lengths)]
        traj_res = self.parser.resample_trajectories(trajectories, traj_min_length)
        traj_for_learning = []
        for traj in traj_res:
            traj_for_learning.append(self.parser.get_relevant_learning_data(traj))


        ix_for_dtw = self.parser.get_trajectories_ix_for_dtw(traj_for_learning)
        print("trajectories with same context: " + str(ix_for_dtw))
        ix_for_dtw_copy = ix_for_dtw
        
        # use DTW when needed
        if len(ix_for_dtw) > 0:

            counter = 1 

            # code to determine which trajectories we need to align
            # we need to align the trajectories with DTW that have similar context
            i = 0
            to_dtw = []
            to_dtw.append(ix_for_dtw_copy[0][0])
            to_dtw.append(ix_for_dtw_copy[0][1])
            del ix_for_dtw_copy[0]

            # we loop until we have had all possible combinations
            while len(ix_for_dtw_copy) > 0:
                
                # if either one of the trajectories was already covered
                # we know for sure that we need to add this one too
                # as both combinations exist
                if ix_for_dtw_copy[i][0] in to_dtw and ix_for_dtw_copy[i][1] not in to_dtw:
                    to_dtw.append(ix_for_dtw_copy[i][1])
                    ix_for_dtw_copy = ix_for_dtw

                    del ix_for_dtw_copy[i]
                elif ix_for_dtw_copy[i][1] in to_dtw and ix_for_dtw_copy[i][0] not in to_dtw:
                    to_dtw.append(ix_for_dtw_copy[i][0])
                    ix_for_dtw_copy = ix_for_dtw

                    del ix_for_dtw_copy[i]

                # if both are not in the to_dtw vector we need to add both of them
                elif ix_for_dtw_copy[i][1] not in to_dtw and ix_for_dtw_copy[i][0] not in to_dtw:
                    to_dtw.append(ix_for_dtw_copy[i][0])
                    to_dtw.append(ix_for_dtw_copy[i][1])

                    del ix_for_dtw_copy[i]

                # if both are already in the to_dtw we do nothing but delete this tuple
                elif ix_for_dtw_copy[i][1] in to_dtw and ix_for_dtw_copy[i][0] in to_dtw:
                    
                    del ix_for_dtw_copy[i]
            print("trajectories to be aligned: " + str(to_dtw))
            
            traj_to_dtw = []
            for i in range(len(to_dtw)):
                traj_to_dtw.append(traj_for_learning[to_dtw[i]])
            
            reference = dtw.determine_reference(traj_to_dtw)
            reference = to_dtw[reference]

            for i in range(len(to_dtw)):

                if to_dtw[i] != reference:

                    x,y = dtw.apply_dtw(traj_for_learning[reference], traj_for_learning[to_dtw[i]])
                    y = self.parser.arrays_in_list_to_list_in_list(y)
                    x = self.parser.arrays_in_list_to_list_in_list(x)

                    if counter == 1:
                        plt.subplot(211)
                        plt.title('Before DTW')
                        plt.xlabel('datapoint [-]')
                        plt.ylabel('position [m]')
                        plt.plot(self.parser.getCartesianPositions(traj_for_learning[reference]))
                        plt.plot(self.parser.getCartesianPositions(traj_for_learning[to_dtw[i]]))
                        plt.subplot(212)
                        plt.title('After DTW')
                        plt.xlabel('datapoint [-]')
                        plt.ylabel('position [m]')
                        plt.plot(self.parser.getCartesianPositions(x))
                        plt.plot(self.parser.getCartesianPositions(y))
                        counter += 1
                        # print(self.parser.getCartesianPositions(x)[0])
                    del traj_for_learning[to_dtw[i]]
                    del traj_for_learning[reference]

                    traj_for_learning.append(y)
                    traj_for_learning.append(x)
            plt.show()
        return traj_for_learning

    def store_resampled_aligned_trajectories(self, traj, output_path):
        for i in range(len(traj)):
            traj_file = open(output_path + "resampled_" + str(i) + ".txt", "w+")
            traj_file.write(str(traj[i]))
            traj_file.close()

if __name__ == "__main__":
    dtw = DTW()

    input_path = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_teaching/data/with_object2/'
    output_path = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_refinement/data/resampled/'
    dt = 0.01

    # traj_parsed = dtw.parse_to_relative_trajectory(input_path)

    traj_aligned_for_learning = dtw.align_necessary_trajectories(input_path, dt)
    for traj in traj_aligned_for_learning:
        print(len(traj))
    dtw.store_resampled_aligned_trajectories(traj_aligned_for_learning, output_path)

 

