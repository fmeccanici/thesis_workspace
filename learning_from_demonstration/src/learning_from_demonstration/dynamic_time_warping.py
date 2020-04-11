#!/usr/bin/env python3.5

from fastdtw import fastdtw
import numpy as np

from learning_from_demonstration.trajectory_parser import trajectoryParser
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
        
        similarity = list(similarity)
        reference = similarity.index(sorted(similarity)[0])

        most_similar_to_reference = similarity.index(sorted(similarity)[1])

        return reference, most_similar_to_reference

    def get_difference_in_context(self, traj1, traj2):
        c1 = self.parser.get_context(traj1)
        c2 = self.parser.get_context(traj2)

        dx = abs(c1[0] - c2[0])
        dy = abs(c1[1] - c2[1])
        dz = abs(c1[2] - c2[2])

        return dx, dy, dz

    def get_dcontext_matrices(self, demonstrations):
        # function that determines the total difference in context
        # context matrix is the context wrt other trajectories
        
        difference_matrix_x = np.zeros((len(demonstrations), len(demonstrations)))
        difference_matrix_y = np.zeros((len(demonstrations), len(demonstrations)))
        difference_matrix_z = np.zeros((len(demonstrations), len(demonstrations)))

        difference_matrix_total = np.zeros((len(demonstrations), len(demonstrations)))

        for i in range(len(demonstrations)):
            for j in range(len(demonstrations)):
                dx, dy, dz = self.get_difference_in_context(demonstrations[i][0], demonstrations[j][0])

                difference_matrix_x[i,j] = dx
                difference_matrix_y[i,j] = dy
                difference_matrix_z[i,j] = dz

                difference_matrix_total[i,j] = dx + dy + dz

        return difference_matrix_x, difference_matrix_y, difference_matrix_z, difference_matrix_total

    def get_trajectories_ix_for_dtw(self, demonstrations):
        # code to determine which trajectories we need to align
        # we need to align the trajectories with DTW that have similar context

        dx, dy, dz, dtotal = self.get_dcontext_matrices(demonstrations)
        traj_to_dtw = []
        appended_ix = []

        threshold = 0.05


        for i in range(len(np.where(dtotal<threshold)[0])):
            # np.where(dtotal<threshold) contains tuple of arrays that are the indices of the matrix that are lower than the treshold
            # e.g. ( [0,0,0], [0,1,2] ) which means that (0,1), (0,2) and (0,3) trajectories have the same context

            # if the index is (n,n) we are dealing with the same demonstrations, hence this has obviously the same context --> these are excluded
            # and if the included tuples are not stored yet we add them
            if np.where(dtotal<threshold)[0][i] != np.where(dtotal<threshold)[1][i] and (np.where(dtotal<threshold)[0][i], np.where(dtotal<threshold)[1][i]) not in appended_ix:

                # append tuple
                appended_ix.append((np.where(dtotal<threshold)[0][i], np.where(dtotal<threshold)[1][i]))

            else: pass

        return appended_ix

    def align_necessary_trajectories(self, traj_for_learning):

        # initialize matrix that is filled with aligned trajectories wrt context
        align_matrix = []

        # get the tuples that have the same context
        ix_for_dtw = self.get_trajectories_ix_for_dtw(traj_for_learning)

        # makea copy of this used for the logic
        ix_for_dtw_copy = ix_for_dtw[:]
        
        # # use DTW when needed
        # if len(ix_for_dtw) > 0:

        counter = 1 

        i = 0
        to_dtw = []
        
        # add the first trajectories from the tuple to the vector
        to_dtw.append(ix_for_dtw_copy[0][0])
        to_dtw.append(ix_for_dtw_copy[0][1])

        # delete the first tuple
        del ix_for_dtw_copy[0]

        # variable used to check if we have covered every trajectory of a specific context
        i_prev = 0

        # we loop until we have had all possible combinations
        while True:
            # print(ix_for_dtw_copy)
            # if we are finished
            if len(ix_for_dtw_copy) == 0:
                align_matrix.append(to_dtw)
                break

            # if either one of the trajectories was already covered
            # we know for sure that we need to add this one too
            # as both combinations exist
            try:
                if ix_for_dtw_copy[i][0] in to_dtw and ix_for_dtw_copy[i][1] not in to_dtw:
                    to_dtw.append(ix_for_dtw_copy[i][1])
                    
                    # print(len(ix_for_dtw))
                    # print(len(ix_for_dtw_copy[0]))
                    # ix_for_dtw_copy = ix_for_dtw

                    del ix_for_dtw_copy[i]

                    i = 0

                elif ix_for_dtw_copy[i][1] in to_dtw and ix_for_dtw_copy[i][0] not in to_dtw:
                    to_dtw.append(ix_for_dtw_copy[i][0])
                    # ix_for_dtw_copy = ix_for_dtw
                    
                    del ix_for_dtw_copy[i]

                    i = 0

                elif ix_for_dtw_copy[i][1] not in to_dtw and ix_for_dtw_copy[i][0] not in to_dtw:
                    # to_dtw.append(ix_for_dtw_copy[i][0])
                    # to_dtw.append(ix_for_dtw_copy[i][1])
                    i += 1

                    # del ix_for_dtw_copy[i]

                # if both are already in the to_dtw we do nothing but delete this tuple
                elif ix_for_dtw_copy[i][1] in to_dtw and ix_for_dtw_copy[i][0] in to_dtw:
                    
                    del ix_for_dtw_copy[i]

                    i = 0

            except IndexError:
                print('check')
                # if index error occurs we are sure that we covered the trajctories of a context
                # add these trajectories to the matrix
                align_matrix.append(to_dtw)

                # empty the to_dtw vector
                to_dtw = []

                # add the first trajectories from the tuple to the vector
                to_dtw.append(ix_for_dtw_copy[0][0])
                to_dtw.append(ix_for_dtw_copy[0][1])

                # start over again but now create a new context row
                i = 0
            
            
        print("Trajectories to be aligned: " + str(align_matrix))
        
        traj_aligned = []

        # loop over the contexts
        for same_context in align_matrix:
            traj_to_dtw = []
            
            # add trajectories
            for index in same_context:
                traj_to_dtw.append(traj_for_learning[index])

            # determine reference trajectory
            reference_index, most_similar_to_reference = self.determine_reference(traj_to_dtw)
            reference = traj_for_learning[same_context[reference_index]]

            # apply dtw
            for index in same_context:
                print(index)
                print("len ref = " + str(len(reference)))
                print("len traj = " + str(len(traj_for_learning[index])))

                if index != reference_index and index == most_similar_to_reference:
                    x,y = self.apply_dtw(reference, traj_for_learning[index])
                    
                    print("reference length = " + str(len(x)))
                    print("traj length = " + str(len(y)))

                    # x is the reference
                    y = self.parser.arrays_in_list_to_list_in_list(y)
                    x = self.parser.arrays_in_list_to_list_in_list(x)
                    


        # for i in range(len(to_dtw) > 0):

        #     if to_dtw[i] != reference:

        #         x,y = self.apply_dtw(traj_for_learning[reference], traj_for_learning[to_dtw[i]])
        #         y = self.parser.arrays_in_list_to_list_in_list(y)
        #         x = self.parser.arrays_in_list_to_list_in_list(x)

        #         if counter == 1:
        #             plt.subplot(211)
        #             plt.title('Before DTW')
        #             plt.xlabel('datapoint [-]')
        #             plt.ylabel('position [m]')
        #             plt.plot(self.parser.getCartesianPositions(traj_for_learning[reference]))
        #             plt.plot(self.parser.getCartesianPositions(traj_for_learning[to_dtw[i]]))
        #             plt.subplot(212)
        #             plt.title('After DTW')
        #             plt.xlabel('datapoint [-]')
        #             plt.ylabel('position [m]')
        #             plt.plot(self.parser.getCartesianPositions(x))
        #             plt.plot(self.parser.getCartesianPositions(y))
        #             counter += 1
        #             # print(self.parser.getCartesianPositions(x)[0])
        #         del traj_for_learning[to_dtw[i]]
        #         del traj_for_learning[reference]

        #         traj_for_learning.append(y)
        #         traj_for_learning.append(x)
        # plt.show()
        return traj_for_learning



if __name__ == "__main__":
    dtw = DTW()

    input_path = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_teaching/data/both_wrt_base/'
    output_path = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_refinement/data/resampled/'
    dt = 0.1

    # traj_parsed = dtw.parse_to_relative_trajectory(input_path)

    traj_aligned_for_learning = dtw.align_necessary_trajectories(input_path, dt)
    for traj in traj_aligned_for_learning:
        print(len(traj))
    # dtw.store_trajectories(traj_aligned_for_learning, output_path)

 

