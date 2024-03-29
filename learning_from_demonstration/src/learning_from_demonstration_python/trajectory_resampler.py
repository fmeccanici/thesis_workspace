#!/usr/bin/env python3.5
from learning_from_demonstration_python.trajectory_parser import *

import numpy as np
from scipy.interpolate import interp1d, InterpolatedUnivariateSpline
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

class trajectoryResampler():
    def __init__(self):
        self.parser = trajectoryParser()

    def interpolate_quaternions(self, qstart, qend, n, include_endpoints=True):
        q1 = Quaternion(a=qstart[3], b=qstart[0], c=qstart[1], d=qstart[2])
        q2 = Quaternion(a=qend[3], b=qend[0], c=qend[1], d=qend[2])

        return Quaternion.intermediates(q1, q2, n, include_endpoints=include_endpoints)
    
    def resample_time(self, traj, T):
        n = len(traj)
        t = np.linspace(0, T, n)

        for i in range(len(traj)):
            traj[i][-1] = t[i]
        
        return traj

    def interpolate_learned_keypoints(self, pred_traj, n_desired):
        T_desired = 20

        n = len(pred_traj)
        T = pred_traj[-1][-1]
        
        # in experiment this somehow happened
        if T < 0:
            print("T is negative: " + str(T))
            T = T_desired

        x = np.linspace(0, T, n)
        print("STRICTLY INCREASING DEBUG")
        print("n = " + str(n))
        print("ndesired = " + str(n_desired))

        print("T = " + str(T))

        # print("n = " + str(n))
        # print("x = " + str(x))
        # print("T = " + str(T))
        # print("pred_traj = " + str(pred_traj[-1]))

        object_pose = pred_traj[0][7:10]
        cartx = [data[0] for data in pred_traj]
        carty = [data[1] for data in pred_traj]
        cartz = [data[2] for data in pred_traj]
        q = [ [data[3], data[4], data[5], data[6]] for data in pred_traj]
        
        splinex = InterpolatedUnivariateSpline(x, cartx)
        spliney = InterpolatedUnivariateSpline(x, carty)
        splinez = InterpolatedUnivariateSpline(x, cartz)

        xdesired = np.linspace(0, T, n_desired)
        dt_new = T / n_desired

        cartx_new = splinex(xdesired)
        carty_new = spliney(xdesired)
        cartz_new = splinez(xdesired)

        qstart = pred_traj[0][3:7]
        qend = pred_traj[-1][3:7]

        slerp = Slerp(x, R.from_quat(q))
        interp_rots = slerp(xdesired)

        interpol_pred_traj = []
        for i,q in enumerate(self.interpolate_quaternions(qstart, qend, n_desired, False)):
            pose = [cartx_new[i], carty_new[i], cartz_new[i], q[1], q[2], q[3], q[0]]
            ynew = pose + [xdesired[i]]

        for i, data in enumerate(cartx_new):
            pos = [cartx_new[i], carty_new[i], cartz_new[i]]
            ori = list(interp_rots[i].as_quat())
            t = [xdesired[i]]
            demo = pos + ori + t
            interpol_pred_traj.append( demo )

        return interpol_pred_traj
    
    def isStrictlyIncreasing(self, t0, t1):
        return t1 > t0

    def makeStrictlyIncreasing(self, raw_traj):
        traj_time = self.parser.get_time_vector_float(raw_traj)
        res = []

        for i in range(len(traj_time)):
            try:
                if self.isStrictlyIncreasing(traj_time[i], traj_time[i+1]):
                    res.append(raw_traj[i])
            except IndexError:
                break
        
        return res
            
    def interpolate_raw_trajectory(self, raw_traj, n):

        raw_traj = self.makeStrictlyIncreasing(raw_traj)

        traj_pose = self.parser.getCartesianPositions(raw_traj)
        traj_time = self.parser.get_time_vector_float(raw_traj)

        T = self.parser.get_total_time(raw_traj)

        object_pose = self.parser.get_object_pose(raw_traj)

        xvals = np.linspace(0, T, n)

        traj_pos_x = (np.asarray(self.parser.getXpositions(traj_pose)).reshape(len(traj_time), 1))
        traj_pos_y = (np.asarray(self.parser.getYpositions(traj_pose)).reshape(len(traj_time), 1))
        traj_pos_z = (np.asarray(self.parser.getZpositions(traj_pose)).reshape(len(traj_time), 1))
        q = [ [data[3], data[4], data[5], data[6]] for data in raw_traj]

        yinterp_x = interp1d((traj_time), np.transpose(traj_pos_x), axis=1, fill_value="extrapolate")
        yinterp_y = interp1d((traj_time), np.transpose(traj_pos_y), axis=1, fill_value="extrapolate")
        yinterp_z = interp1d((traj_time), np.transpose(traj_pos_z), axis=1, fill_value="extrapolate")

        y_new_x = yinterp_x(xvals)
        y_new_y = yinterp_y(xvals)
        y_new_z = yinterp_z(xvals)

        qstart = raw_traj[0][3:7]
        qend = raw_traj[-1][3:7]
        
        # using zip() + all() 
        # to check for strictly increasing list 
        res = all(i < j for i, j in zip([round(x,3) for x in traj_time], [round(x,3) for x in traj_time][1:])) 

        # printing result 
        print ("Is list strictly increasing ? : " + str(res)) 

        slerp = Slerp(traj_time, R.from_quat(q))
        interp_rots = slerp(xvals)

        interpol_traj = []

        for i, data in enumerate(y_new_x[0]):
            pos = [y_new_x[0][i], y_new_y[0][i], y_new_z[0][i]]
            ori = list(interp_rots[i].as_quat()) 
            t = [xvals[i]]
            demo = pos + ori + object_pose + t
            # print(pos)
            # print(ori)
            # print(t)
            # print()
            # print(demo)
            # print()
            interpol_traj.append( demo )

        return interpol_traj

    def interpolate_predicted_trajectory(self, traj, n):
        traj_pos = self.parser.getCartesianPositions(traj)
        traj_time = self.parser.get_time_vector_float(traj)
        T = self.parser.get_total_time(traj)

        object_pos = self.parser.get_object_position(traj)

        xvals = np.linspace(0, T, n)

        traj_pos_x = (np.asarray(self.parser.getXpositions(traj_pos)).reshape(len(traj_time), 1))
        traj_pos_y = (np.asarray(self.parser.getYpositions(traj_pos)).reshape(len(traj_time), 1))
        traj_pos_z = (np.asarray(self.parser.getZpositions(traj_pos)).reshape(len(traj_time), 1))
        q = [ [data[3], data[4], data[5], data[6]] for data in traj]

        yinterp_x = interp1d((traj_time), np.transpose(traj_pos_x), axis=1, fill_value="extrapolate")
        yinterp_y = interp1d((traj_time), np.transpose(traj_pos_y), axis=1, fill_value="extrapolate")
        yinterp_z = interp1d((traj_time), np.transpose(traj_pos_z), axis=1, fill_value="extrapolate")

        y_new_x = yinterp_x(xvals)
        y_new_y = yinterp_y(xvals)
        y_new_z = yinterp_z(xvals)

        qstart = traj[0][3:7]
        qend = traj[-1][3:7]
        
        slerp = Slerp(traj_time, R.from_quat(q))
        interp_rots = slerp(xvals)
        
        interpol_traj = []
        
        for i, data in enumerate(y_new_x[0]):
            pos = [y_new_x[0][i], y_new_y[0][i], y_new_z[0][i]]
            ori = list(interp_rots[i].as_quat())
            t = [xvals[i]]
            demo = pos + ori + t
            interpol_traj.append( demo )

        return interpol_traj


    def match_refined_predicted(self, pred_traj, refined_traj):
        # pred_traj = self.parser.normalize_trajectory_time_float(pred_traj)
        
        # refined_traj = self.parser.normalize_trajectory_time_float(refined_traj)

        ## resample trajectories such that they can be subtracted

        refined_traj_pose = self.parser.getCartesianPositions(refined_traj)
        refined_traj_time = self.parser.get_time_vector_float(refined_traj)

        pred_traj_pose = self.parser.getCartesianPositions(pred_traj)
        pred_traj_time = self.parser.get_time_vector_float(pred_traj)
        # print("pred_traj_time = " + str(pred_traj_time))
        # print("ref_traj_time = " + str(refined_traj_time))
        # print("pred_traj_pos = " + str(pred_traj_pose))
        # print("ref_traj_pos = " + str(refined_traj_pose))

        n_pred = len(pred_traj)
        n_refined = len(refined_traj)

        # get lengths of both vectors
        n = [n_pred, n_refined]
        max_length = max(n)

        # dt = self.parser.getTimeInterval(trajectories[np.argmax(n)])
        dt_pred = self.parser.getTimeInterval(pred_traj)
        dt_refined = self.parser.getTimeInterval(refined_traj)

        T_pred = pred_traj_time[-1]
        T_refined = refined_traj_time[-1]

        l = max_length

        xvals_refined = np.linspace(0.0, T_refined, l)
        xvals_pred = np.linspace(0.0, T_pred, l)

        refined_traj_pos_x = (np.asarray(self.parser.getXpositions(refined_traj_pose)).reshape(len(refined_traj_time), 1))
        refined_traj_pos_y = (np.asarray(self.parser.getYpositions(refined_traj_pose)).reshape(len(refined_traj_time), 1))
        refined_traj_pos_z = (np.asarray(self.parser.getZpositions(refined_traj_pose)).reshape(len(refined_traj_time), 1))

        yinterp_refined_x = interp1d((refined_traj_time), np.transpose(refined_traj_pos_x), axis=1, fill_value="extrapolate")
        yinterp_refined_y = interp1d((refined_traj_time), np.transpose(refined_traj_pos_y), axis=1, fill_value="extrapolate")
        yinterp_refined_z = interp1d((refined_traj_time), np.transpose(refined_traj_pos_z), axis=1, fill_value="extrapolate")

        y_refined_new_x = yinterp_refined_x(xvals_refined)
        y_refined_new_y = yinterp_refined_y(xvals_refined)
        y_refined_new_z = yinterp_refined_z(xvals_refined)
        

        pred_traj_pos_x = (np.asarray(self.parser.getXpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))
        pred_traj_pos_y = (np.asarray(self.parser.getYpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))
        pred_traj_pos_z = (np.asarray(self.parser.getZpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))

        yinterp_pred_x = interp1d((pred_traj_time), np.transpose(pred_traj_pos_x), axis=1, fill_value="extrapolate")
        yinterp_pred_y = interp1d((pred_traj_time), np.transpose(pred_traj_pos_y), axis=1, fill_value="extrapolate")
        yinterp_pred_z = interp1d((pred_traj_time), np.transpose(pred_traj_pos_z), axis=1, fill_value="extrapolate")


        y_pred_new_x = yinterp_pred_x(xvals_pred)
        y_pred_new_y = yinterp_pred_y(xvals_pred)
        y_pred_new_z = yinterp_pred_z(xvals_pred)

        ypred = [list(y_pred_new_x[0]), list(y_pred_new_y[0]), list(y_pred_new_z[0])]
        yref = [list(y_refined_new_x[0]), list(y_refined_new_y[0]), list(y_refined_new_z[0])]

        return ypred, yref