#!/usr/bin/env python3.5
from learning_from_demonstration.trajectory_parser import trajectoryParser
import numpy as np
from scipy.interpolate import interp1d, InterpolatedUnivariateSpline
from pyquaternion import Quaternion

class trajectoryResampler():
    def __init__(self):
        self.parser = trajectoryParser()
    
    def interpolate_quaternions(self, qstart, qend, n, include_endpoints=True):
        q1 = Quaternion(a=qstart[3], b=qstart[0], c=qstart[1], d=qstart[2])
        q2 = Quaternion(a=qend[3], b=qend[0], c=qend[1], d=qend[2])

        return Quaternion.intermediates(q1, q2, n, include_endpoints=include_endpoints)
    
    def interpolate_learned_keypoints(self, pred_traj, n_desired):
        n = len(pred_traj)
        T = pred_traj[-1][-1]
        x = np.linspace(0, T, n)
        print("n = " + str(n))
        print("x = " + str(x))
        print("T = " + str(T))
        print("pred_traj = " + str(pred_traj[-1]))
        
        object_pose = pred_traj[0][7:10]
        cartx = [data[0] for data in pred_traj]
        carty = [data[1] for data in pred_traj]
        cartz = [data[2] for data in pred_traj]

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

        interpol_pred_traj = []
        for i,q in enumerate(self.interpolate_quaternions(qstart, qend, n_desired, False)):
            pos = [cartx_new[i], carty_new[i], cartz_new[i], q[1], q[2], q[3], q[0]]
            ynew = pos + object_pose + [xdesired[i]]

            interpol_pred_traj.append(ynew)

        return interpol_pred_traj, dt_new

    def interpolate_raw_trajectory(self, raw_traj, n):
        traj_pose = self.parser.getCartesianPositions(raw_traj)
        traj_time = self.parser.get_time_vector_float(raw_traj)
        T = self.parser.get_total_time(raw_traj)

        object_pose = self.parser.get_object_pose(raw_traj)

        xvals = np.linspace(0, T, n)

        traj_pos_x = (np.asarray(self.parser.getXpositions(traj_pose)).reshape(len(traj_time), 1))
        traj_pos_y = (np.asarray(self.parser.getYpositions(traj_pose)).reshape(len(traj_time), 1))
        traj_pos_z = (np.asarray(self.parser.getZpositions(traj_pose)).reshape(len(traj_time), 1))

        yinterp_x = interp1d((traj_time), np.transpose(traj_pos_x), axis=1, fill_value="extrapolate")
        yinterp_y = interp1d((traj_time), np.transpose(traj_pos_y), axis=1, fill_value="extrapolate")
        yinterp_z = interp1d((traj_time), np.transpose(traj_pos_z), axis=1, fill_value="extrapolate")

        y_new_x = yinterp_x(xvals)
        y_new_y = yinterp_y(xvals)
        y_new_z = yinterp_z(xvals)

        qstart = raw_traj[0][3:7]
        qend = raw_traj[-1][3:7]

        interpol_traj = []

        for i,q in enumerate(self.interpolate_quaternions(qstart, qend, n, False)):
            pos = [y_new_x[0][i], y_new_y[0][i], y_new_z[0][i], q[1], q[2], q[3], q[0]]
            ynew = pos + object_pose + [xvals[i]]

            interpol_traj.append(ynew)


        return interpol_traj