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
        # print("n = " + str(n))
        # print("x = " + str(x))
        # print("T = " + str(T))
        # print("pred_traj = " + str(pred_traj[-1]))

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

    def interpolate_predicted_trajectory(self, traj, n):
        traj_pos = self.parser.getCartesianPositions(traj)
        traj_time = self.parser.get_time_vector_float(traj)
        T = self.parser.get_total_time(traj)

        object_pos = self.parser.get_context(traj)

        xvals = np.linspace(0, T, n)

        traj_pos_x = (np.asarray(self.parser.getXpositions(traj_pos)).reshape(len(traj_time), 1))
        traj_pos_y = (np.asarray(self.parser.getYpositions(traj_pos)).reshape(len(traj_time), 1))
        traj_pos_z = (np.asarray(self.parser.getZpositions(traj_pos)).reshape(len(traj_time), 1))

        yinterp_x = interp1d((traj_time), np.transpose(traj_pos_x), axis=1, fill_value="extrapolate")
        yinterp_y = interp1d((traj_time), np.transpose(traj_pos_y), axis=1, fill_value="extrapolate")
        yinterp_z = interp1d((traj_time), np.transpose(traj_pos_z), axis=1, fill_value="extrapolate")

        y_new_x = yinterp_x(xvals)
        y_new_y = yinterp_y(xvals)
        y_new_z = yinterp_z(xvals)

        qstart = traj[0][3:7]
        qend = traj[-1][3:7]

        interpol_traj = []

        for i,q in enumerate(self.interpolate_quaternions(qstart, qend, n, False)):
            pos = [y_new_x[0][i], y_new_y[0][i], y_new_z[0][i], q[1], q[2], q[3], q[0]]
            ynew = pos + object_pos + [xvals[i]]

            interpol_traj.append(ynew)


        return interpol_traj


    def match_refined_predicted(self, pred_traj, refined_traj):
        pred_traj = self.parser.trajFloatToSecsNsecs(pred_traj)
        
        refined_traj = self.parser.normalize(refined_traj)

        new_trajectory = []

        ## resample trajectories such that they can be subtracted
        
        refined_traj_pose = self.parser.getCartesianPositions(refined_traj)
        refined_traj_time = self.parser.get_time_vector_float(refined_traj)

        pred_traj_pose = self.parser.getCartesianPositions(pred_traj)
        pred_traj_time = self.parser.get_time_vector_float(pred_traj)

        # plt.plot(refined_traj_pose)
        # plt.plot(pred_traj_pose)
        # plt.title('Predicted and refined trajectory before resampling')
        # plt.xlabel('datapoint [-]')
        # plt.ylabel('position [m]')
        # plt.show()

        n_pred = len(pred_traj)
        n_refined = len(refined_traj)

        # get lengths of both vectors
        n = [n_pred, n_refined]
        max_length = max(n)

        # dt = self.parser.getTimeInterval(trajectories[np.argmax(n)])
        dt_pred = self.parser.getTimeInterval(pred_traj)
        dt_refined = self.parser.getTimeInterval(refined_traj)

        T_pred = self.parser.secs_nsecs_to_float_single(pred_traj_time[-1])
        T_refined = self.parser.secs_nsecs_to_float_single(refined_traj_time[-1])

        l = max_length

        xvals_refined = np.linspace(0.0, T_refined, l)
        xvals_pred = np.linspace(0.0, T_pred, l)
        print('check2')

        refined_traj_time = ((np.asarray(self.parser.secs_nsecs_to_float_vector(refined_traj_time))))
        refined_traj_pos_x = (np.asarray(self.parser.getXpositions(refined_traj_pose)).reshape(len(refined_traj_time), 1))
        refined_traj_pos_y = (np.asarray(self.parser.getYpositions(refined_traj_pose)).reshape(len(refined_traj_time), 1))
        refined_traj_pos_z = (np.asarray(self.parser.getZpositions(refined_traj_pose)).reshape(len(refined_traj_time), 1))

        yinterp_refined_x = interp1d((refined_traj_time), np.transpose(refined_traj_pos_x), axis=1, fill_value="extrapolate")
        yinterp_refined_y = interp1d((refined_traj_time), np.transpose(refined_traj_pos_y), axis=1, fill_value="extrapolate")
        yinterp_refined_z = interp1d((refined_traj_time), np.transpose(refined_traj_pos_z), axis=1, fill_value="extrapolate")

        y_refined_new_x = yinterp_refined_x(xvals_refined)
        y_refined_new_y = yinterp_refined_y(xvals_refined)
        y_refined_new_z = yinterp_refined_z(xvals_refined)
        

        pred_traj_time = ((np.asarray(self.parser.secs_nsecs_to_float_vector(pred_traj_time))))
        pred_traj_pos_x = (np.asarray(self.parser.getXpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))

        pred_traj_pos_y = (np.asarray(self.parser.getYpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))
        pred_traj_pos_z = (np.asarray(self.parser.getZpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))

        print(pred_traj_pos_x[0])
        yinterp_pred_x = interp1d((pred_traj_time), np.transpose(pred_traj_pos_x), axis=1, fill_value="extrapolate")
        yinterp_pred_y = interp1d((pred_traj_time), np.transpose(pred_traj_pos_y), axis=1, fill_value="extrapolate")
        yinterp_pred_z = interp1d((pred_traj_time), np.transpose(pred_traj_pos_z), axis=1, fill_value="extrapolate")

        print('check3')


        y_pred_new_x = yinterp_pred_x(xvals_pred)
        y_pred_new_y = yinterp_pred_y(xvals_pred)
        y_pred_new_z = yinterp_pred_z(xvals_pred)

        ypred = [list(y_pred_new_x[0]), list(y_pred_new_y[0]), list(y_pred_new_z[0])]
        yref = [list(y_refined_new_x[0]), list(y_refined_new_y[0]), list(y_refined_new_z[0])]

        return ypred, yref