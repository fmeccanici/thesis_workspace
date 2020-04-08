#!/usr/bin/env python

import rospy, ast, os, os.path
import numpy as np
from pyquaternion import Quaternion
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline, InterpolatedUnivariateSpline

class trajectoryParser():
    def __init__(self):
        self.raw_trajectory = []

    def plan2traj(self, plan, qstart, qend):
        traj = []

        # set amount of datapoints for linear interpolation of quaternionss
        n = len(plan.plan.points)-2

        # linearly interpolate the quaternions
        quat = trajectoryParser.interpolateQuaternions(qstart, qend, n)
        for i,q in enumerate(quat):
            traj.append([plan.plan.points[i].positions[0], plan.plan.points[i].positions[1], plan.plan.points[i].positions[2], q.imaginary[0], q.imaginary[1], q.imaginary[2], q.real, rospy.Duration(plan.plan.times[i]).secs, rospy.Duration(plan.plan.times[i]).nsecs] )

        return traj
    
    @classmethod
    def interpolateQuaternions(cls, qstart, qend, n, include_endpoints=True):
        q1 = Quaternion(a=qstart[3], b=qstart[0], c=qstart[1], d=qstart[2])
        q2 = Quaternion(a=qend[3], b=qend[0], c=qend[1], d=qend[2])

        return Quaternion.intermediates(q1, q2, n, include_endpoints=include_endpoints)


    def getCartesianPositions(self, traj):
        return [traj[0:3] for traj in traj]

    def getXpositions(self, traj):
        return [traj[0] for traj in traj]

    def getYpositions(self, traj):
        return [traj[1] for traj in traj]
        
    def getZpositions(self, traj):
        return [traj[2] for traj in traj]

    def openTrajectoryFile(self, traj_file, path):
        with open(path+traj_file, "r") as traj:
            raw_trajectory = ast.literal_eval(traj.read())    
        return raw_trajectory

    def _containsOrientation(self, trajectory):
        if len(trajectory) == 9:
            return True
        elif len(trajectory) == 5:
            return False

    def remove_t_float(self, traj):
        trajectory_without_t = []
        for x in traj:
            trajectory_without_t.append(x[:-1])

        return trajectory_without_t
    def add_t_secs_nsecs(self, traj, t_secs_nsecs):

        for i in range(len(traj)):
            traj[i] += t_secs_nsecs[i]

        return traj

    def trajFloatToSecsNsecs(self, trajectory):
        t_float = self.getTimeVectorFloat(trajectory)
        t_secs_nsecs = self._floatToSecsNsecs(t_float)
        

        traj = self.remove_t_float(trajectory)

        return self.add_t_secs_nsecs(traj, t_secs_nsecs)

    def getTimeVectorFloat(self, trajectory):
        return [  x[-1] for x in trajectory ]

    def _getTimeVector(self, trajectory):
        return [ [x[-2], x[-1] ] for x in trajectory ]

    def _secsNsecsToFloat(self, t_secs_nsecs):
        return map(lambda x: float(x[0]) + x[1] / 10.0**9, t_secs_nsecs)

    def secsNsecsToFloatSingle(self, t_secs_nsecs):
        return float(t_secs_nsecs[0]) + t_secs_nsecs[1] / 10.0**9

    def _floatToSecsNsecs(self, t_float):
        t_secs_nsecs = map(lambda x: [rospy.Duration(x).secs, rospy.Duration(x).nsecs], t_float)
        return t_secs_nsecs
    
    def getTimeIntervalFloat(self, trajectory):
        return trajectory[1][-1] - trajectory[0][-1]
    
    def getTimeInterval(self, trajectory):
        t = self._secsNsecsToFloat(self._getTimeVector(trajectory))
        return t[1] - t[0]

    def getQuaternion(self, traj):
        return traj[3:7]
    
    def _dt2n(self, dt):
        return np.log10(dt * 10**9)

    def _secsNsecs2Duration(self, secs, nsecs):
        return rospy.Duration(secs, nsecs)

    def durationVector2secsNsecsVector(self,t):
        return map(lambda x: [x.secs, x.nsecs], t)

    def _duration2secsNsecs(self, t):
        return [t.secs, t.nsecs]

    def _roundTmatrix(self, t, digits):
        return map(lambda x: round(x, digits), t)

    def _removeTmatrix(self, trajectory):
        trajectory_without_t = []
        for x in trajectory:
            trajectory_without_t.append(x[-2:])

        return trajectory_without_t

    def _addTmatrix(self, trajectory, t):
        
        for i,x in enumerate(trajectory):
            trajectory.append(t[i])
        return trajectory

    def _numDigits(self, dt):
        s = str(dt)
        if not '.' in s:
            return 0
        return len(s) - s.index('.') - 1
        
    # parse trajectory txt file
    def _normalize(self, trajectory):
        t = self._getTimeVector(trajectory)

        # check if trajectory time does not start with 0 --> normalize them
        if trajectory[0][-2] != 0:
            t0 = self._getTimeVector([trajectory[0]])[0]
            t0 = self._secsNsecs2Duration(t0[0], t0[1])

            t = list(map(lambda x: self._duration2secsNsecs(rospy.Duration(x[0], x[1]) - t0), t))

            secs = [_t[0] for _t in t]
            nsecs = [_t[1] for _t in t]

            for i,datapoint in enumerate(trajectory):
                datapoint[-2] = secs[i]
                datapoint[-1] = nsecs[i]


        return trajectory

    def downsample(self, trajectory, dt):
        normalized_trajectory = self._normalize(trajectory)


        n = self._dt2n(dt)
        resampled_trajectory = []

        if isinstance(dt, int):
            for i in range(len(normalized_trajectory)):
                try:
                    # sample one second
                    if normalized_trajectory[i][-2] != normalized_trajectory[i+1][-2]:
                        resampled_trajectory.append(normalized_trajectory[i+1])

                except IndexError: continue

            if dt != 1:
                i = 0
                j = 0
                
                while True:
                    try:
                        if normalized_trajectory[i+1][-2] == dt + normalized_trajectory[j][-2]:
                            resampled_trajectory.append(normalized_trajectory[i+1])
                            j = i + 1
                            i += 1
                        else:
                            i += 1
                    except IndexError: break

        elif (10**n % 10 == 0) and (dt < 1.0) and (dt > 0.0):
            t = self._secsNsecsToFloat(self._getTimeVector(normalized_trajectory))
            
            t = list(self._roundTmatrix(t, self._numDigits(dt)))
            t_s_ns = list(self._floatToSecsNsecs(t))


            self._removeTmatrix(normalized_trajectory)

            for i in range(len(normalized_trajectory)):
                try:
                    if t[i] != t[i+1]:
                        resampled_trajectory.append(normalized_trajectory[i+1][0:-2] + t_s_ns[i])

                except IndexError: continue
        else:
            print("Invalid dt")
    

        return resampled_trajectory

    def parse(self, traj_file, path, dt):
        trajectory = self.openTrajectoryFile(traj_file, path)
        return self.downsample(trajectory, dt)

    def get_context(self, traj):
        return traj[8:]

    def round_list(self, l):
        return [ round(x, 2) for x in l]

    def sort_first(self, val):
        return val[0]

    def arrays_in_list_to_list_in_list(self, traj):
        return [list(x) for x in traj]

    def interpolate_raw_trajectory(self, raw_traj, n):
        traj_pose = self.getCartesianPositions(raw_traj)
        traj_time = self._getTimeVector(raw_traj)
                
        T = self.secsNsecsToFloatSingle(raw_traj[-1][-2:])
        object_pose = raw_traj[0][7:14]

        xvals = np.linspace(0, T, n)

        traj_time = ((np.asarray(self._secsNsecsToFloat(traj_time))))
        traj_pos_x = (np.asarray(self.getXpositions(traj_pose)).reshape(len(traj_time), 1))
        traj_pos_y = (np.asarray(self.getYpositions(traj_pose)).reshape(len(traj_time), 1))
        traj_pos_z = (np.asarray(self.getZpositions(traj_pose)).reshape(len(traj_time), 1))

        yinterp_x = interp1d((traj_time), np.transpose(traj_pos_x), axis=1, fill_value="extrapolate")
        yinterp_y = interp1d((traj_time), np.transpose(traj_pos_y), axis=1, fill_value="extrapolate")
        yinterp_z = interp1d((traj_time), np.transpose(traj_pos_z), axis=1, fill_value="extrapolate")

        y_new_x = yinterp_x(xvals)
        y_new_y = yinterp_y(xvals)
        y_new_z = yinterp_z(xvals)

        qstart = raw_traj[0][3:7]
        qend = raw_traj[-1][3:7]

        interpol_traj = []
        t_secs_nsecs = self._floatToSecsNsecs(xvals)
        for i,q in enumerate(self.interpolateQuaternions(qstart, qend, n, False)):
            pos = [y_new_x[0][i], y_new_y[0][i], y_new_z[0][i], q[1], q[2], q[3], q[0]]
            ynew = pos + object_pose + [xvals[i]]

            interpol_traj.append(ynew)


        return interpol_traj
    def interpolate_learned_keypoints(self, traj, n_desired):
        n = len(traj)
        T = traj[-1][-1]
        # print(traj[-1])
        x = np.linspace(0, T, n)

        object_pose = traj[0][7:10]
        # cs = CubicSpline(x, traj)
        cartx = [data[0] for data in traj]
        carty = [data[1] for data in traj]
        cartz = [data[2] for data in traj]

        splinex = InterpolatedUnivariateSpline(x, cartx)
        spliney = InterpolatedUnivariateSpline(x, carty)
        splinez = InterpolatedUnivariateSpline(x, cartz)

        xdesired = np.linspace(0, T, n_desired)
        dt_new = T / n_desired
        print(x[-1])
        print(xdesired[-1])
        cartx_new = splinex(xdesired)
        carty_new = spliney(xdesired)
        cartz_new = splinez(xdesired)

        qstart = traj[0][3:7]
        qend = traj[-1][3:7]

        interpol_traj = []
        for i,q in enumerate(self.interpolateQuaternions(qstart, qend, n_desired, False)):
            pos = [cartx_new[i], carty_new[i], cartz_new[i], q[1], q[2], q[3], q[0]]
            ynew = pos + object_pose + [xdesired[i]]

            interpol_traj.append(ynew)

        # ynew_list = [list(x) for x in ynew]

        return interpol_traj, dt_new

    def isRaw(self, traj):
        if len(traj[0]) == 16:
            return True
        else:
            return False

    def tIsFloat(self, traj):
        if isinstance(traj[0][14], int):
            return False
        elif isinstance(traj[0][14], float):
            return True
            
    def load_trajectories_from_folder_and_downsample(self, input_path, dt):
        traj_files = [name for name in os.listdir(input_path) if os.path.isfile(os.path.join(input_path, name))]
        
        trajectories = []
        trajectories_lengths = []

        for traj in traj_files:
            trajectory = self.openTrajectoryFile(traj, input_path)
            trajectory = self._normalize(trajectory)
            trajectory = self.downsample(trajectory, dt)
            print(len(trajectory))

            trajectories.append(trajectory)
            trajectories_lengths.append(len(trajectory))

        return trajectories, trajectories_lengths

    def resample_trajectory(self,traj1, traj2):

        n1 = len(traj1)
        n2 = len(traj2)
        dt1 = self.getTimeInterval(self._normalize(traj1))
        dt2 = self.getTimeInterval(self._normalize(traj2))

        traj1 = self._normalize(traj1)
        traj2 = self._normalize(traj2)



        dt_new = n2 / (n1 / dt1)

        traj2_pos = self.getCartesianPositions(traj2)
        traj2_time = self._getTimeVector(traj2)

        # we want to downsample to the smallest trajectory, which is traj1
        l = len(traj1)
        xvals2 = np.linspace(0, l*dt_new, l)

        traj2_time = np.asarray(self._secsNsecsToFloat(traj2_time))
        traj2_pos_x = np.asarray(self.getXpositions(traj2_pos)).reshape(len(traj2_pos), 1)
        traj2_pos_y = np.asarray(self.getYpositions(traj2_pos)).reshape(len(traj2_pos), 1)
        traj2_pos_z = np.asarray(self.getZpositions(traj2_pos)).reshape(len(traj2_pos), 1)

        yinterp_traj2_x = interp1d((traj2_time), np.transpose(traj2_pos_x), axis=1, fill_value="extrapolate")
        yinterp_traj2_y = interp1d((traj2_time), np.transpose(traj2_pos_y), axis=1, fill_value="extrapolate")
        yinterp_traj2_z = interp1d((traj2_time), np.transpose(traj2_pos_z), axis=1, fill_value="extrapolate")
        
        y_traj2_new_x = yinterp_traj2_x(xvals2)
        y_traj2_new_y = yinterp_traj2_y(xvals2)
        y_traj2_new_z = yinterp_traj2_z(xvals2)
        
        qstart = traj2[0][3:7]
        qend = traj2[-1][3:7]
        
        object_info = traj2[0][7:14]

        traj2 = []

        # print(object_info)

        for i,q in enumerate(self.interpolateQuaternions(qstart, qend, l, False)):
            traj2.append([y_traj2_new_x[0][i], y_traj2_new_y[0][i], y_traj2_new_z[0][i]] + [q[1], q[2], q[3], q[0]] + object_info + [xvals2[i]])
        
        return traj2

    def resample_trajectories(self, trajectories, traj_min_length):
        trajectories_resampled = []
        for traj in trajectories:
            traj_res = self.resample_trajectory(traj_min_length, traj)
            trajectories_resampled.append(traj_res)
            plt.plot(self.getCartesianPositions(traj))
            plt.xlabel('datapoint [-]')
            plt.ylabel('position [m]')
            plt.title('Cartesian end effector positions before resampling')
        plt.show()
        return trajectories_resampled

    def resample_and_store_trajectories(self, trajectories, traj_min_length, output_path):

        trajectories_resampled = self.resample_trajectories(trajectories, traj_min_length)

        for i in range(0, len(trajectories_resampled)):
            traj = self.get_relevant_learning_data(trajectories_resampled[i])
            traj_file = open(output_path + "resampled_" + str(i) + ".txt", "w+")
            traj_file.write(str(traj))
            traj_file.close()

    def get_relevant_learning_data(self, traj):
        traj_new = []

        # T doesnt work properly --> chose dt as output
        dt = traj[-1][-1] - traj[-2][-1]
        for data in traj:
            traj_new.append(data[0:7] + data[7:10] + [dt] )

        return traj_new
    
    def parse_to_relative_trajectory(self, input_path):
        traj_files = [name for name in os.listdir(input_path) if os.path.isfile(os.path.join(input_path, name))]
        trajs = []
        for traj in traj_files:
            trajs.append(self.openTrajectoryFile(traj, input_path))

        traj_files_parsed = []
        for traj in trajs:
            marker_pos = traj[0][7:10]
            traj_files_parsed.append(self.traj_wrt_base_to_wrt_marker(marker_pos, traj)) 

    def traj_wrt_base_to_wrt_marker(self, marker_pos, traj):
        marker_wrt_base = np.asarray(marker_pos)

        traj_wrt_ee = []
        for data in traj:
            
            pos = list(np.subtract(data[0:3], marker_wrt_base))
            ori = list(data[3:7])
            traj_wrt_ee.append(pos + ori + data[7:])
        
        return traj_wrt_ee

    def marker_wrt_base_to_marker_wrt_ee(self, marker_pos, traj):
        marker_wrt_ee = np.asarray(marker_pos)

        marker_wrt_ee = []
        for data in traj:
            pos = list(np.subtract(marker_wrt_ee, data[0:3]))
            ori = list(data[3:7])
            marker_wrt_ee.append(pos)
        
        return traj_wrt_ee

    def prepare_for_learning(self, input_path, output_path):
        relative_trajectories = parser.parse_to_relative_trajectory(input_path)
        for traj in relative_trajectories:
            context = traj[7:11]
            
if __name__ == "__main__":
    parser = trajectoryParser()

    dt = 0.01

    input_path = '/home/fmeccanici/Documents/thesis/lfd_ws/src/trajectory_teaching/data/with_object2/'
