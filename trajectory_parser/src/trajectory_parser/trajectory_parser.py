#!/usr/bin/env python

import ast
import numpy as np
import rospy
from pyquaternion import Quaternion

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

if __name__ == "__main__":

    trajectory_parser = trajectoryParser()
    traj_file = "EEtrajectory_5.txt"
    path = "/home/fmeccanici/Documents/thesis/lfd_ws/src/marco_lfd/data/"
    dt = 0.1
    resampled_trajectory = (trajectory_parser.parse(traj_file, path, dt))

    res_traj_file = "EEtrajectory_5_resampled.txt"
    res_path = "/home/fmeccanici/Documents/thesis/lfd_ws/src/marco_lfd/data/resampled/"

    with open(res_path + res_traj_file, 'w+') as f:
        f.write(str(resampled_trajectory))
