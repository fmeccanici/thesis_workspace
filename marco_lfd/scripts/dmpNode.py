#!/usr/bin/env python

import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *
from os import listdir
from os.path import isfile, join
import ast
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time
from geomagic_touch_m.msg import GeomagicButtonEvent
from slave_control.msg import ControlState
import os, stat
import json

from trajectoryParser.trajectoryParser import *
from scipy.interpolate import interp1d
from trajectory_visualizer.trajectory_visualizer import *
from spline_interpolation.spline_interpolation import *



class dmpVariables():
    def __init__(self, traj, dt, K, D, goal=[0.77,-0.05,0.72], x_0 = [0.635174242834, -0.119081233692, 0.904780355276], dims=3, num_bases=4, x_dot_0 = [0.0, 0.0, 0.0], t_0 = 0, goal_thresh = [0.2,0.2,0.2], seg_length = -1, integrate_iter = 5, tau_factor=1):
        self.traj = traj
        self.dt = dt
        self.goal = goal
        self.x_0 = x_0
        self.dims = dims
        self.K = K
        self.D = D
        self.num_bases = num_bases
        self.x_dot_0 = x_dot_0
        self.t_0 = t_0
        self.goal_thresh = goal_thresh
        self.seg_length = seg_length
        self.integrate_iter = integrate_iter
        self.tau_factor = tau_factor

class trajectoryStorageVariables():
    def __init__(self, open_loop_path, refined_path, resampled_path, raw_path, raw_file, new_path):
        self.open_loop_path = open_loop_path
        self.refined_path = refined_path
        self.resampled_path = resampled_path
        self.raw_path = raw_path
        self.raw_file = raw_file
        self.new_path = new_path

class dmpNode():

    def __init__(self):
        rospy.init_node('dmp_node', anonymous=True)

        self.end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", geometry_msgs.msg.PoseStamped, queue_size=10)
        # self.end_effector_current_state_sub = rospy.Subscriber("/end_effector_pose", geometry_msgs.msg.PoseStamped, self.end_effector_state_callback())
        self.geo_button_sub = rospy.Subscriber("geo_buttons_m", GeomagicButtonEvent, self._buttonCallbackToggle)
        self.slave_control_state_sub = rospy.Subscriber("slave_control_state", ControlState, self._slave_control_state_callback)
        self.traj_pub = rospy.Publisher('trajectory_visualizer/trajectory', MarkerArray, queue_size=10)
        self.frame_id = 'base_footprint'

        self.refinementFlag = 0

        self.grey_button = 0
        self.grey_button_prev = 0
        self.grey_button_toggle = 0

        self.white_button = 0
        self.white_button_prev = 0
        self.white_button_toggle = 0
        
        self.EEtrajectory = []
        self.recorded_motion = []
        self.parser = trajectoryParser()
        self.spl_interp = splineInterpolation()


    
    def _buttonCallbackToggle(self, data):
        self.grey_button_prev = self.grey_button
        self.grey_button = data.grey_button

        if (self.grey_button != self.grey_button_prev) and (self.grey_button == 1):
            self.grey_button_toggle = not self.grey_button_toggle

        if (self.white_button != self.white_button_prev) and (self.white_button == 1):
            self.white_button_toggle = not self.white_button_toggle

    def _buttonCallbackNoToggle(self, data):
        self.grey_button = data.grey_button
        self.white_button = data.white_button

    def _slave_control_state_callback(self,data):
        self.current_slave_pose = data.slave_pose.pose

    #Learn a DMP from demonstration data
    def _makeLFDRequest(self, dims, traj, dt, K_gain, 
                    D_gain, num_bases):
        demotraj = DMPTraj()
            
        for i in range(len(traj)):
            pt = DMPPoint()
            pt.positions = traj[i]
            demotraj.points.append(pt)
            demotraj.times.append(dt*i)
                
        k_gains = [K_gain]*dims
        d_gains = [D_gain]*dims
            
        print "Starting LfD..."
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj, k_gains, d_gains, num_bases)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "LfD done"    
                
        return resp

    #Set a DMP as active for planning
    def _makeSetActiveRequest(self, dmp_list):
        try:
            sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    #Generate a plan from a DMP
    def _makePlanRequest(self, x_0, x_dot_0, t_0, goal, goal_thresh, 
                        seg_length, tau, dt, integrate_iter):
        print "Starting DMP planning..."
        rospy.wait_for_service('get_dmp_plan')
        try:
            gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
            resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "DMP planning done"   
                
        return resp

    def makePlan(self, dmpVariables):
        #Create a DMP from a 3D trajectory
        dims = dmpVariables.dims               
        K = dmpVariables.K                 
        D = dmpVariables.D 
        num_bases = dmpVariables.num_bases
        traj = dmpVariables.traj
        dt = dmpVariables.dt

        resp = self._makeLFDRequest(dims, traj, dt, K, D, num_bases)

        #Set it as the active DMP
        self._makeSetActiveRequest(resp.dmp_list)

        # Get DMP parameters
        x_dot_0 = dmpVariables.x_dot_0   
        t_0 = dmpVariables.t_0                
        goal = dmpVariables.goal         #Plan to a different goal than demo
        goal_thresh = dmpVariables.goal_thresh
        seg_length = dmpVariables.seg_length         #Plan until convergence to goal
        tau = dmpVariables.tau_factor * resp.tau       #Desired plan should take twice as long as demo
        x_0 = dmpVariables.x_0        
        
        integrate_iter = dmpVariables.integrate_iter      # dt is rather large, so this is > 1  
        self.plan = self._makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                            seg_length, tau, dt, integrate_iter)
    
    def getInitPose(self):
        slave_goal = geometry_msgs.msg.PoseStamped()
        slave_goal.pose.position.x = 0.635174242834
        slave_goal.pose.position.y = -0.119081233692
        slave_goal.pose.position.z = 0.904780355276

        slave_goal.pose.orientation.w = 0.00470101575578
        slave_goal.pose.orientation.x = 0.994781110161
        slave_goal.pose.orientation.y = 0.100705187531
        slave_goal.pose.orientation.z = 0.0157133230571

        slave_goal.header.seq = 0
        slave_goal.header.frame_id = "/base_footprint"

        return [slave_goal.pose.position.x, slave_goal.pose.position.y, slave_goal.pose.position.z, slave_goal.pose.orientation.w, slave_goal.pose.orientation.x, slave_goal.pose.orientation.y, slave_goal.pose.orientation.z]

    def goToInitialPose(self):
        rospy.loginfo("Moving to initial pose")
        T = 3
        x = [self.current_slave_pose.position.x, 0.403399335619]
        y = [self.current_slave_pose.position.y, -0.430007534239]
        z = [self.current_slave_pose.position.z, 1.16269467394]

        # y = [[0.403399335619, -0.430007534239, 1.16269467394], [self.current_slave_pose.position.x, self.current_slave_pose.position.y, self.current_slave_pose.position.z]]
        t = [rospy.Time.now(), rospy.Time.now() + rospy.Duration(T)]

        t = self.parser._secsNsecsToFloat(self.parser.durationVector2secsNsecsVector(t))

        fx = interp1d(t, x, fill_value="extrapolate")
        fy = interp1d(t, y, fill_value="extrapolate")
        fz = interp1d(t, z, fill_value="extrapolate")

        dt = 0.1
        tnew = np.arange(t[0],t[-1],dt)
        xnew = fx(tnew)
        ynew = fy(tnew)
        znew = fz(tnew)

        # print(ynew)
        for i in range(len(ynew)):
            slave_goal = geometry_msgs.msg.PoseStamped()
            # slave_goal.pose.position.x = 0.635174242834
            # slave_goal.pose.position.y = -0.119081233692
            # slave_goal.pose.position.z = 0.904780355276
            # slave_goal.pose.position.x = 0.403399335619
            # slave_goal.pose.position.y = -0.430007534239
            # slave_goal.pose.position.z = 1.16269467394
            slave_goal.pose.position.x = xnew[i]
            slave_goal.pose.position.y = ynew[i]
            slave_goal.pose.position.z = znew[i]


            slave_goal.pose.orientation.w = 0.00470101575578
            slave_goal.pose.orientation.x = 0.994781110161
            slave_goal.pose.orientation.y = 0.100705187531
            slave_goal.pose.orientation.z = 0.0157133230571

            slave_goal.header.seq = 0
            slave_goal.header.frame_id = "/base_footprint"

            # print(xnew[i])
            slave_goal.header.stamp = (rospy.Time.now())
            self.end_effector_goal_pub.publish(slave_goal)
            
            time.sleep(dt)

        # slave_goal.pose.position.x = 0.418045511755
        # slave_goal.pose.position.y = -0.257890132079
        # slave_goal.pose.position.z = 1.22142635352

        # slave_goal.pose.orientation.w = 0.10324307264
        # slave_goal.pose.orientation.x = 0.931765770479
        # slave_goal.pose.orientation.y = 0.0775427860034
        # slave_goal.pose.orientation.z = -0.339323640872

    def getUniqueRawFileName(self, raw_path):
        files = [f for f in listdir(path) if isfile(join(path, f))]
        numbers = [int(os.path.splitext(f)[0].split('_')[1]) for f in files]
        numbers.sort()
        files = ["raw_trajectory_" + str(number) + ".txt" for number in numbers]
        if os.stat(path + files[-1]).st_size == 0:
            return files[-1]
        else:
            return "raw_trajectory_" + str(int(os.path.splitext(files[-1])[0].split('_')[-1]) + 1) + ".txt"

    def getFileName(self, raw_file_name, traj_type = 'refined'):
        if traj_type == 'refined':
            return "refined_trajectory_" + raw_file_name.split('_')[-1]
        elif traj_type == 'resampled':
            return "resampled_trajectory_" + raw_file_name.split('_')[-1]
        elif traj_type == 'open_loop':
            return "open_loop_trajectory_" + raw_file_name.split('_')[-1]
        elif traj_type == 'new':
            return "new_trajectory_" + raw_file_name.split('_')[-1]

    def getCartesianPositions(self, traj):
        return [traj[0:3] for traj in traj]

    def getXpositions(self, traj):
        return [traj[0] for traj in traj]

    def getYpositions(self, traj):
        return [traj[1] for traj in traj]

    def getZpositions(self, traj):
        return [traj[2] for traj in traj]

    def positions2pointMessage(self, positions):
        point = Point(x=positions[0], 
                        y = positions[1],
                        z = positions[2])
        return point


    def trajectory2markerArray(self,traj):
        marker_array = []
        for i,positions in enumerate(traj):
            point = self.positions2pointMessage(positions)
            pose = Pose(position=point)

            marker = Marker(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.frame_id),
                                            pose=pose,
                                            type=Marker.SPHERE,
                                            scale=Vector3(0.02,0.02,0.02),
                                            id=i,
                                            color=ColorRGBA(r=1,a=1))
            marker_array.append(marker)

        return marker_array
    
    def publishMarkerArray(self, marker_array):
        self.traj_pub.publish(MarkerArray(markers=marker_array))

    def recordMotion(self, dmpVariables, storageVariables, motion_type='open_loop'):
        if motion_type == 'open_loop':
            rospy.loginfo("Recording open loop trajectory")
            dt = dmpVariables.dt
            t0 = rospy.Time.now()
            
            recorded_motion = []

            for i in range(len(self.plan.plan.points)):
                
                slave_goal = geometry_msgs.msg.PoseStamped()

                slave_goal.pose.position.x = self.plan.plan.points[i].positions[0]
                slave_goal.pose.position.y = self.plan.plan.points[i].positions[1]
                slave_goal.pose.position.z = self.plan.plan.points[i].positions[2]

                # slave_goal.pose.orientation.w = 0.60
                # slave_goal.pose.orientation.x = 0.77 
                # slave_goal.pose.orientation.y = 0.11
                # slave_goal.pose.orientation.z = -0.15

                slave_goal.pose.orientation.w = 0.00470101575578
                slave_goal.pose.orientation.x = 0.994781110161
                slave_goal.pose.orientation.y = 0.100705187531
                slave_goal.pose.orientation.z = 0.0157133230571

                slave_goal.header.seq = 0
            
                slave_goal.header.stamp = t0 + rospy.Duration(self.plan.plan.times[i])
                slave_goal.header.frame_id = "/base_footprint"
                
                self.end_effector_goal_pub.publish(slave_goal)
                time.sleep(dt)
                
                recorded_motion_time =  rospy.Duration(self.plan.plan.times[i])

                recorded_motion.append([self.current_slave_pose.position.x, self.current_slave_pose.position.y, self.current_slave_pose.position.z, recorded_motion_time.secs, recorded_motion_time.nsecs])
            
            self.save_data(storageVariables.open_loop_path, self.getFileName(storageVariables.raw_file, 'open_loop'), recorded_motion)
        
        elif motion_type == 'refinement':
            rospy.loginfo("Recording refined trajectory")
            dt = dmpVariables.dt
            t0 = rospy.Time.now()

            refined_datapoint = geometry_msgs.msg.PoseStamped()
            refined_trajectory = []

            open_loop_file = self.getFileName(storageVariables.raw_file, 'open_loop')
            open_loop_traj = self.parser.openTrajectoryFile(open_loop_file, storageVariables.open_loop_path)

            i = 0
            t = [t0]
            refined_motion_time = 0
            T = 0.0

            while self.white_button_toggle == 0:
                refined_motion_time += dt
                t.append(t0 + rospy.Duration(refined_motion_time)) 

                print(self.grey_button_toggle)
                if self.grey_button_toggle == 0: 
                    # print(self.refinementFlag)

                    if self.refinementFlag == 1:
                            try:
                                T = 1.0

                                # y1 = [self.plan.plan.points[i-1].positions[0], self.plan.plan.points[i-1].positions[1], self.plan.plan.points[i-1].positions[2]]
                                y1 = [self.current_slave_pose.position.x, self.current_slave_pose.position.y, self.current_slave_pose.position.z]
                                y2 = [self.plan.plan.points[i].positions[0], self.plan.plan.points[i].positions[1], self.plan.plan.points[i].positions[2]]

                                y3 = [self.plan.plan.points[i+1].positions[0], self.plan.plan.points[i+1].positions[1], self.plan.plan.points[i+1].positions[2]]
                                y4 = [self.plan.plan.points[i+2].positions[0], self.plan.plan.points[i+2].positions[1], self.plan.plan.points[i+2].positions[2]]

                                # t1 = self.parser._secsNsecsToFloat([self.parser._duration2secsNsecs(t[-2])])[0]
                                # t2 = self.parser._secsNsecsToFloat([self.parser._duration2secsNsecs(t[-1])])[0]
                                t1 = self.plan.plan.times[i-1]
                                t2 = self.plan.plan.times[i] + T
                                t3 = self.plan.plan.times[i+1] + T
                                t4 = self.plan.plan.times[i+2] + T

                                print(y1)
                                print(y2)
                                print(y3)
                                print(y4)

                                # t3 = t2 + dt
                                # t4 = t3 + dt
                                # t.insert(-1, rospy.Duration(t3))
                                # t.insert(-1, rospy.Duration(t4))

                                t_interp = [t1, t2, t3, t4]
                                y_interp = [y1, y2, y3, y4]
                                T = t2
                                try:
                                    tnew,ynew = (self.spl_interp.interpolate(y_interp, dt, t_interp, T))
                                except: break

                                for position in ynew:
                                    j = 0
                                    slave_goal = geometry_msgs.msg.PoseStamped()
                                    slave_goal.pose.position.x = position[0]
                                    slave_goal.pose.position.y = position[1]
                                    slave_goal.pose.position.z = position[2]
                                    slave_goal.pose.orientation.w = 0.00470101575578
                                    slave_goal.pose.orientation.x = 0.994781110161
                                    slave_goal.pose.orientation.y = 0.100705187531
                                    slave_goal.pose.orientation.z = 0.0157133230571
                                                
                                    slave_goal.header.seq = 0
                                
                                    slave_goal.header.stamp = rospy.Duration(tnew[j])

                                    slave_goal.header.frame_id = "/base_footprint"
                                    
                                    refined_trajectory.append([self.current_slave_pose.position.x, self.current_slave_pose.position.y, self.current_slave_pose.position.z,slave_goal.header.stamp.secs, slave_goal.header.stamp.nsecs])

                                    self.end_effector_goal_pub.publish(slave_goal)
                                    time.sleep(dt)

                                    j += 1

                                self.refinementFlag = 0

                            except IndexError: break
                    else:
                        try:
                            print('execution mode')
                            self.refinementFlag = 0

                            slave_goal = geometry_msgs.msg.PoseStamped()
                                
                            slave_goal.pose.position.x = self.plan.plan.points[i].positions[0]
                            slave_goal.pose.position.y = self.plan.plan.points[i].positions[1]
                            slave_goal.pose.position.z = self.plan.plan.points[i].positions[2]

                            slave_goal.pose.orientation.w = 0.00470101575578
                            slave_goal.pose.orientation.x = 0.994781110161
                            slave_goal.pose.orientation.y = 0.100705187531
                            slave_goal.pose.orientation.z = 0.0157133230571

                            slave_goal.header.seq = 0
                        
                            slave_goal.header.stamp = t0 + rospy.Duration(self.plan.plan.times[i] + T)
                            slave_goal.header.frame_id = "/base_footprint"

                            refined_datapoint.pose.position.x = self.current_slave_pose.position.x
                            refined_datapoint.pose.position.y = self.current_slave_pose.position.y
                            refined_datapoint.pose.position.z = self.current_slave_pose.position.z
                            # refined_datapoint.header.stamp = t[-1]
                            t_ = t0 + rospy.Duration(self.plan.plan.times[i] + T)
                            refined_datapoint.header.stamp = t_

                            self.end_effector_goal_pub.publish(slave_goal)
                            
                            time.sleep(dt)
                            refined_trajectory.append([refined_datapoint.pose.position.x, refined_datapoint.pose.position.y, refined_datapoint.pose.position.z, t_.secs, t_.nsecs])

                            i += 1
                        except IndexError: break
                    
                elif self.grey_button_toggle == 1:         
                    print('refinement mode')
                    self.refinementFlag = 1
                    refined_datapoint.pose.position.x = self.current_slave_pose.position.x
                    refined_datapoint.pose.position.y = self.current_slave_pose.position.y
                    refined_datapoint.pose.position.z = self.current_slave_pose.position.z

                    time.sleep(dt)
                    # refined_motion_time = rospy.Duration(self.plan.plan.times[i]) 
                    refined_trajectory.append([refined_datapoint.pose.position.x, refined_datapoint.pose.position.y, refined_datapoint.pose.position.z, t[-1].secs, t[-1].nsecs])

                    i += 1
                    
            self.save_data(storageVariables.refined_path, self.getFileName(storageVariables.raw_file, 'refined'), self.parser._normalize(refined_trajectory))
            return self.parser._normalize(refined_trajectory)

    def getDtFromPlan(self, plan):
        return plan.plan.times[1] - plan.plan.times[0]
    
    def plan2trajectory(self, plan):
        traj = []
        for i in range(len(plan.plan.times)):
            traj.append([plan.plan.points[i].positions[0], plan.plan.points[i].positions[1], plan.plan.points[i].positions[2]] + [rospy.Duration(plan.plan.times[i]).secs, rospy.Duration(plan.plan.times[i]).nsecs])
        return traj

    # from Ewerton: tau_D^new = tau_D^old + alpha * (tau_HR - tau_R)
    def determineRefinedTrajectory(self, storageVariables, alpha = 1):
        open_loop_traj = self.parser.openTrajectoryFile(self.getFileName(storageVariables.raw_file, 'open_loop'), storageVariables.open_loop_path)
        refined_traj = self.parser.openTrajectoryFile(self.getFileName(storageVariables.raw_file, 'refined'), storageVariables.refined_path)
        pred_traj = self.plan2trajectory(self.plan)

        if self.parser.getTimeInterval(pred_traj) == self.parser.getTimeInterval(refined_traj) == self.parser.getTimeInterval(open_loop_traj):
            print("dt matches")
            lengths = [len(self.parser._getTimeVector(pred_traj)), len(self.parser._getTimeVector(refined_traj)), len(self.parser._getTimeVector(open_loop_traj))]

            dt = self.parser.getTimeInterval(pred_traj)
            max_length = max(lengths)
            min_length = min(lengths)
            open_loop_traj_pose = self.getCartesianPositions(open_loop_traj)
            open_loop_traj_time = self.parser._getTimeVector(open_loop_traj)
            refined_traj_pose = self.getCartesianPositions(refined_traj)
            refined_traj_time = self.parser._getTimeVector(refined_traj)
            pred_traj_pose = self.getCartesianPositions(pred_traj)
            pred_traj_time = self.parser._getTimeVector(pred_traj)


            l = max_length

            dt_open_loop = dt * len(open_loop_traj_time) / lengths[np.argmax(lengths)]
            dt_refined = dt * len(open_loop_traj_time) / lengths[np.argmax(lengths)]
            dt_pred = dt * len(pred_traj_time) / lengths[np.argmax(lengths)]


            xvals_open_loop = np.linspace(dt_open_loop, l*dt_open_loop, l)
            xvals_refined = np.linspace(dt_refined, l*dt_refined, l)
            xvals_pred = np.linspace(dt_pred, l*dt_pred, l)

            open_loop_traj_time = ((np.asarray(self.parser._secsNsecsToFloat(open_loop_traj_time))))
            open_loop_traj_pos_x = (np.asarray(self.getXpositions(open_loop_traj_pose)).reshape(len(open_loop_traj_time), 1))
            open_loop_traj_pos_y = (np.asarray(self.getYpositions(open_loop_traj_pose)).reshape(len(open_loop_traj_time), 1))
            open_loop_traj_pos_z = (np.asarray(self.getZpositions(open_loop_traj_pose)).reshape(len(open_loop_traj_time), 1))

            # for some reason it says that it needs to extrapolate, which is not logical but it works right now
            yinterp_open_loop_x = interp1d((open_loop_traj_time), np.transpose(open_loop_traj_pos_x), axis=1, fill_value="extrapolate")
            yinterp_open_loop_y = interp1d((open_loop_traj_time), np.transpose(open_loop_traj_pos_y), axis=1, fill_value="extrapolate")
            yinterp_open_loop_z = interp1d((open_loop_traj_time), np.transpose(open_loop_traj_pos_z), axis=1, fill_value="extrapolate")

            y_open_loop_new_x = yinterp_open_loop_x(xvals_open_loop)
            y_open_loop_new_y = yinterp_open_loop_y(xvals_open_loop)
            y_open_loop_new_z = yinterp_open_loop_z(xvals_open_loop)


            refined_traj_time = ((np.asarray(self.parser._secsNsecsToFloat(refined_traj_time))))
            refined_traj_pos_x = (np.asarray(self.getXpositions(refined_traj_pose)).reshape(len(refined_traj_time), 1))
            refined_traj_pos_y = (np.asarray(self.getYpositions(refined_traj_pose)).reshape(len(refined_traj_time), 1))
            refined_traj_pos_z = (np.asarray(self.getZpositions(refined_traj_pose)).reshape(len(refined_traj_time), 1))

            yinterp_refined_x = interp1d((refined_traj_time), np.transpose(refined_traj_pos_x), axis=1, fill_value="extrapolate")
            yinterp_refined_y = interp1d((refined_traj_time), np.transpose(refined_traj_pos_y), axis=1, fill_value="extrapolate")
            yinterp_refined_z = interp1d((refined_traj_time), np.transpose(refined_traj_pos_z), axis=1, fill_value="extrapolate")

            y_refined_new_x = yinterp_refined_x(xvals_refined)
            y_refined_new_y = yinterp_refined_y(xvals_refined)
            y_refined_new_z = yinterp_refined_z(xvals_refined)

            pred_traj_time = ((np.asarray(self.parser._secsNsecsToFloat(pred_traj_time))))
            pred_traj_pos_x = (np.asarray(self.getXpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))
            pred_traj_pos_y = (np.asarray(self.getYpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))
            pred_traj_pos_z = (np.asarray(self.getZpositions(pred_traj_pose)).reshape(len(pred_traj_time), 1))

            yinterp_pred_x = interp1d((pred_traj_time), np.transpose(pred_traj_pos_x), axis=1, fill_value="extrapolate")
            yinterp_pred_y = interp1d((pred_traj_time), np.transpose(pred_traj_pos_y), axis=1, fill_value="extrapolate")
            yinterp_pred_z = interp1d((pred_traj_time), np.transpose(pred_traj_pos_z), axis=1, fill_value="extrapolate")

            y_pred_new_x = yinterp_pred_x(xvals_pred)
            y_pred_new_y = yinterp_pred_y(xvals_pred)
            y_pred_new_z = yinterp_pred_z(xvals_pred)

            # lengths are the same so doesnt matter which length I take
            open_loop_traj = []
            refined_traj = []
            pred_traj = []

            new_trajectory = []

            for i in range(len(refined_traj_time)):

                open_loop_traj = [y_open_loop_new_x[0][i], y_open_loop_new_y[0][i], y_open_loop_new_z[0][i], xvals_open_loop[i]]
                refined_traj = [y_refined_new_x[0][i], y_refined_new_y[0][i], y_refined_new_z[0][i], xvals_refined[i]] 
                pred_traj = [y_pred_new_x[0][i], y_pred_new_y[0][i], y_pred_new_z[0][i], xvals_pred[i]]
                
                # tau_D^new = tau_D^old + alpha * (tau_HR - tau_R)
                new_trajectory.append(list(np.add(pred_traj, alpha * (np.subtract(refined_traj, open_loop_traj) ))))
                
            self.save_data(storageVariables.new_path, self.getFileName(storageVariables.raw_file, 'new'), new_trajectory)

            return new_trajectory
        else:
            print("dt does not match")
            
        

    def executeTrajectory(self, traj):
        rospy.loginfo("Executing trajectory")
        dt = traj[0][-1]
        t0 = rospy.Time.now()
           
        recorded_motion = []

        for point in traj:
            
            slave_goal = geometry_msgs.msg.PoseStamped()

            slave_goal.pose.position.x = point[0]
            slave_goal.pose.position.y = point[1]
            slave_goal.pose.position.z = point[2]

            slave_goal.pose.orientation.w = 0.60
            slave_goal.pose.orientation.x = 0.77 
            slave_goal.pose.orientation.y = 0.11
            slave_goal.pose.orientation.z = -0.15

            slave_goal.header.seq = 0
        
            slave_goal.header.stamp = t0 + rospy.Duration(point[-1])
            slave_goal.header.frame_id = "/base_footprint"
            
            self.end_effector_goal_pub.publish(slave_goal)
            time.sleep(dt)

    def save_data(self, path, file_name, trajectory):
        
        with open(path+file_name, 'w+') as f:
            f.write(str(trajectory))
        os.chmod(path+file_name,stat.S_IRWXO)        
        os.chmod(path+file_name,stat.S_IRWXU)



if __name__ == '__main__':
    dmp_node = dmpNode()
    K = 100
    D = 2.0 * np.sqrt(K)
    dt = 0.1
    raw_file = "raw_trajectory_12.txt"

    trajectory_parser = trajectoryParser()
    storage_variables = trajectoryStorageVariables("/home/fmeccanici/Documents/thesis/lfd_ws/src/marco_lfd/data/open_loop/", 
                                                    "/home/fmeccanici/Documents/thesis/lfd_ws/src/marco_lfd/data/refined/",
                                                    "/home/fmeccanici/Documents/thesis/lfd_ws/src/marco_lfd/data/resampled/",
                                                    "/home/fmeccanici/Documents/thesis/lfd_ws/src/marco_lfd/data/raw/",
                                                        raw_file,
                                                        "/home/fmeccanici/Documents/thesis/lfd_ws/src/marco_lfd/data/new/"
                                                    )

    traj = trajectory_parser.parse(raw_file, storage_variables.raw_path, dt)

    # set goal position to last position in traj demonstration
    goal = traj[-1][:]    

    r = rospy.Rate(30)
    dmp_node.goToInitialPose()

    while True:
        # get only Cartesian positions
        traj = trajectory_parser.getCartesianPositions(traj)

        # visualize trajectory
        for i in range(10):
            dmp_node.publishMarkerArray(dmp_node.trajectory2markerArray(traj))
            rospy.Rate(30)
        # set goal position to last position in traj demonstration
        # goal = traj[-1][:]
        

        # execute open loop recording 
        dmp_variables = dmpVariables(traj, dt, K, D, goal, x_0=[0.403399335619, -0.430007534239, 1.16269467394])
        dmp_node.makePlan(dmp_variables)
        
        dmp_node.recordMotion(dmp_variables, storage_variables, 'open_loop')
        if dmp_node.grey_button_toggle == 1:
            dmp_node.grey_button_toggle = 0

        time.sleep(2)

        dmp_node.goToInitialPose()
        
        time.sleep(2)

        ref_traj = dmp_node.recordMotion(dmp_variables, storage_variables, 'refinement')
        ref_traj = dmp_node.getCartesianPositions(ref_traj)
        
        goal = ref_traj[-1][:]

        traj = dmp_node.determineRefinedTrajectory(storage_variables)
        
        if dmp_node.grey_button_toggle == 1:
            dmp_node.grey_button_toggle = 0
        time.sleep(2)

        dmp_node.goToInitialPose()
        
        r.sleep()
