#!/usr/bin/env python

from dmp.srv import *
from dmp.msg import *
import rospy

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

class dmpPython():
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
        plan = self._makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                            seg_length, tau, dt, integrate_iter)

        return plan