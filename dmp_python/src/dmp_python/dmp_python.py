#!/usr/bin/env python

from dmp.srv import *
from dmp.msg import *
import rospy

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
            
        print("Starting LfD...")
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj, k_gains, d_gains, num_bases)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        print("LfD done"    )
                
        return resp

    #Set a DMP as active for planning
    def _makeSetActiveRequest(self, dmp_list):
        try:
            sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    #Generate a plan from a DMP
    def _makePlanRequest(self, x_0, x_dot_0, t_0, goal, goal_thresh, 
                        seg_length, tau, dt, integrate_iter):
        print("Starting DMP planning...")
        rospy.wait_for_service('get_dmp_plan')
        try:
            gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
            resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        print("DMP planning done")   
                
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