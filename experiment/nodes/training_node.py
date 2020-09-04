import rospy
import numpy as np
from collections import deque

class TrainingNode(object):
    def __init__(self):
        rospy.init_node('experiment')
        self._getRosParameters()

        self.training_scores = deque([0,0,0,0])

    def _getRosParameters(self):
        self.method = rospy.get_param('~method')            

    def startTrial(self):
        self.openGripper()
        self.goToInitialPose()
        self.setDishwasherPosition()
        self.setObjectPosition()
        time.sleep(4)
        self.getContext()
        self.setDataLoggerParameters()
        self.predict()
        self.visualize('prediction')
        self.traffic_light_updater.update('red')
        self.text_updater.update("AUTONOMOUS EXECUTION")
        obstacle_hit, object_reached, object_kicked_over = self.executeTrajectory(self.prediction)
        
        # store prediction along with failure
        self.storeData(prediction=1, obstacle_hit=obstacle_hit, object_missed = not object_reached, object_kicked_over=object_kicked_over)
        self.saveData()
        
        # update text in operator gui
        if obstacle_hit or not object_reached or object_kicked_over:
            self.text_updater.update("FAILURE:")
        else:
            self.text_updater.update("SUCCESS!")
        
        if obstacle_hit:
            self.text_updater.append("OBSTACLE HIT")
        if not object_reached:
            self.text_updater.append("OBJECT MISSED")
        if object_kicked_over:
            self.text_updater.append("OBJECT KICKED OVER")

        if self.method == 'online+pendant':
            while np.mean(list(self.training_scores)) < 2: 
                print("Trajectory failure!")

                self.goToInitialPose()
                self.setDishwasherPosition()
                time.sleep(3)
                self.setObjectPosition()

                self.traffic_light_updater.update('green')
                # wait until the operator clicked the red or green button
                self.text_updater.update("REFINE RED OR GREEN?")
                # rospy.wait_for_message('operator_gui_interaction', OperatorGUIinteraction)
                # rospy.wait_for_message('keyboard_control', Keyboard)
                self.waitForKeyPress()

                self.stop_updating_flag = 0

                refine_trajectory = rospy.ServiceProxy('refine_trajectory', RefineTrajectory)
                
                if self.refinePrediction():

                    # we only need to start the timer if it is equal to zero, else just keep the timer running
                    if self.start_time == 0:
                        # start timer
                        self.startTimer()
                    else: pass

                    resp = refine_trajectory(self.prediction, self.T_desired)
                
                elif self.refineRefinement():

                    # we only need to start the timer if it is equal to zero, else just keep the timer running
                    if self.start_time == 0:
                        # start timer
                        self.startTimer()
                    else: pass

                    resp = refine_trajectory(self.refined_trajectory, self.T_desired)

                self.traffic_light_updater.update('red')

                self.refined_trajectory = resp.refined_trajectory
                with open('/home/fmeccanici/Documents/thesis/thesis_workspace/src/experiment/debug/refined_trajectory.txt', 'w+') as f:
                    f.write(str(self.refined_trajectory))
                    
                time.sleep(5)

                obstacle_hit = resp.obstacle_hit.data
                execution_failure = rospy.ServiceProxy('get_execution_failure', GetExecutionFailure)
                resp = execution_failure()
                object_reached = resp.object_reached.data
                object_kicked_over = resp.object_kicked_over.data

                print("\n")

                rospy.loginfo("object missed: " + str(not object_reached))
                rospy.loginfo("obstacle hit: " + str(obstacle_hit))
                rospy.loginfo("object kicked over: " + str(object_kicked_over))

                print("\n")

                self.stop_updating_flag = 1

                # update text in operator gui
                if obstacle_hit or not object_reached or object_kicked_over:
                    self.text_updater.update("FAILURE:")
                else:
                    self.text_updater.update("SUCCESS!")

                if obstacle_hit:
                    self.text_updater.append("OBSTACLE HIT")
                if not object_reached:
                    self.text_updater.append("OBJECT MISSED")
                if object_kicked_over:
                    self.text_updater.append("OBJECT KICKED OVER")

                time.sleep(2)
                # store refinement along with if it failed or not
                self.storeData(refinement=1, obstacle_hit=obstacle_hit, object_missed = not object_reached, object_kicked_over=object_kicked_over)
               
                number_of_refinements += 1

                # increment number of refinements
                rospy.wait_for_service('set_number_of_refinements', timeout=2.0)
                
                set_nr_refinement = rospy.ServiceProxy('set_number_of_refinements', SetNumberOfRefinements)
                set_nr_refinement(Byte(self.participant_number), Byte(number_of_refinements))

                rospy.loginfo("Got a refined trajectory")

                self.visualize('both')
                print("number of refinement = " + str(number_of_refinements))
                self.number_of_refinements_updater.update(str(number_of_refinements))

                if number_of_refinements >= self.max_refinements:
                    self.text_updater.update("MAX REFINEMENT AMOUNT REACHED!")
        
        elif self.method == 'offline+pendant':

            while np.mean(list(self.training_scores)) < 2: 
                self.goToInitialPose()
                self.setDishwasherPosition()
                time.sleep(3)
                self.setObjectPosition()

                rospy.wait_for_service('/offline_pendant/add_waypoint', timeout=2.0)
                add_waypoint = rospy.ServiceProxy('/offline_pendant/add_waypoint', AddWaypoint)
                add_waypoint()

                set_teach_state(Bool(True))

                self.traffic_light_updater.update('green')

                self.text_updater.update("START TEACHING")
                self.collision_updating_flag = 1

                # we only need to start the timer if it is equal to zero, else just keep the timer running
                if self.start_time == 0:
                    # start timer
                    self.startTimer()
                else: pass

                rospy.wait_for_service('offline_pendant/get_teach_state', timeout=2.0)
                get_teach_state = rospy.ServiceProxy('offline_pendant/get_teach_state', GetTeachState)
                resp = get_teach_state()
                isTeachingOffline = resp.teach_state.data      
                
                # use teach_pendant node to teach offline
                while isTeachingOffline:
                    resp = get_teach_state()
                    isTeachingOffline = resp.teach_state.data        

                self.traffic_light_updater.update('red')

                
                rospy.wait_for_service('get_demonstration_pendant', timeout=2.0)
                get_demo_pendant = rospy.ServiceProxy('get_demonstration_pendant', GetDemonstrationPendant)
                resp = get_demo_pendant()

                set_teach_state(Bool(False))

                # self.stopNode('teach_pendant.launch')
                
                self.goToInitialPose()
                self.setDishwasherPosition()
                time.sleep(3)
                self.setObjectPosition()

                self.refined_trajectory = resp.demo
                self.visualize('both')
                time.sleep(3)

                self.collision_updating_flag = 0

                self.stopTimer()
                obstacle_hit, object_reached, object_kicked_over = self.executeTrajectory(self.refined_trajectory)
                self.startTimer()
                
                with open('/home/fmeccanici/Documents/thesis/thesis_workspace/src/experiment/debug/refined_trajectory.txt', 'w+') as f:
                    f.write(str(self.refined_trajectory))
                    
                print("\n")

                rospy.loginfo("object missed: " + str(not object_reached))
                rospy.loginfo("obstacle hit: " + str(obstacle_hit))
                rospy.loginfo("object kicked over: " + str(object_kicked_over))

                print("\n")

                # update text in operator gui
                if obstacle_hit or not object_reached or object_kicked_over:
                    self.text_updater.update("FAILURE:")
                else:
                    self.text_updater.update("SUCCESS!")

                if obstacle_hit:
                    self.text_updater.append("OBSTACLE HIT")
                if not object_reached:
                    self.text_updater.append("OBJECT MISSED")
                if object_kicked_over:
                    self.text_updater.append("OBJECT KICKED OVER")

                time.sleep(2)
                # store refinement along with if it failed or not
                self.storeData(refinement=1, obstacle_hit=obstacle_hit, object_missed = not object_reached, object_kicked_over=object_kicked_over)
               
                # increment number of refinements
                rospy.wait_for_service('set_number_of_refinements', timeout=2.0)
                
                number_of_refinements += 1
                set_nr_refinement = rospy.ServiceProxy('set_number_of_refinements', SetNumberOfRefinements)
                set_nr_refinement(Byte(self.participant_number), Byte(number_of_refinements))

                rospy.loginfo("Got a refined trajectory")

                print("number of refinement = " + str(number_of_refinements))
                self.number_of_refinements_updater.update(str(number_of_refinements))

                if number_of_refinements >= self.max_refinements:
                    self.text_updater.update("MAX REFINEMENT AMOUNT REACHED!")
        
            rospy.wait_for_service('offline_pendant/clear_waypoints', timeout=2.0)
            clear_waypoints = rospy.ServiceProxy('offline_pendant/clear_waypoints', ClearWaypoints)
            clear_waypoints()
        
        elif self.method == 'online+omni':
            while np.mean(list(self.training_scores)) < 2: 
                print("Trajectory failure!")

                self.goToInitialPose()
                self.setDishwasherPosition()
                time.sleep(3)
                self.setObjectPosition()

                self.traffic_light_updater.update('green')
                # wait until the operator clicked the red or green button
                self.text_updater.update("REFINE RED OR GREEN?")
                self.waitForKeyPress()
                self.text_updater.update("PRESS WHITE BUTTON TO STOP REFINING")

                self.stop_updating_flag = 0

                refine_trajectory = rospy.ServiceProxy('refine_trajectory', RefineTrajectory)
                
                if self.refinePrediction():

                    # we only need to start the timer if it is equal to zero, else just keep the timer running
                    if self.start_time == 0:
                        # start timer
                        self.startTimer()
                    else: pass

                    resp = refine_trajectory(self.prediction, self.T_desired)
                
                elif self.refineRefinement():

                    # we only need to start the timer if it is equal to zero, else just keep the timer running
                    if self.start_time == 0:
                        # start timer
                        self.startTimer()
                    else: pass

                    resp = refine_trajectory(self.refined_trajectory, self.T_desired)
                
                self.traffic_light_updater.update('red')


                self.refined_trajectory = resp.refined_trajectory
                with open('/home/fmeccanici/Documents/thesis/thesis_workspace/src/experiment/debug/refined_trajectory.txt', 'w+') as f:
                    f.write(str(self.refined_trajectory))
                    
                time.sleep(5)

                obstacle_hit = resp.obstacle_hit.data
                execution_failure = rospy.ServiceProxy('get_execution_failure', GetExecutionFailure)
                resp = execution_failure()
                object_reached = resp.object_reached.data
                object_kicked_over = resp.object_kicked_over.data

                print("\n")

                rospy.loginfo("object missed: " + str(not object_reached))
                rospy.loginfo("obstacle hit: " + str(obstacle_hit))
                rospy.loginfo("object kicked over: " + str(object_kicked_over))

                print("\n")

                self.stop_updating_flag = 1

                # update text in operator gui
                if obstacle_hit or not object_reached or object_kicked_over:
                    self.text_updater.update("FAILURE:")
                else:
                    self.text_updater.update("SUCCESS!")

                if obstacle_hit:
                    self.text_updater.append("OBSTACLE HIT")
                if not object_reached:
                    self.text_updater.append("OBJECT MISSED")
                if object_kicked_over:
                    self.text_updater.append("OBJECT KICKED OVER")

                time.sleep(2)
                # store refinement along with if it failed or not
                self.storeData(refinement=1, obstacle_hit=obstacle_hit, object_missed = not object_reached, object_kicked_over=object_kicked_over)
               
                number_of_refinements += 1

                # increment number of refinements
                rospy.wait_for_service('set_number_of_refinements', timeout=2.0)
                
                set_nr_refinement = rospy.ServiceProxy('set_number_of_refinements', SetNumberOfRefinements)
                set_nr_refinement(Byte(self.participant_number), Byte(number_of_refinements))

                rospy.loginfo("Got a refined trajectory")

                self.visualize('both')
                print("number of refinement = " + str(number_of_refinements))
                self.number_of_refinements_updater.update(str(number_of_refinements))

                if number_of_refinements >= self.max_refinements:
                    self.text_updater.update("MAX REFINEMENT AMOUNT REACHED!")
        
        elif self.method == 'offline+omni':

            while np.mean(list(self.training_scores)) < 2: 
                self.goToInitialPose()
                self.setDishwasherPosition()
                time.sleep(3)
                self.setObjectPosition()

                rospy.wait_for_service('/set_part_to_publish', timeout=2.0)
                set_part_to_publish = rospy.ServiceProxy('/set_part_to_publish', SetPartToPublish)
                set_part_to_publish(String('both'))

                self.collision_updating_flag = 1

                # we only need to start the timer if it is equal to zero, else just keep the timer running
                if self.start_time == 0:
                    # start timer
                    self.startTimer()
                else: pass

                rospy.wait_for_service('trajectory_teaching/get_teach_state', timeout=2.0)
                get_teach_state = rospy.ServiceProxy('trajectory_teaching/get_teach_state', GetTeachState)
                resp = get_teach_state()
                isTeachingOffline = resp.teach_state.data      
                
                self.traffic_light_updater.update('green')

                while not isTeachingOffline:
                    self.text_updater.update("PRESS WHITE BUTTON TO START TEACHING")
                    resp = get_teach_state()
                    isTeachingOffline = resp.teach_state.data 

                # use teach_pendant node to teach offline
                while isTeachingOffline:
                    self.text_updater.update("PRESS WHITE BUTTON TO STOP TEACHING")

                    resp = get_teach_state()                
                    isTeachingOffline = resp.teach_state.data 

                self.traffic_light_updater.update('red')

                self.text_updater.update("STOPPED TEACHING")

                rospy.wait_for_service('trajectory_teaching/get_trajectory', timeout=2.0)
                get_demo = rospy.ServiceProxy('trajectory_teaching/get_trajectory', GetTrajectory)
                resp = get_demo()

                set_teach_state(Bool(False))

                while self.isArmEnabled():
                    self.text_updater.update("PRESS GREY BUTTON")
                
                self.text_updater.update("GREY BUTTON PRESSED")

                self.goToInitialPose()
                self.setDishwasherPosition()
                time.sleep(3)
                self.setObjectPosition()

                self.refined_trajectory = resp.demo
                self.visualize('both')
                time.sleep(3)

                self.collision_updating_flag = 0

                obstacle_hit, object_reached, object_kicked_over = self.executeTrajectory(self.refined_trajectory)
                

                with open('/home/fmeccanici/Documents/thesis/thesis_workspace/src/experiment/debug/refined_trajectory.txt', 'w+') as f:
                    f.write(str(self.refined_trajectory))
                    
                print("\n")

                rospy.loginfo("object missed: " + str(not object_reached))
                rospy.loginfo("obstacle hit: " + str(obstacle_hit))
                rospy.loginfo("object kicked over: " + str(object_kicked_over))

                print("\n")

                # update text in operator gui
                if obstacle_hit or not object_reached or object_kicked_over:
                    self.text_updater.update("FAILURE:")
                else:
                    self.text_updater.update("SUCCESS!")

                if obstacle_hit:
                    self.text_updater.append("OBSTACLE HIT")
                if not object_reached:
                    self.text_updater.append("OBJECT MISSED")
                if object_kicked_over:
                    self.text_updater.append("OBJECT KICKED OVER")

                time.sleep(2)
                # store refinement along with if it failed or not
                self.storeData(refinement=1, obstacle_hit=obstacle_hit, object_missed = not object_reached, object_kicked_over=object_kicked_over)
               
                # increment number of refinements
                rospy.wait_for_service('set_number_of_refinements', timeout=2.0)
                
                number_of_refinements += 1
                set_nr_refinement = rospy.ServiceProxy('set_number_of_refinements', SetNumberOfRefinements)
                set_nr_refinement(Byte(self.participant_number), Byte(number_of_refinements))

                rospy.loginfo("Got a refined trajectory")

                print("number of refinement = " + str(number_of_refinements))
                self.number_of_refinements_updater.update(str(number_of_refinements))
                
                rospy.wait_for_service('trajectory_teaching/clear_trajectory', timeout=2.0)
                clear_trajectory = rospy.ServiceProxy('trajectory_teaching/clear_trajectory', ClearTrajectory)
                resp = clear_trajectory()

                if number_of_refinements >= self.max_refinements:
                    self.text_updater.update("MAX REFINEMENT AMOUNT REACHED!")
        
            rospy.wait_for_service('trajectory_teaching/clear_trajectory', timeout=2.0)
            clear_trajectory = rospy.ServiceProxy('trajectory_teaching/clear_trajectory', ClearTrajectory)
            clear_trajectory()
        
        self.number_of_refinements_updater.update(str(0))
        
        self.stopTimer()
        
        # store time
        self.storeData(time=True)
        
        ###### save data ######
        self.saveData()
        self.zeroTimer()

        self.current_trial += 1

    def run(self):
