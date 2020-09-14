#! /usr/bin/env python2.7

import rospy
from data_logger_python.data_logger_python import ParticipantData
from data_logger.srv import (CreateParticipant, CreateParticipantResponse, AddRefinement, AddRefinementResponse,
                                SetPrediction, SetPredictionResponse, SetObjectMissed, SetObjectMissedResponse,
                                SetObstaclesHit, SetObstaclesHitResponse, ToCsv, ToCsvResponse,
                                IncrementNumberOfRefinements, IncrementNumberOfRefinementsResponse,
                                SetParameters, SetParametersResponse, SetTime, SetTimeResponse, 
                                SetNumberOfRefinements, SetNumberOfRefinementsResponse)

from learning_from_demonstration.srv import GetContext
import copy
from std_srvs.srv import EmptyResponse

class DataLoggerNode(object):
    def __init__(self):
        rospy.init_node('data_logger')

        self._create_participant_service = rospy.Service('create_participant', CreateParticipant, self._createParticipant)
        self._add_refinement_service = rospy.Service('add_refinement', AddRefinement, self._addRefinement)
        self._set_prediction_service = rospy.Service('set_prediction', SetPrediction, self._setPredicted)
        
        self._set_object_missed_service = rospy.Service('set_object_missed', SetObjectMissed, self._setObjectMissed)
        self._set_obstacles_hit_service = rospy.Service('set_obstacles_hit', SetObstaclesHit, self._setObstaclesHit)


        self._increment_number_of_refinements_service = rospy.Service('increment_number_of_refinements', IncrementNumberOfRefinements, self._incrementNumberOfRefinements)
        self._set_number_of_refinements_service = rospy.Service('set_number_of_refinements', SetNumberOfRefinements, self._setNumberOfRefinements)

        # self._set_number_of_updates_service = rospy.Service('set_number_of_updates', SetNumberOfUpdates, self._setNumberOfUpdates)
        self._to_csv_service = rospy.Service('to_csv', ToCsv, self._toCsv)
        self._set_time_service = rospy.Service('data_logger/set_time', SetTime, self._setTime)

        self._set_parameters_service = rospy.Service('data_logger/set_parameters', SetParameters, self._setParameters)

        self.data = {}

    def _createParticipant(self, req):
        rospy.loginfo("Creating participant using service")
        number = req.number.data
        age = req.age.data
        gender = req.gender.data
        field_of_study = req.field_of_study.data

        teleop_experience = req.teleop_experience.data
        keyboard_experience = req.keyboard_experience.data
        left_right_handed = req.left_right_handed.data

        num_objects = 3

        self.participant_data = ParticipantData(number, gender, age, field_of_study, teleop_experience, keyboard_experience, left_right_handed, num_objects)
        self.data[number] = copy.deepcopy(self.participant_data)

        resp = CreateParticipantResponse()
        rospy.loginfo("loaded/created participant " + str(number) + ": " + str(self.data[number]))

        return resp
    
    def _setParameters(self, req):
        number = req.number.data
        object_position = req.object_position.data
        trial = req.trial.data
        method = req.method.data
        model = req.model.data

        self.data[number].setMethod(method)
        self.data[number].setTrial(trial)
        self.data[number].setObjectPosition(object_position)
        self.data[number].setModel(model)

        rospy.loginfo("Data logger parameters set to: ")
        rospy.loginfo("model " + str(model))
        rospy.loginfo("method " + str(method))
        rospy.loginfo("object position " + str(object_position))
        rospy.loginfo("trial " + str(trial))

        resp = SetParametersResponse()
        
        return resp       
    
    def _setTime(self, req):
        time = req.time.data
        number = req.number.data
        rospy.loginfo("Setting time using service: t = " + str(time))

        self.data[number].setTime(time=time)

        resp = SetTimeResponse()
        return resp

    def _addRefinement(self, req):
        rospy.loginfo("Adding refinement using service")
        number = req.number.data
        refinement = req.trajectory_data.trajectory
        object_missed = req.trajectory_data.object_missed.data
        object_kicked_over = req.trajectory_data.object_kicked_over.data
        obstacle_hit = req.trajectory_data.obstacle_hit.data

        if object_missed or obstacle_hit or object_kicked_over:
            success = False
        else:
            success = True

        self.data[number].setRefinedTrajectory(refinement=refinement,
                                                object_missed=object_missed, object_kicked_over=object_kicked_over,
                                                obstacle_hit=obstacle_hit,
                                                success=success)
    
        resp = AddRefinementResponse()
        return resp

    def _setPredicted(self, req):
        rospy.loginfo("Set predicted trajectory using service")

        number = req.number.data
        prediction = req.trajectory_data.trajectory
        object_missed = req.trajectory_data.object_missed.data
        object_kicked_over = req.trajectory_data.object_kicked_over.data
        obstacle_hit = req.trajectory_data.obstacle_hit.data

        if object_missed or obstacle_hit or object_kicked_over:
            success = False
        else:
            success = True

        try:
            rospy.wait_for_service('get_context', timeout=2.0)

            get_context = rospy.ServiceProxy('get_context', GetContext)
            resp = get_context()
            context = [resp.context.x, resp.context.y, resp.context.z]
            self.participant_data.setContext(context)

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)       

        self.data[number].setPredictedTrajectory(prediction=prediction, 
                                                object_missed=object_missed,
                                                obstacle_hit=obstacle_hit, success=success,
                                                object_kicked_over=object_kicked_over)


        resp = SetPredictionResponse()
        return resp

    def _setObstaclesHit(self, req):
        number = req.number.data
        # condition = req.condition.data
        self.data[number].setObstaclesHit()

        rospy.loginfo("Set obstacle hit")
        resp = SetObstaclesHitResponse()
        return resp

    def _setObjectMissed(self, req):
        number = req.number.data
        # condition = req.condition.data
        self.data[number].setObjectMissed()
        rospy.loginfo("Set object miss")

        resp = SetObjectMissedResponse()
        return resp

    def _incrementNumberOfRefinements(self, req):
        number = req.number.data 
        self.data[number].incrementNumberOfRefinements()

        resp = IncrementNumberOfRefinementsResponse()
        return resp

    def _setNumberOfRefinements(self, req):
        number = req.number.data 
        number_of_refinements = req.number_of_refinements.data

        self.data[number].setNumberOfRefinements(number_of_refinements)

        resp = SetNumberOfRefinementsResponse()
        return resp

    # def _setNumberOfUpdates(self, req):
    #     number = req.number.data
    #     condition = req.condition.data
    #     number_of_updates = req.number_of_updates.data

    #     self.data[number].setNumberOfUpdates(condition=condition, value=number_of_updates)

    #     resp = SetNumberOfUpdatesResponse()
    #     return resp

    def _toCsv(self, req):
        rospy.loginfo("Converting data to csv using service")
        number = req.number.data
        self.data[number].toCSV()

        resp = ToCsvResponse()
        return resp

    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    node = DataLoggerNode()
    node.run()