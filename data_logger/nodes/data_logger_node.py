#! /usr/bin/env python2.7

import rospy
from data_logger_python.data_logger_python import ParticipantData
from data_logger.srv import (CreateParticipant, CreateParticipantResponse, AddRefinement, AddRefinementResponse,
                                SetPrediction, SetPredictionResponse, IncrementObjectMissed, IncrementObjectMissedResponse,
                                IncrementObstaclesHit, IncrementObstaclesHitResponse, IncrementNumberOfUpdates, IncrementNumberOfUpdatesResponse,
                                SetNumberOfUpdates, SetNumberOfUpdatesResponse, ToCsv, ToCsvResponse)

from learning_from_demonstration.srv import GetContext

class DataLoggerNode(object):
    def __init__(self):
        rospy.init_node('data_logger')

        self._create_participant_service = rospy.Service('create_participant', CreateParticipant, self._createParticipant)
        self._add_refinement_service = rospy.Service('add_refinement', AddRefinement, self._addRefinement)
        self._set_prediction_service = rospy.Service('set_prediction', SetPrediction, self._setPredicted)
        self._increment_object_missed_service = rospy.Service('increment_object_missed', IncrementObjectMissed, self._incrementObjectMissed)
        self._increment_obstacles_hit_service = rospy.Service('increment_obstacles_hit', IncrementObstaclesHit, self._incrementObstaclesHit)
        self._increment_number_of_updates_service = rospy.Service('increment_number_of_updates', IncrementNumberOfUpdates, self._incrementNumberOfUpdates)
        self._set_number_of_updates_service = rospy.Service('set_number_of_updates', SetNumberOfUpdates, self._setNumberOfUpdates)
        self._to_csv_service = rospy.Service('to_csv', ToCsv, self._toCsv)

        self.data = {}

    def _createParticipant(self, req):
        rospy.loginfo("Creating participant using service")
        number = req.number.data
        age = req.age.data
        sex = req.sex.data

        self.participant_data = ParticipantData(number, sex, age)
        self.data[number] = self.participant_data

        resp = CreateParticipantResponse()
        print(number)

        return resp

    def _addRefinement(self, req):
        rospy.loginfo("Adding refinement using service")
        number = req.number.data
        context = req.context
        object_position = req.object_position.data
        variation = req.variation.data
        time = req.time.data
        trial = req.trial.data

        self.data[number].setRefinedTrajectory(from_file=1, context=context, variation=variation,
                                                object_position=object_position, 
                                                time=time, trial=trial)
    
        resp = AddRefinementResponse()
        return resp

    def _setPredicted(self, req):
        rospy.loginfo("Set predicted trajectory using service")
        
        number = req.number.data
        context = req.context
        object_position = req.object_position.data
        variation = req.variation.data
        time = req.time.data
        trial = req.trial.data

        try:
            rospy.wait_for_service('get_context', timeout=2.0)

            get_context = rospy.ServiceProxy('get_context', GetContext)
            resp = get_context()
            context = [resp.context.x, resp.context.y, resp.context.z]
            self.participant_data.setContext(context)

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)       

        self.data[number].setPredictedTrajectory(from_file=1, context=context, variation=variation,
                                                object_position=object_position, 
                                                time=time, trial=trial)


        resp = SetPredictionResponse()
        return resp

    def _incrementObstaclesHit(self, req):
        number = req.number.data
        condition = req.condition.data
        self.data[number].incrementObstaclesHit(condition=condition)

        resp = IncrementObstaclesHitResponse()
        return resp

    def _incrementObjectMissed(self, req):
        number = req.number.data
        condition = req.condition.data
        self.data[number].incrementObjectMissed(condition=condition)

        resp = IncrementObjectMissedResponse()
        return resp

    def _incrementNumberOfUpdates(self, req):
        number = req.number.data 
        condition = req.condition.data
        self.data[number].incrementNumberOfUpdates(condition=condition)

        resp = IncrementNumberOfUpdatesResponse()
        return resp

    def _setNumberOfUpdates(self, req):
        number = req.number.data
        condition = req.condition.data
        number_of_updates = req.number_of_updates.data

        self.data[number].setNumberOfUpdates(condition=condition, value=number_of_updates)

        resp = SetNumberOfUpdatesResponse()
        return resp

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