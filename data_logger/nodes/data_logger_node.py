#! /usr/bin/env python2.7

import rospy
from data_logger_python.data_logger_python import ParticipantData
from data_logger.srv import (CreateParticipant, CreateParticipantResponse, AddRefinement, AddRefinementResponse,
                                SetPrediction, SetPredictionResponse, IncrementObjectMissed, IncrementObjectMissedResponse,
                                IncrementObstaclesHit, IncrementObstaclesHitResponse, IncrementNumberOfUpdates, IncrementNumberOfUpdatesResponse,
                                SetNumberOfUpdates, SetNumberOfUpdatesResponse, ToCsv, ToCsvResponse)

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
        number = req.number
        age = req.age
        sex = req.sex

        self.participant_data = ParticipantData(number, sex, age)
        self.data[number] = self.participant_data

        resp = CreateParticipantResponse()

        return resp

    def _addRefinement(self, req):
        number = req.number
        condition = req.condition
        self.data[number].addRefinedTrajectory(from_file=1, condition=condition)
    
        resp = AddRefinementResponse()
        return resp

    def _setPredicted(self, req):
        number = req.number
        condition = req.condition
        self.data[number].setPredictedTrajectory(from_file=1, condition=condition)

        resp = SetPredictionResponse()
        return resp

    def _incrementObstaclesHit(self, req):
        number = req.number 
        condition = req.condition
        self.data[number].incrementObstaclesHit(condition=condition)

        resp = IncrementObstaclesHitResponse()
        return resp

    def _incrementObjectMissed(self, req):
        number = req.number 
        condition = req.condition
        self.data[number].incrementObjectMissed(condition=condition)

        resp = IncrementObjectMissedResponse()
        return resp

    def _incrementNumberOfUpdates(self, req):
        number = req.number 
        condition = req.condition
        self.data[number].incrementNumberOfUpdates(condition=condition)

        resp = IncrementNumberOfUpdatesResponse()
        return resp

    def _setNumberOfUpdates(self, req):
        number = req.number 
        condition = req.condition
        number_of_updates = req.number_of_updates

        self.data[number].setNumberOfUpdates(condition=condition, value=number_of_updates)

        resp = SetNumberOfUpdatesResponse()
        return resp

    def _toCsv(self, req):
        number = req.number 
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