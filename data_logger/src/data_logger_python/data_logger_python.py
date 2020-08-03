#!/usr/bin/env python3.5

import os, csv, ast, copy
import pandas as pd
from learning_from_demonstration_python.trajectory_parser import trajectoryParser

class ParticipantData(object):
    def __init__(self, number, gender, age, path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/'):

        self.number = number
        self.num_methods = 4
        self.methods = {}
        self.parser = trajectoryParser()

        # check if path exists, create if not
        self.path = path + 'participant_' + str(number) + '/'
        if not os.path.exists(self.path):
            os.makedirs(self.path)
            self.gender = gender
            self.age = age

            self.initData()

        # if it exists --> load data 
        elif os.path.exists(self.path) and os.path.isfile(self.path+'data.txt'):
            self.loadData()
        else:
            self.gender = gender
            self.age = age
            self.initData()

        # parameters used to fill the methods dictionary
        self.predicted_trajectory = {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [], 'object_missed': False, 'obstacle_hit': False, 'object_kicked_over': False, 'success': True}
        self.refined_trajectory = {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [], 'object_missed': False, 'obstacle_hit': False, 'object_kicked_over': False, 'success': True}
        self.trial = 1
        self.object_position = 1
        self.context = []
        self.method = 1

        # for i in range(self.num_variations):
        #     self.variations[i+1] = self.object_positions

        # 1 = Online + Teleop
        # 2 = Ofline + Teleop
        # 3 = Online + Pendant
        # 4 = Offline + Pendant
        # for i in range(self.num_methods):
        #     self.methods[i+1] = self.variations
    
    # create large empty dictionary containing the data
    def initData(self):
        self.object_positions = {}
        self.methods = {}
        self.trials = {}

        self.num_methods = 4
        self.num_object_positions = 6
        self.num_trials = 5

        for i in range(self.num_trials):
            self.trials[i+1] = {
                'predicted_trajectory': {}, 'context': [],
                'refined_trajectory': {},
                'number_of_refinements': 0,
                'time': 0
            }

        for j in range(self.num_object_positions):
            self.object_positions[j+1] = {
            'trial': copy.deepcopy(self.trials)
            }
        
        for i in range(self.num_methods):
            self.methods[i+1] = {
                'object_position': copy.deepcopy(self.object_positions)            }

    def loadData(self):
        data_path = self.path + 'data.txt'
        print(data_path)
        
        with open(data_path, "r") as infile:
            outfile = ast.literal_eval(infile.read())

            self.gender = outfile['gender']
            self.number = outfile['number']
            self.age = outfile['age']
            
            for i in range(self.num_methods):
                self.methods[i+1] = outfile['method'][i+1]

    def getMethods(self):
        return self.methods

    def setName(self, name):
        self.name = name
    
    def getName(self, name):
        return self.name

    def setGender(self, gender):
        self.gender = gender
   
    def getGender(self, gender):
        return self.gender
    
    def setAge(self, age):
        self.age = age

    def getAge(self, age):
        return self.age

    def setStoragePath(self, path):
        # check if path exists, create if not
        self.path = path + 'participant_' + str(self.number) + '/'
        if not os.path.exists(self.path):
            os.makedirs(self.path)

    def getStoragePath(self):
        return self.path

    def setContext(self, context):
        self.context = context
    
    def getContext(self):
        return self.context

    def setTrial(self, trial):
        self.trial = trial
    
    def getTrial(self):
        return self.trial

    def setObjectPosition(self, object_position):
        self.object_position = object_position

    def setMethod(self, method):
        self.method = method

    def setTime(self, *args, **kwargs):
        time = kwargs["time"]

        self.methods[self.method]['object_position'][self.object_position]['trial'][self.trial]['time'] = copy.deepcopy(time)

    def setRefinedTrajectory(self, *args, **kwargs):

        if "refinement" in kwargs:            
            refinement = kwargs["refinement"]
            trajectory = refinement.poses
            t = list(refinement.times)
            object_missed = kwargs["object_missed"]
            object_kicked_over = kwargs["object_kicked_over"]
            obstacle_hit = kwargs["obstacle_hit"]
            success = kwargs["success"]

            x = []
            y = []
            z = []
            qx = []
            qy = []
            qz = []
            qw = []

            for datapoint in trajectory:
                x.append(float(datapoint.position.x))
                y.append(float(datapoint.position.y))
                z.append(float(datapoint.position.z))
                qx.append(float(datapoint.orientation.x))
                qy.append(float(datapoint.orientation.y))
                qz.append(float(datapoint.orientation.z))
                qw.append(float(datapoint.orientation.w))

            self.refined_trajectory['x'] = x
            self.refined_trajectory['y'] = y
            self.refined_trajectory['z'] = z
            self.refined_trajectory['qx'] = qx
            self.refined_trajectory['qy'] = qy
            self.refined_trajectory['qz'] = qz
            self.refined_trajectory['qw'] = qw
            self.refined_trajectory['t'] = t
            self.refined_trajectory['object_missed'] = object_missed
            self.refined_trajectory['object_kicked_over'] = object_kicked_over
            self.refined_trajectory['obstacle_hit'] = obstacle_hit
            self.refined_trajectory['success'] = success
        
            trajectory = copy.deepcopy(self.refined_trajectory)
            self.methods[self.method]['object_position'][self.object_position]['trial'][self.trial]['refined_trajectory'] = trajectory

            return 0
                
    def setPredictedTrajectory(self, *args, **kwargs):

        if "prediction" in kwargs and "object_missed" in kwargs and "obstacle_hit" in kwargs:            
            prediction = kwargs["prediction"]

            # time = kwargs["time"]
            context = self.parser.point_to_list(prediction.object_position)
            trajectory = prediction.poses
            t = list(prediction.times)
            object_missed = kwargs["object_missed"]
            obstacle_hit = kwargs["obstacle_hit"]
            object_kicked_over = kwargs["object_kicked_over"]
            success = kwargs["success"]

            x = []
            y = []
            z = []
            qx = []
            qy = []
            qz = []
            qw = []

            for datapoint in trajectory:
                x.append(float(datapoint.position.x))
                y.append(float(datapoint.position.y))
                z.append(float(datapoint.position.z))
                qx.append(float(datapoint.orientation.x))
                qy.append(float(datapoint.orientation.y))
                qz.append(float(datapoint.orientation.z))
                qw.append(float(datapoint.orientation.w))

            self.predicted_trajectory['x'] = x
            self.predicted_trajectory['y'] = y
            self.predicted_trajectory['z'] = x
            self.predicted_trajectory['qx'] = qx
            self.predicted_trajectory['qy'] = qy
            self.predicted_trajectory['qz'] = qz
            self.predicted_trajectory['qw'] = qw
            self.predicted_trajectory['t'] = t
            self.predicted_trajectory['object_missed'] = object_missed
            self.predicted_trajectory['object_kicked_over'] = object_kicked_over
            self.predicted_trajectory['obstacle_hit'] = obstacle_hit
            self.predicted_trajectory['success'] = success

            trajectory = copy.deepcopy(self.predicted_trajectory)

            

            self.methods[self.method]['object_position'][self.object_position]['trial'][self.trial]['predicted_trajectory'] = trajectory
            self.methods[self.method]['object_position'][self.object_position]['trial'][self.trial]['context'] = context
            # self.methods[self.method]['object_position'][self.object_position]['trial'][self.trial]['time'] = time

            print("Prediction stored in dictionary")
            
    def setObstaclesHit(self):
        self.methods[self.method]['object_position'][self.object_position]['trial'][self.trial]['obstacle_hit'] = True
        self.methods[self.method]['object_position'][self.object_position]['trial'][self.trial]['success'] = False

        # self.methods[method]['obstacles_hit'] += 1
 
    def setObjectMissed(self):

        self.methods[self.method]['object_position'][self.object_position]['trial'][self.trial]['object_missed'] = True
        self.methods[self.method]['object_position'][self.object_position]['trial'][self.trial]['success'] = False

        # self.methods[method]['object_missed'] += 1

    def incrementNumberOfRefinements(self):
        self.methods[self.method]['object_position'][self.object_position]['trial'][self.trial]['number_of_refinements'] += 1
        print("method " + str(self.method))
        print("object position " + str(self.object_position))
        print("trial " + str(self.trial))
        print("number of refinements incremented to " + str(self.methods[self.method]['object_position'][self.object_position]['trial'][self.trial]['number_of_refinements']))

    def incrementNumberOfUpdates(self, method):
        self.methods[method]['number_of_updates'] += 1

    def setNumberOfUpdates(self, method, value):
        self.methods[method]['number_of_updates'] = value
    
    def toCSV(self):
        data = {'number': self.number, 'age': self.age, 'gender': self.gender, 'method': self.methods}
        with open(self.path + 'data.txt', 'w+') as f:
            f.write(str(data))
            
            
if __name__ == "__main__":
    participant1_data = ParticipantData(1, 'm', 26)
    participant1_data.incrementObjectMissed(method=1)
    participant1_data.incrementObstaclesHit(method=1)
    participant1_data.setPredictedTrajectory(method=1, from_file=True)
    participant1_data.toCSV()
