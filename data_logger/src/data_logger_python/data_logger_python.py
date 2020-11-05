#!/usr/bin/env python3.5

import os, csv, ast, copy
import pandas as pd
from learning_from_demonstration_python.trajectory_parser import trajectoryParser
from experiment_variables.experiment_variables import ExperimentVariables

class ParticipantData(object):
    def __init__(self, number, gender, age, field_of_study, teleop_experience, keyboard_experience, left_right_handed, num_object_positions=6, num_trials=5, path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/'):

        self.experiment_variables = ExperimentVariables()        
        self.number = number
        self.num_methods = self.experiment_variables.num_methods
        self.num_models = self.experiment_variables.num_models
        self.methods = {}
        self.num_object_positions = self.experiment_variables.num_object_positions
        self.num_trials = self.experiment_variables.num_trials
        
        self.num_training_trials = self.experiment_variables.num_training_trials

        self.parser = trajectoryParser()

        # check if path exists, create if not
        self.path = path + 'participant_' + str(number) + '/'

        if not os.path.exists(self.path):
            os.makedirs(self.path)
            self.gender = gender
            self.age = age
            self.field_of_study = field_of_study
            self.teleop_experience = teleop_experience
            self.keyboard_experience = keyboard_experience
            self.left_right_handed = left_right_handed

            self.initData()

        # if it exists --> load data 
        elif os.path.exists(self.path) and os.path.isfile(self.path+'data.txt'):
            self.loadData()
        else:
            self.gender = gender
            self.age = age
            self.field_of_study = field_of_study
            self.teleop_experience = teleop_experience
            self.keyboard_experience = keyboard_experience
            self.left_right_handed = left_right_handed
            
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
        self.models = {}

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
        
        for i in range(self.num_models):
            self.models[i+1] = {
                'object_position': copy.deepcopy(self.object_positions)
            }

        for i in range(self.num_methods):
            self.methods[i+1] = {
                'model': copy.deepcopy(self.models)            }

    def loadData(self):
        data_path = self.path + 'data.txt'
        
        with open(data_path, "r") as infile:
            outfile = ast.literal_eval(infile.read())

            self.number = outfile['number']
            self.gender = outfile['gender']
            self.age = outfile['age']
            self.field_of_study = outfile['field_of_study']
            self.teleop_experience = outfile['teleop_experience']
            self.keyboard_experience = outfile['keyboard_experience']
            self.left_right_handed = outfile['left_right_handed']

            for i in range(self.num_methods):
                self.methods[i+1] = outfile['method'][i+1]
        
    def readTlx(self):
        tlx_file = self.path + 'tlx.csv'
        self.tlx_data = pd.read_csv(tlx_file)

    def getFieldOfStudy(self):
        return self.field_of_study

    def getTeleopExperience(self):
        return self.teleop_experience
    
    def getKeyboardExperience(self):
        return self.keyboard_experience

    def getMethods(self):
        return self.methods

    def setName(self, name):
        self.name = name
    
    def getName(self, name):
        return self.name

    def setGender(self, gender):
        self.gender = gender
   
    def getGender(self):
        return self.gender
    
    def getRightHanded(self):
        return self.left_right_handed
        
    def setAge(self, age):
        self.age = age

    def getAge(self):
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

    def setModel(self, model):
        self.model = model

    def setMethod(self, method):
        self.method = method

    def setTime(self, *args, **kwargs):
        time = kwargs["time"]

        self.methods[self.method]['model'][self.model]['object_position'][self.object_position]['trial'][self.trial]['time'] = copy.deepcopy(time)
    
    def getNumber(self):
        return self.number

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
            self.methods[self.method]['model'][self.model]['object_position'][self.object_position]['trial'][self.trial]['refined_trajectory'] = trajectory

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

            

            self.methods[self.method]['model'][self.model]['object_position'][self.object_position]['trial'][self.trial]['predicted_trajectory'] = trajectory
            self.methods[self.method]['model'][self.model]['object_position'][self.object_position]['trial'][self.trial]['context'] = context
            # self.methods[self.method]['model'][self.model]['object_position'][self.object_position]['trial'][self.trial]['time'] = time

            print("Prediction stored in dictionary")
            
    def setObstaclesHit(self):
        self.methods[self.method]['model'][self.model]['object_position'][self.object_position]['trial'][self.trial]['obstacle_hit'] = True
        self.methods[self.method]['model'][self.model]['object_position'][self.object_position]['trial'][self.trial]['success'] = False

        # self.methods[method]['obstacles_hit'] += 1
 
    def setObjectMissed(self):

        self.methods[self.method]['model'][self.model]['object_position'][self.object_position]['trial'][self.trial]['object_missed'] = True
        self.methods[self.method]['model'][self.model]['object_position'][self.object_position]['trial'][self.trial]['success'] = False

        # self.methods[method]['object_missed'] += 1

    def incrementNumberOfRefinements(self):
        self.methods[self.method]['model'][self.model]['object_position'][self.object_position]['trial'][self.trial]['number_of_refinements'] += 1

    def setNumberOfRefinements(self, number_of_refinements):
        self.methods[self.method]['model'][self.model]['object_position'][self.object_position]['trial'][self.trial]['number_of_refinements'] = number_of_refinements
        print('Set number of refinements to ' + str(self.methods[self.method]['model'][self.model]['object_position'][self.object_position]['trial'][self.trial]['number_of_refinements']))
    
    def incrementNumberOfUpdates(self, method):
        self.methods[method]['number_of_updates'] += 1

    def setNumberOfUpdates(self, method, value):
        self.methods[method]['number_of_updates'] = value
    
    def toCSV(self):
        data = {'number': self.number, 'age': self.age, 'gender': self.gender, 'field_of_study': self.field_of_study, 'teleop_experience': self.teleop_experience, 'keyboard_experience': self.keyboard_experience, 'left_right_handed': self.left_right_handed, 'method': self.methods}
        with open(self.path + 'data.txt', 'w+') as f:
            f.write(str(data))

    def getDict(self):
        data = {'number': self.number, 'age': self.age, 'gender': self.gender, 'field_of_study': self.field_of_study, 'teleop_experience': self.teleop_experience, 'keyboard_experience': self.keyboard_experience, 'left_right_handed': self.left_right_handed, 'method': self.methods}
        return data     
            
if __name__ == "__main__":
    participant1_data = ParticipantData(1, 'm', 26)
    participant1_data.incrementObjectMissed(method=1)
    participant1_data.incrementObstaclesHit(method=1)
    participant1_data.setPredictedTrajectory(method=1, from_file=True)
    participant1_data.toCSV()
