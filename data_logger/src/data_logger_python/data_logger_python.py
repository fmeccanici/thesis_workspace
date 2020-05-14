#!/usr/bin/env python3.5

import os, csv
import pandas as pd

class ParticipantData(object):
    def __init__(self, number, gender, age, path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/'):

        self.number = number
        self.gender = gender
        self.age = age
        self.conditions = {}
        
        for i in range(3):
            self.conditions[i+1] = { 
                            'predicted_trajectory': {'before': {}, 'after': {}, 'number_of_updates': 0}, 
                            'refined_trajectories': {}, 'number_of_refinements': 0,
                            'object_missed': 0, 'obstacles_hit': 0}

        self.predicted_trajectory = {'trajectory': {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [] }, 'context': [], 'forces': []}        
        self.refined_trajectory = {'trajectory': {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [] }, 'context': [], 'forces': []}

        # check if path exists, create if not
        self.path = path + 'participant_' + str(number) + '/'
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        
            # for i in range(3):
            #     os.makedirs(self.path + 'condition_' + str(i+1) + '/')


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

    def addRefinedTrajectory(self, *args, **kwargs):
        if "from_file" in kwargs and kwargs["from_file"] == 1 and "context" in kwargs and "condition" in kwargs:
            condition = kwargs["condition"]
            context = kwargs["context"]

            path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/gui/data/experiment/'
            file_name = 'refined_trajectory.csv'
            with open(path+file_name, 'r') as f:
                reader = csv.reader(f)
                data = list(reader)
                x = []
                y = []
                z = []
                qx = []
                qy = []
                qz = []
                qw = []
                t = []

                for datapoint in data[1:]:
                    x.append(float(datapoint[0]))
                    y.append(float(datapoint[1]))
                    z.append(float(datapoint[2]))
                    qx.append(float(datapoint[3]))
                    qy.append(float(datapoint[4]))
                    qz.append(float(datapoint[5]))
                    qw.append(float(datapoint[6]))
                    t.append(float(datapoint[7]))

            self.refined_trajectory['trajectory']['x'] = x
            self.refined_trajectory['trajectory']['y'] = y
            self.refined_trajectory['trajectory']['z'] = z
            self.refined_trajectory['trajectory']['qx'] = qx
            self.refined_trajectory['trajectory']['qy'] = qy
            self.refined_trajectory['trajectory']['qz'] = qz
            self.refined_trajectory['trajectory']['qw'] = qw
            self.refined_trajectory['trajectory']['t'] = t
            self.refined_trajectory['context'] = [context.x, context.y, context.z]

            # append dictionary
            # we start with 0 refinement --> n + 1
            n = self.conditions[condition]['number_of_refinements'] + 1
            print(n)
            self.conditions[condition]['refined_trajectories'][n] = self.refined_trajectory          
            
            # increment number of refined trajectories
            self.conditions[condition]['number_of_refinements'] += 1

            return 0

    def getRefinedTrajectories(self, condition):
        return self.refined_trajectories[condition]['trajectories']
                
    def setPredictedTrajectory(self, *args, **kwargs):
        if "from_file" in kwargs and kwargs["from_file"] == 1 and "condition" in kwargs and "context" in kwargs and "before_after" in kwargs and "num_updates" in kwargs:
            path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/gui/data/experiment/'
            file_name = 'predicted_trajectory.csv'
            
            condition = kwargs["condition"]
            context = kwargs["context"]
            before_after = kwargs["before_after"]
            num_updates = kwargs["num_updates"]


            with open(path+file_name, 'r') as f:
                reader = csv.reader(f)
                data = list(reader)
                x = []
                y = []
                z = []
                qx = []
                qy = []
                qz = []
                qw = []
                t = []

                for datapoint in data[1:]:
                    x.append(float(datapoint[0]))
                    y.append(float(datapoint[1]))
                    z.append(float(datapoint[2]))
                    qx.append(float(datapoint[3]))
                    qy.append(float(datapoint[4]))
                    qz.append(float(datapoint[5]))
                    qw.append(float(datapoint[6]))
                    t.append(float(datapoint[7]))

            self.predicted_trajectory['trajectory']['x'] = x
            self.predicted_trajectory['trajectory']['y'] = y
            self.predicted_trajectory['trajectory']['z'] = x
            self.predicted_trajectory['trajectory']['qx'] = qx
            self.predicted_trajectory['trajectory']['qy'] = qy
            self.predicted_trajectory['trajectory']['qz'] = qz
            self.predicted_trajectory['trajectory']['qw'] = qw
            self.predicted_trajectory['trajectory']['t'] = t
            self.predicted_trajectory['context'] = [context.x, context.y, context.z]

            if before_after == 1:
                
                self.conditions[condition]['predicted_trajectory']['before'] = self.predicted_trajectory
   
                print("Set prediction before updating")

            else:
                self.conditions[condition]['predicted_trajectory']['after'] = self.predicted_trajectory
                self.conditions[condition]['predicted_trajectory']['number_of_updates'] = num_updates
                
                print("Set prediction after updating")

    
    def getPredictedTrajectory(self, condition):
        return self.conditions[condition]['predicted_trajectory']['trajectory']

    def incrementObstaclesHit(self, condition):
        self.conditions[condition]['obstacles_hit'] += 1

    def incrementObjectMissed(self, condition):
        self.conditions[condition]['object_missed'] += 1
    
    def incrementNumberOfRefinement(self, condition):
        self.conditions[condition]['number_of_refinements'] += 1

    def incrementNumberOfUpdates(self, condition):
        self.conditions[condition]['number_of_updates'] += 1

    def setNumberOfUpdates(self, condition, value):
        self.conditions[condition]['number_of_updates'] = value
    
    def toCSV(self):
        data = {'number': self.number, 'age': self.age, 'sex': self.gender, 'condition': self.conditions}
        df = pd.DataFrame.from_dict(data)
        df.to_csv(self.path + 'data.csv')

            
if __name__ == "__main__":
    participant1_data = ParticipantData(1, 'm', 26)
    participant1_data.incrementObjectMissed(condition=1)
    participant1_data.incrementObstaclesHit(condition=1)
    participant1_data.setPredictedTrajectory(condition=1, from_file=True)
    participant1_data.toCSV()
