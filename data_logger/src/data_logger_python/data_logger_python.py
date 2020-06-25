#!/usr/bin/env python3.5

import os, csv
import pandas as pd

class ParticipantData(object):
    def __init__(self, number, sex, age, path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/'):

        self.number = number
        self.sex = sex
        self.age = age
        
        self.object_positions = {}
        self.methods = {}
        self.variations = {}

        self.num_methods = 4
        self.num_variations = 2
        self.num_object_positions = 6
        
        for j in range(self.num_object_positions):
            self.object_positions[j+1] = {
            'predicted_trajectory': {}, 'context': [], 
            'refined_trajectory': {},
            'number_of_refinements' : 0, 
            'object_missed' : 0, 'obstacles_hit' : 0,
            'adaptation_time': 0
            }
        self.predicted_trajectory = {'trajectory': {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [] }}
        self.refined_trajectory = {'trajectory': {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [] }}

        for i in range(self.num_variations):
            self.variations[i+1] = self.object_positions

        # 1 = Online + Teleop
        # 2 = Ofline + Teleop
        # 3 = Online + Pendant
        # 4 = Offline + Pendant
        for i in range(self.num_methods):
            self.methods[i+1] = self.variations
        
        # check if path exists, create if not
        self.path = path + 'participant_' + str(number) + '/'
        if not os.path.exists(self.path):
            os.makedirs(self.path)

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

    def setRefinedTrajectory(self, *args, **kwargs):
        if "from_file" in kwargs and kwargs["from_file"] == 1 and "context" in kwargs and "method" in kwargs and "environment" in kwargs:
            method = kwargs["method"]
            context = kwargs["context"]
            environment = kwargs["environment"]

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
            n = self.methods[method][environment]['number_of_refinements'] + 1
            print(n)
            self.methods[method][environment]['refined_trajectory'] = self.refined_trajectory          
            
            # increment number of refined trajectories
            self.methods[method][environment]['number_of_refinements'] += 1

            return 0

    def getRefinedTrajectories(self, method):
        return self.refined_trajectories[method]['trajectories']
                
    def setPredictedTrajectory(self, *args, **kwargs):
        if "from_file" in kwargs and kwargs["from_file"] == 1 and "method" in kwargs and "context" in kwargs and "before_after" in kwargs and "num_updates" in kwargs and "environment" in kwargs:
            path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/gui/data/experiment/'
            file_name = 'predicted_trajectory.csv'
            
            method = kwargs["method"]
            context = kwargs["context"]
            before_after = kwargs["before_after"]
            num_updates = kwargs["num_updates"]
            environment = kwargs["environment"]

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
                print('method = ' + str(method))
                print('environment = ' + str(environment))


                self.methods[method][environment]['predicted_trajectory']['before'] = self.predicted_trajectory
                with open('/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/test1.txt', 'w+') as f:
                    f.write(str(self.methods))

                print("Set prediction before updating")

            else:
                self.methods[method][environment]['predicted_trajectory']['after'] = self.predicted_trajectory
                self.methods[method][environment]['predicted_trajectory']['number_of_updates'] = num_updates
                
                print("Set prediction after updating")

    
    def getPredictedTrajectory(self, method):
        return self.methods[method]['predicted_trajectory']['trajectory']

    def incrementObstaclesHit(self, method):
        self.methods[method]['obstacles_hit'] += 1

    def incrementObjectMissed(self, method):
        self.methods[method]['object_missed'] += 1
    
    def incrementNumberOfRefinement(self, method):
        self.methods[method]['number_of_refinements'] += 1

    def incrementNumberOfUpdates(self, method):
        self.methods[method]['number_of_updates'] += 1

    def setNumberOfUpdates(self, method, value):
        self.methods[method]['number_of_updates'] = value
    
    def toCSV(self):
        with open('/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/test2.txt', 'w+') as f:
            f.write(str(self.methods))

        data = {'number': self.number, 'age': self.age, 'sex': self.gender, 'method': self.methods}

        df = pd.DataFrame.from_dict(data)
        df.to_csv(self.path + 'data.csv')

            
if __name__ == "__main__":
    participant1_data = ParticipantData(1, 'm', 26)
    participant1_data.incrementObjectMissed(method=1)
    participant1_data.incrementObstaclesHit(method=1)
    participant1_data.setPredictedTrajectory(method=1, from_file=True)
    participant1_data.toCSV()
