#!/usr/bin/env python3.5

import os, csv, ast, copy
import pandas as pd

class ParticipantData(object):
    def __init__(self, number, sex, age, path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/'):

        self.number = number
        self.num_methods = 4
        self.methods = {}

        # check if path exists, create if not
        self.path = path + 'participant_' + str(number) + '/'
        if not os.path.exists(self.path):
            os.makedirs(self.path)
            self.sex = sex
            self.age = age

            self.initData()

        # if it exists --> load data 
        elif os.path.exists(self.path) and os.path.isfile(self.path+'data.txt'):
            self.loadData()
        else:
            self.sex = sex
            self.age = age
            self.initData()

        # parameters used to fill the methods dictionary
        self.predicted_trajectory = {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [] }
        self.refined_trajectory = {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [] }
        self.variation = 1
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
        self.variations = {}
        self.trials = {}

        self.num_methods = 4
        self.num_variations = 2
        self.num_object_positions = 6
        self.num_trials = 5

        for i in range(self.num_trials):
            self.trials[i+1] = {
                'predicted_trajectory': {}, 'context': [],
                'refined_trajectory': {},
                'number_of_refinements': 0,
                'object_missed': False,
                'obstacle_hit': False,
                'success': True,
                'time': 0
            }

        for j in range(self.num_object_positions):
            self.object_positions[j+1] = {
            'trial': copy.deepcopy(self.trials),
            'adaptation_time': 0
            }
        
        for i in range(self.num_variations):
            self.variations[i+1] = {
                'object_position': copy.deepcopy(self.object_positions),
                'total_adaptation_time': 0
            }
        
        for i in range(self.num_methods):
            self.methods[i+1] = {
                'variation': copy.deepcopy(self.variations),
            }

    def loadData(self):
        data_path = self.path + 'data.txt'
        print(data_path)
        
        with open(data_path, "r") as infile:
            outfile = ast.literal_eval(infile.read())

            self.sex = outfile['sex']
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

    def setVariation(self, variation):
        self.variation = variation

    def setObjectPosition(self, object_position):
        self.object_position = object_position

    def setMethod(self, method):
        self.method = method


    def setRefinedTrajectory(self, *args, **kwargs):
        if "from_file" in kwargs and kwargs["from_file"] == 1 and "context" in kwargs and "time" in kwargs:
            
            context = kwargs["context"]
            time = kwargs["time"]
            
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

            self.refined_trajectory['x'] = x
            self.refined_trajectory['y'] = y
            self.refined_trajectory['z'] = z
            self.refined_trajectory['qx'] = qx
            self.refined_trajectory['qy'] = qy
            self.refined_trajectory['qz'] = qz
            self.refined_trajectory['qw'] = qw
            self.refined_trajectory['t'] = t

            # append dictionary
            # we start with 0 refinement --> n + 1
            #             
            # increment number of refined trajectories
            # self.methods[self.method]['variation'][self.variation]['object_position'][self.object_position]['trial'][self.trial]['number_of_refinements'] += 1
            trajectory = copy.deepcopy(self.refined_trajectory)

            self.methods[self.method]['variation'][self.variation]['object_position'][self.object_position]['trial'][self.trial]['refined_trajectory'] = trajectory
            
            # increment time (need to think about how to get the correct time)
            self.methods[self.method]['variation'][self.variation]['object_position'][self.object_position]['trial'][self.trial]['time'] = time

            return 0
                
    def setPredictedTrajectory(self, *args, **kwargs):

        if "from_file" in kwargs and kwargs["from_file"] == 1 and "context" in kwargs and "time" in kwargs:
            
            context = kwargs["context"]
            time = kwargs["time"]
            print("elapsed time to store python = " + str(time))

            path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/gui/data/experiment/'
            file_name = 'predicted_trajectory.csv'

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

            self.predicted_trajectory['x'] = x
            self.predicted_trajectory['y'] = y
            self.predicted_trajectory['z'] = x
            self.predicted_trajectory['qx'] = qx
            self.predicted_trajectory['qy'] = qy
            self.predicted_trajectory['qz'] = qz
            self.predicted_trajectory['qw'] = qw
            self.predicted_trajectory['t'] = t

            # self.predicted_trajectory['context'] = [context.x, context.y, context.z]

            # if before_after == 1:
            #     print('method = ' + str(method))
            #     print('object_position = ' + str(object_position))


            #     self.methods[method][object_position]['predicted_trajectory']['before'] = self.predicted_trajectory
            #     with open('/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/test1.txt', 'w+') as f:
            #         f.write(str(self.methods))

            #     print("Set prediction before updating")

            # else:
            #     self.methods[method][object_position]['predicted_trajectory']['after'] = self.predicted_trajectory
            #     self.methods[method][object_position]['predicted_trajectory']['number_of_updates'] = num_updates
                
            #     print("Set prediction after updating")

            # self.methods[self.method][self.variation][self.object_position]['predicted'][self.trial] = self.predicted_trajectory

            trajectory = copy.deepcopy(self.predicted_trajectory)

            self.methods[self.method]['variation'][self.variation]['object_position'][self.object_position]['trial'][self.trial]['predicted_trajectory'] = trajectory
            self.methods[self.method]['variation'][self.variation]['object_position'][self.object_position]['trial'][self.trial]['context'] = context
            self.methods[self.method]['variation'][self.variation]['object_position'][self.object_position]['trial'][self.trial]['time'] = time


    def setObstaclesHit(self):
        self.methods[self.method]['variation'][self.variation]['object_position'][self.object_position]['trial'][self.trial]['obstacle_hit'] = True
        self.methods[self.method]['variation'][self.variation]['object_position'][self.object_position]['trial'][self.trial]['success'] = False

        # self.methods[method]['obstacles_hit'] += 1

    def setObjectMissed(self):
        self.methods[self.method]['variation'][self.variation]['object_position'][self.object_position]['trial'][self.trial]['object_missed'] = True
        self.methods[self.method]['variation'][self.variation]['object_position'][self.object_position]['trial'][self.trial]['success'] = False

        # self.methods[method]['object_missed'] += 1
    
    def incrementNumberOfRefinements(self):
        self.methods[self.method]['variation'][self.variation]['object_position'][self.object_position]['trial'][self.trial]['number_of_refinements'] += 1
        print("method " + str(self.method))
        print("variation " + str(self.variation))
        print("object position " + str(self.object_position))
        print("trial " + str(self.trial))
        print("number of refinements incremented to " + str(self.methods[self.method]['variation'][self.variation]['object_position'][self.object_position]['trial'][self.trial]['number_of_refinements']))

    def incrementNumberOfUpdates(self, method):
        self.methods[method]['number_of_updates'] += 1

    def setNumberOfUpdates(self, method, value):
        self.methods[method]['number_of_updates'] = value
    
    def toCSV(self):
        # with open('/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/test2.txt', 'w+') as f:
        #     f.write(str(self.methods))
        print("number of refinements incremented to " + str(self.methods[self.method]['variation'][self.variation]['object_position'][self.object_position]['trial'][self.trial]['number_of_refinements']))

        data = {'number': self.number, 'age': self.age, 'sex': self.sex, 'method': self.methods}
        with open(self.path + 'data.txt', 'w+') as f:
            f.write(str(data))
            
        # df = pd.DataFrame.from_dict(data)
        # df.to_csv(self.path + 'data.csv')

            
if __name__ == "__main__":
    participant1_data = ParticipantData(1, 'm', 26)
    participant1_data.incrementObjectMissed(method=1)
    participant1_data.incrementObstaclesHit(method=1)
    participant1_data.setPredictedTrajectory(method=1, from_file=True)
    participant1_data.toCSV()
