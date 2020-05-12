import rospy, os, csv
import pandas as pd

class ParticipantData(object):
    def __init__(self, number, gender, age, path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logging/data/'):

        self.number = number
        self.gender = gender
        self.age = age
        self.conditions = {}
        
        for i in range(3):
            self.conditions[i+1] = { 
                            'predicted_trajectory': {'trajectory': {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [] }, 'context': []}, 
                            'refined_trajectories': {'trajectories': {}, 'contexts': {}, 'forces': {}}, 'number_of_refinement': 0,
                            'objects_missed': 0, 'obstacles_hit': 0}
        
        self.refined_trajectories = {}
        self.refined_trajectory = {'trajectory': {'x': [], 'y': [], 'z': [], 'qx': [],'qy': [], 'qz': [], 'qw': [], 't': [] }, 'context': []}

        # check if path exists, create if not
        self.path = path + 'participant_' + str(number) + '/'
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        
            # for i in range(3):
            #     os.makedirs(self.path + 'condition_' + str(i+1) + '/')

        print(self.conditions[1])

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
        self.path = path

    def getStoragePath(self):
        return self.path

    def addRefinedTrajectory(self, *args, **kwargs):
        if "trajectory" in kwargs and "context" in kwargs and "condition" in kwargs:

            # add refined trajectory 
            self.refined_trajectories[condition]['trajectory'].append(trajectory)
            self.refined_trajectories[condition]['context'].append(context)
            
            # increment number of refined trajectories
            self.number_of_refinements[condition] += 1
            
            return 0
            
        elif "from_file" in kwargs and "condition" in kwargs:
            condition = kwargs["condition"]

            path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/gui/data/experiment/'
            file_name = 'refined_trajectory.csv'
            with open(path+file_name, 'r') as f:
                reader = csv.reader(f)
                data = list(reader)
            
            # append dictionary
            self.refined_trajectories[condition]['trajectories'][self.number_of_refinements[condition]] = data
            
            # increment number of refined trajectories
            self.number_of_refinements[condition] += 1

            return 0
    
    def getRefinedTrajectories(self, condition):
        return self.refined_trajectories[condition]['trajectories']
                
    def setPredictedTrajectory(self, *args, **kwargs):
        if "trajectory" in kwargs and "context" in kwargs and "condition" in kwargs:
            condition = kwargs["condition"]

            # add refined trajectory 
            self.predicted_trajectory[condition]['trajectory'].append(trajectory)
            self.predicted_trajectory[condition]['context'].append(context)
            
            return 0
        
        elif "from_file" in kwargs and kwargs["condition"] == 1 and "condition" in kwargs:
            path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/gui/data/experiment/'
            file_name = 'predicted_trajectory.csv'
            
            condition = kwargs["condition"]

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
            
            self.conditions[condition]['predicted_trajectory']['trajectory']['x'] = x
            self.conditions[condition]['predicted_trajectory']['trajectory']['y'] = y
            self.conditions[condition]['predicted_trajectory']['trajectory']['z'] = x
            self.conditions[condition]['predicted_trajectory']['trajectory']['qx'] = qx
            self.conditions[condition]['predicted_trajectory']['trajectory']['qy'] = qy
            self.conditions[condition]['predicted_trajectory']['trajectory']['qz'] = qz
            self.conditions[condition]['predicted_trajectory']['trajectory']['qw'] = qw
            self.conditions[condition]['predicted_trajectory']['trajectory']['t'] = t
                
    
    def getPredictedTrajectory(self, condition):
        return self.conditions[condition]['predicted_trajectory']['trajectory']

    def incrementObstaclesHit(self, condition):
        self.obstacles_hit[condition] += 1

    def incrementObjectMissed(self, condition):
        self.object_missed[condition] += 1

    def toCSV(self):
        df = pd.DataFrame.from_dict(self.conditions)
        df.to_csv(self.path + 'data.csv')
        # print(df.head())
        # file_names = ['condition_1.csv','condition_2.csv', 'condition_3.csv']
        # column_names = ['ParticipantNumber', 'Gender', 'Age', 'RefinedTrajectories', 'PredictedTrajectory', 'NumberOfRefinements', 'Context',
        #                 'ObstaclesHit', 'ObjectMissed', 'Forces']
        # data = 'x, y, z, qx, qy, qz, qw, t \n'
        # for i,file_name in enumerate(file_names):
        #     data = ""
        #     for column in column_names:
        #         data += "%s, " % (column)
        #     data += '\n'
        #     try:
        #         with open(self.path+file_name, 'w+') as csv_file:
        #             data += "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s \n" % (self.number, self.gender, self.age, 
        #                                                     self.refined_trajectories[i+1]['trajectories'], 
        #                                                     self.predicted_trajectory[i+1]['trajectory'], 
        #                                                     self.number_of_refinements[i+1], self.predicted_trajectory[i+1]['context'],
        #                                                     self.obstacles_hit[i+1], self.object_missed[i+1], self.forces[i+1])
        #             csv_file.write(data)

        #     except IOError:
        #         print("I/O error")
            
if __name__ == "__main__":
    participant1_data = ParticipantData(1, 'm', 26)
    # participant1_data.incrementObjectMissed(condition=1)
    participant1_data.setPredictedTrajectory(condition=1, from_file=True)
    participant1_data.toCSV()
    # print(participant1_data.getPredictedTrajectory(condition=1))