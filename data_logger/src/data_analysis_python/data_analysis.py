import ast
from data_logger_python.data_logger_python import ParticipantData
import matplotlib.pyplot as plt

class DataAnalysis(object):
    def __init__(self):
        self.data_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/'
        self.data = {}
        self.num_methods = 4

    def loadData(self, participant_number):
        participant = ParticipantData(participant_number, 0, 0)
        self.data[participant_number] = participant        


    def plotPrediction(self, participant_number, method, variation, object_position, trial):
        methods = self.data[participant_number].getMethods()
        prediction = methods[method]['variation'][variation]['object_position'][object_position]['trial'][trial]['predicted_trajectory']

        plt.figure('prediction')
        plt.title('Prediction: method ' + str(method) + ', variation ' + str(variation) + ', object position ' + str(object_position) + ', trial ' + str(trial))
        for key in prediction:
            if key is not 't':
                plt.plot(prediction[key], label=key)
        plt.legend()
        plt.grid()

    def plotRefinement(self, participant_number, method, variation, object_position, trial):
        methods = self.data[participant_number].getMethods()
        refinement = methods[method]['variation'][variation]['object_position'][object_position]['trial'][trial]['refined_trajectory']
        
        plt.figure('refinement')
        plt.title('Refinement: method ' + str(method) + ', variation ' + str(variation) + ', object position ' + str(object_position) + ', trial ' + str(trial))
        for key in refinement:
            if key is not 't':
                plt.plot(refinement[key], label=key)
        plt.legend()
        plt.grid()

    def getTime(self, participant_number, method, variation, object_position, trial):
        methods = self.data[participant_number].getMethods()
        time = methods[method]['variation'][variation]['object_position'][object_position]['trial'][trial]['time']

        return time

    # def calculateTotalAdaptationTime(self, participant_number, method, variation):
    #     methods = self.data[participant_number].getMethods()
        
    #     total_adaptation_time = 0

    #     for x in methods[method]['variation'][variation]['object_position']:
    #         for y in 
    #     time = methods[method]['variation'][variation]['object_position'][object_position]['trial'][trial]['time']

if __name__ == "__main__":
    data_analysis = DataAnalysis()
    data_analysis.loadData(1)
    data_analysis.plotPrediction(1, 4, 1, 1, 1)
    data_analysis.plotRefinement(1, 4, 1, 1, 1)
    print(data_analysis.getTime(1, 3, 1, 1, 1))
    
