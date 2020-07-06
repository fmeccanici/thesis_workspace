import ast
from data_logger_python.data_logger_python import ParticipantData
import matplotlib.pyplot as plt
import numpy as np

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

    def calculateAdaptationTime(self, participant_number, method, variation):
        methods = self.data[participant_number].getMethods()
        
        total_adaptation_time = 0
        adaptation_time = []

        for object_position in methods[method]['variation'][variation]['object_position']:
            time = 0
            for trial in methods[method]['variation'][variation]['object_position'][object_position]['trial']:
                
                time += self.getTime(participant_number, method, variation, object_position, trial)

            adaptation_time.append(time)

        total_adaptation_time = sum(adaptation_time)
        
        return total_adaptation_time, adaptation_time

    def getNumberOfRefinements(self, participant_number, method, variation, object_position):
        
        for trial in methods[method]['variation'][variation]['object_position'][object_position]['trial']:
            
        return methods[method]['variation'][variation]['object_position'][object_position]['trial'][trial]['number_of_refinements']

    def plotAdaptationTime(self, participant_number):
        
        # set to test, need to loop and calculate mean in final version
        variation = 1

        methods = ['online + omni', 'offline + omni', 'online + keyboard', 'offline + keyboard']
        object_positions = ['1', '2', '3', '4', '5', '6']

        for method in range(1, self.num_methods+1):
            plt.figure()
            adaptation_times = self.calculateAdaptationTime(participant_number, method, variation)[1]
            plt.bar(object_positions, adaptation_times)
            plt.title(methods[method-1])
            plt.xlabel("Object position [-]")
            plt.ylabel("Adaptation time [s]")
        plt.show()
    
    def plotNumberOfRefinements(self, participant_number):
        # set to test, need to loop and calculate mean in final version
        variation = 1

        methods = ['online + omni', 'offline + omni', 'online + keyboard', 'offline + keyboard']
        object_positions = ['1', '2', '3', '4', '5', '6']
        
        for method in range(1, self.num_methods+1):
            plt.figure()
            number_of_refinements = getNumberOfRefinements
            plt.bar(object_positions, adaptation_times)
            plt.title(methods[method-1])
            plt.xlabel("Object position [-]")
            plt.ylabel("Adaptation time [s]")
        plt.show()


if __name__ == "__main__":
    data_analysis = DataAnalysis()
    data_analysis.loadData(1)
    # data_analysis.plotPrediction(1, 4, 1, 1, 1)
    # data_analysis.plotRefinement(1, 4, 1, 1, 1)
    # print(data_analysis.getTime(1, 3, 1, 1, 1))
    data_analysis.calculateAdaptationTime(1, 3, 1)
    data_analysis.plotAdaptationTime(1)
