import ast, os
from data_logger_python.data_logger_python import ParticipantData
import matplotlib.pyplot as plt
import numpy as np

class DataAnalysis(object):
    def __init__(self):
        self.data_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/'
        self.figures_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/figures/'
        self.data = {}
        self.num_methods = 4
        self.num_object_positions = 6
        self.num_trials = 5
        self.methods_labels = ['online + omni', 'offline + omni', 'online + keyboard', 'offline + keyboard']
        self.object_positions_labels = ['1', '2', '3', '4', '5', '6']

    def loadData(self, participant_number):
        participant = ParticipantData(participant_number, 0, 0)
        self.data[participant_number] = participant        

        # check if path exists, create if not
        path = self.figures_path + 'participant_' + str(participant_number) + '/'
        if not os.path.exists(path):
            os.makedirs(path)

    def plotPrediction(self, participant_number, method, object_position, trial):
        methods = self.data[participant_number].getMethods()
        prediction = methods[method]['object_position'][object_position]['trial'][trial]['predicted_trajectory']

        plt.figure('prediction')
        plt.title('Prediction: method ' + str(method) + ', object position ' + str(object_position) + ', trial ' + str(trial))
        for key in prediction:
            if key is not 't':
                plt.plot(prediction[key], label=key)
        plt.legend()
        plt.grid()

    def plotRefinement(self, participant_number, method, object_position, trial):
        methods = self.data[participant_number].getMethods()
        refinement = methods[method]['object_position'][object_position]['trial'][trial]['refined_trajectory']
        
        plt.figure('refinement')
        plt.title('Refinement: method ' + str(method) + ', object position ' + str(object_position) + ', trial ' + str(trial))
        for key in refinement:
            if key is not 't':
                plt.plot(refinement[key], label=key)
        plt.legend()
        plt.grid()

    def getTime(self, participant_number, method, object_position, trial):
        methods = self.data[participant_number].getMethods()
        time = methods[method]['object_position'][object_position]['trial'][trial]['time']

        return time

    def calculateAdaptationTime(self, participant_number, method):
        methods = self.data[participant_number].getMethods()
        
        total_adaptation_time = 0
        adaptation_time = []

        for object_position in methods[method]['object_position']:
            time = 0
            for trial in methods[method]['object_position'][object_position]['trial']:
                
                time += self.getTime(participant_number, method, object_position, trial)

            adaptation_time.append(time)

        total_adaptation_time = sum(adaptation_time)
        
        return total_adaptation_time, adaptation_time

    def getNumberOfRefinements(self, participant_number, method):
        methods = self.data[participant_number].getMethods()
        refinements_per_object_position = []

        for object_position in methods[method]['object_position']:
            refinements = 0

            for trial in methods[method]['object_position'][object_position]['trial']:
                refinements += methods[method]['object_position'][object_position]['trial'][trial]['number_of_refinements']

            refinements_per_object_position.append(refinements)

        return refinements_per_object_position

    def getNumberOfObjectMissed(self, participant_number, method, variation):
        methods = self.data[participant_number].getMethods()

        object_missed_per_object_position = []

        for object_position in methods[method]['object_position']:
            object_missed = 0

            for trial in methods[method]['object_position'][object_position]['trial']:
                object_missed += int(methods[method]['object_position'][object_position]['trial'][trial]['success'])

            object_missed_per_object_position.append(object_missed)

        return object_missed_per_object_position

    def getNumberOfObstaclesHit(self, participant_number, method):
        methods = self.data[participant_number].getMethods()

        obstacle_hit_per_object_position = []

        for object_position in methods[method]['object_position']:
            obstacle_hit = 0

            for trial in methods[method]['object_position'][object_position]['trial']:
                obstacle_hit += int(methods[method]['object_position'][object_position]['trial'][trial]['success'])

            obstacle_hit_per_object_position.append(obstacle_hit)

        return obstacle_hit_per_object_position


    def getSuccessfulPredictions(self, participant_number, method):
        methods = self.data[participant_number].getMethods()
        
        success_per_object_position = []
        
        for object_position in methods[method]['object_position']:
            success = 0


            for trial in methods[method]['object_position'][object_position]['trial']:
                try:
                    success += int(methods[method]['object_position'][object_position]['trial'][trial]['predicted_trajectory']['success'])
                except KeyError as e:
                    print("Key not available: " + str(e))
                    continue
            success_per_object_position.append(success)

        return success_per_object_position

    def plotAdaptationTime(self, participant_number):
        plt.figure()

        for method in range(1, self.num_methods+1):
            plt.subplot(2, 2, method)
            adaptation_times = self.calculateAdaptationTime(participant_number, method)[1]
            plt.bar(self.object_positions_labels, adaptation_times)
            plt.title(self.methods_labels[method-1])
            plt.xlabel("Object position [-]")
            plt.ylabel("Adaptation time [s]")
            plt.ylim((0,150))
            plt.tight_layout()

        plt.savefig(self.figures_path + 'participant_' + str(participant_number) + '/adaptation_time.pdf')
    
    def plotNumberOfRefinements(self, participant_number):
        plt.figure()
        
        for method in range(1, self.num_methods+1):
            plt.subplot(2, 2, method)

            number_of_refinements = self.getNumberOfRefinements(participant_number, method)
            plt.bar(self.object_positions_labels, number_of_refinements)
            plt.title(self.methods_labels[method-1])
            plt.xlabel("Object position [-]")
            plt.ylabel("Number of refinements [-]")
            plt.ylim((0,10))

            plt.tight_layout()

        plt.savefig(self.figures_path + 'participant_' + str(participant_number) + '/number_of_refinements.pdf')

    def plotNumberOfObjectMissed(self, participant_number):
        plt.figure()
        for method in range(1, self.num_methods+1):
            plt.subplot(2, 2, method)
            number_of_object_missed = self.getNumberOfObjectMissed(participant_number, method)
            plt.bar(self.object_positions_labels, [ x / self.num_trials * 100 for x in number_of_object_missed ] )
            plt.title(self.methods_labels[method-1])
            plt.xlabel("Object position [-]")
            plt.ylabel("Object missed [%]")
            plt.tight_layout()
        
        plt.savefig(self.figures_path + 'participant_' + str(participant_number) + '/number_of_objects_missed.pdf')

        plt.figure()

    def plotNumberOfObstaclesHit(self, participant_number):

        plt.figure()
        for method in range(1, self.num_methods+1):
            plt.subplot(2, 2, method)
            number_of_obstacles_hit = self.getNumberOfObstaclesHit(participant_number, method, variation)
            plt.bar(self.object_positions_labels, [ x / self.num_trials * 100 for x in number_of_obstacles_hit ] )
            plt.title(self.methods_labels[method-1])
            plt.xlabel("Object position [-]")
            plt.ylabel("Obstacles hit [%]")
            plt.tight_layout()
        
        plt.savefig(self.figures_path + 'participant_' + str(participant_number) + '/number_of_obstacles_hit.pdf')

        plt.figure()

    def plotSuccesfullPredictions(self, participant_number):
        plt.figure()
        for method in range(1, self.num_methods+1):
            plt.subplot(2, 2, method)
            number_of_success = self.getSuccessfulPredictions(participant_number, method)
            plt.bar(self.object_positions_labels, [ x / self.num_trials * 100 for x in number_of_success ] )
            plt.title(self.methods_labels[method-1])
            plt.xlabel("Object position [-]")
            plt.ylabel("Successfull trials [%]")
            plt.tight_layout()
        
        plt.savefig(self.figures_path + 'participant_' + str(participant_number) + '/number_of_successes.pdf')

        plt.figure()

    def plotDataBeforeExperiment(self):
        path = self.data_path + 'before_experiment/data.txt'
        num_object_positions = 4
        object_labels = ["1", "2", "3", "4"]
        
        with open(path, "r") as infile:
            outfile = ast.literal_eval(infile.read())
            data_before_experiment = outfile

            success_list = []
            object_missed_list = []
            obstacle_hit_list = []

            for position in range(1,num_object_positions+1):
                success = 0
                object_missed = 0
                obstacle_hit = 0
                for trial in range(1,self.num_trials+1):
                    success += int(data_before_experiment[position]['trial'][trial]['predicted_trajectory']['success'])
                    object_missed += int(data_before_experiment[position]['trial'][trial]['predicted_trajectory']['object_missed'])
                    obstacle_hit += int(data_before_experiment[position]['trial'][trial]['predicted_trajectory']['obstacle_hit'])

                success_list.append(success)
                object_missed_list.append(object_missed)
                obstacle_hit_list.append(obstacle_hit)
            
            fig = plt.figure()
            fig.suptitle("Initial model")

            plt.subplot(1,3,1)
            plt.bar(object_labels, success)
            plt.xlabel("Object position [-]")
            plt.ylabel("Success [True/False]")
            plt.ylim([0,1])

            plt.subplot(1,3,2)
            plt.bar(object_labels, object_missed)
            plt.xlabel("Object position [-]")
            plt.ylabel("Object missed [True/False]")
            plt.ylim([0,1])


            plt.subplot(1,3,3)
            plt.bar(object_labels, obstacle_hit)
            plt.xlabel("Object position [-]")
            plt.ylabel("Obstacle hit [True/False]")
            plt.ylim([0,1])

            plt.tight_layout()
            fig.subplots_adjust(top=0.88)

        plt.savefig(self.figures_path + "/before_experiment/success.pdf")

if __name__ == "__main__":
    data_analysis = DataAnalysis()
    data_analysis.loadData(90)
    # data_analysis.plotPrediction(1, 4, 1, 1, 1)
    # data_analysis.plotRefinement(1, 4, 1, 1, 1)
    # print(data_analysis.getTime(1, 3, 1, 1, 1))
    # data_analysis.calculateAdaptationTime(1, 3, 1)
    data_analysis.plotAdaptationTime(90)
    data_analysis.plotNumberOfRefinements(90)
    data_analysis.plotSuccesfullPredictions(90)
    data_analysis.plotNumberOfObstaclesHit(90)
    data_analysis.plotNumberOfObjectMissed(90)
    # data_analysis.plotDataBeforeExperiment()