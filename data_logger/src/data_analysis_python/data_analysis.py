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

    def calculateRefinementTime(self, participant_number, method):
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

    def getNumberOfObjectMissed(self,refinement_or_prediction, participant_number, method):
        methods = self.data[participant_number].getMethods()
        
        if refinement_or_prediction == 'refinement':
            trajectory = 'refined_trajectory'
        elif refinement_or_prediction == 'prediction':
            trajectory = 'predicted_trajectory'

        object_missed_per_object_position = []

        for object_position in methods[method]['object_position']:
            object_missed = 0

            for trial in methods[method]['object_position'][object_position]['trial']:
                try:
                    object_missed += int(methods[method]['object_position'][object_position]['trial'][trial][trajectory]['object_missed'])
                except KeyError as e:
                    # print("Key not available: " + str(e))
                    continue

            object_missed_per_object_position.append(object_missed)

        return object_missed_per_object_position

    def getNumberOfObjectKickedOver(self, refinement_or_prediction, participant_number, method):
        methods = self.data[participant_number].getMethods()
        
        if refinement_or_prediction == 'refinement':
            trajectory = 'refined_trajectory'
        elif refinement_or_prediction == 'prediction':
            trajectory = 'predicted_trajectory'

        object_kicked_over_per_object_position = []

        for object_position in methods[method]['object_position']:
            object_kicked_over = 0

            for trial in methods[method]['object_position'][object_position]['trial']:
                try:
                    object_kicked_over += int(methods[method]['object_position'][object_position]['trial'][trial][trajectory]['object_kicked_over'])
                except KeyError as e:
                    # print("Key not available: " + str(e))
                    continue

            object_kicked_over_per_object_position.append(object_kicked_over)

        return object_kicked_over_per_object_position

    def getNumberOfObstaclesHit(self, refinement_or_prediction, participant_number, method):
        methods = self.data[participant_number].getMethods()
        
        if refinement_or_prediction == 'refinement':
            trajectory = 'refined_trajectory'
        elif refinement_or_prediction == 'prediction':
            trajectory = 'predicted_trajectory'

        obstacle_hit_per_object_position = []

        for object_position in methods[method]['object_position']:
            obstacle_hit = 0

            for trial in methods[method]['object_position'][object_position]['trial']:
                
                try:
                    obstacle_hit += int(methods[method]['object_position'][object_position]['trial'][trial][trajectory]['obstacle_hit'])
                except KeyError as e:
                    # print("Key not available: " + str(e))
                    continue
            obstacle_hit_per_object_position.append(obstacle_hit)

        return obstacle_hit_per_object_position

    def getSuccessfulTrials(self, refinement_or_prediction, participant_number, method):
        methods = self.data[participant_number].getMethods()
        if refinement_or_prediction == 'refinement':
            trajectory = 'refined_trajectory'
        elif refinement_or_prediction == 'prediction':
            trajectory = 'predicted_trajectory'

        success_per_object_position = []
        
        for object_position in methods[method]['object_position']:
            success = 0

            for trial in methods[method]['object_position'][object_position]['trial']:

                try:
                    # if number of refinement is zero, we had a successful prediction and no refinement was necessary
                    # then the success has to be 1 but is zero by default since we have no refinement 
                    # it gives a key error thus reverts back to the default which is 0
                    # I only did experiment yet with method 3 on myself but this has to be done for all the methods
                    if method == 3 and trajectory == 'refined_trajectory' and methods[method]['object_position'][object_position]['trial'][trial]['number_of_refinements'] == 0:
                        success += 1
                    else:
                        success += int(methods[method]['object_position'][object_position]['trial'][trial][trajectory]['success'])

                except KeyError as e:
                    # print("Key not available: " + str(e))
                    continue
            success_per_object_position.append(success)

        return success_per_object_position

    def plotRefinementTime(self, participant_number):
        plt.figure()

        for method in range(1, self.num_methods+1):
            plt.subplot(2, 2, method)
            refinement_times = self.calculateRefinementTime(participant_number, method)[1]
            print('adaptation times for plotting: ' + str(refinement_times))
            plt.bar(self.object_positions_labels, refinement_times)
            plt.title(self.methods_labels[method-1])
            plt.xlabel("Object position [-]")
            plt.ylabel("Refinement time [s]")
            plt.ylim((0,250))
            plt.tight_layout()

        plt.savefig(self.figures_path + 'participant_' + str(participant_number) + '/refinement_time.pdf')
    
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

    def plotNumberOfObjectKickedOver(self, participant_number, refinement_or_prediction):

        plt.figure()
        for method in range(1, self.num_methods+1):
            plt.subplot(2, 2, method)
            number_of_object_missed = self.getNumberOfObjectKickedOver(participant_number=participant_number, refinement_or_prediction=refinement_or_prediction, method=method)
            plt.bar(self.object_positions_labels, [ x / self.num_trials * 100 for x in number_of_object_missed ] )
            plt.title(self.methods_labels[method-1])
            plt.xlabel("Object position [-]")
            plt.ylabel("Object kicked over [%]")
            plt.tight_layout()
        
        plt.savefig(self.figures_path + 'participant_' + str(participant_number) + '/number_of_objects_kicked_over_' + str(refinement_or_prediction) + '.pdf')

        plt.figure()

    def plotNumberOfObjectMissed(self, participant_number, refinement_or_prediction):

        plt.figure()
        for method in range(1, self.num_methods+1):
            plt.subplot(2, 2, method)
            number_of_object_missed = self.getNumberOfObjectMissed(participant_number=participant_number, refinement_or_prediction=refinement_or_prediction, method=method)
            plt.bar(self.object_positions_labels, [ x / self.num_trials * 100 for x in number_of_object_missed ] )
            plt.title(self.methods_labels[method-1])
            plt.xlabel("Object position [-]")
            plt.ylabel("Object missed [%]")
            plt.tight_layout()
        
        plt.savefig(self.figures_path + 'participant_' + str(participant_number) + '/number_of_objects_missed_' + str(refinement_or_prediction) + '.pdf')

        plt.figure()

    def plotNumberOfObstaclesHit(self, participant_number, refinement_or_prediction):

        plt.figure()
        for method in range(1, self.num_methods+1):
            plt.subplot(2, 2, method)
            number_of_obstacles_hit = self.getNumberOfObstaclesHit(participant_number=participant_number, refinement_or_prediction=refinement_or_prediction, method=method)
            plt.bar(self.object_positions_labels, [ x / self.num_trials * 100 for x in number_of_obstacles_hit ] )
            plt.title(self.methods_labels[method-1])
            plt.xlabel("Object position [-]")
            plt.ylabel("Obstacles hit [%]")
            plt.tight_layout()
        
        plt.savefig(self.figures_path + 'participant_' + str(participant_number) + '/number_of_obstacles_hit_' + str(refinement_or_prediction) + '.pdf')

        plt.figure()

    def plotSuccesfullPredictions(self, participant_number):
        plt.figure()
        for method in range(1, self.num_methods+1):
            plt.subplot(2, 2, method)
            number_of_success = self.getSuccessfulTrials(participant_number=participant_number, refinement_or_prediction='prediction', method=method)
            plt.bar(self.object_positions_labels, [ x / self.num_trials * 100 for x in number_of_success ] )
            plt.title(self.methods_labels[method-1])
            plt.xlabel("Object position [-]")
            plt.ylabel("Successfull predictions [%]")
            plt.tight_layout()
        
        plt.savefig(self.figures_path + 'participant_' + str(participant_number) + '/successful_predictions.pdf')

        plt.figure()

    def plotSuccesfullRefinements(self, participant_number):
        plt.figure()
        for method in range(1, self.num_methods+1):
            plt.subplot(2, 2, method)
            number_of_success = self.getSuccessfulTrials(participant_number=participant_number, refinement_or_prediction='refinement', method=method)
            plt.bar(self.object_positions_labels, [ x / self.num_trials * 100 for x in number_of_success ] )
            plt.title(self.methods_labels[method-1])
            plt.xlabel("Object position [-]")
            plt.ylabel("Successfull refinements [%]")
            plt.tight_layout()
        
        plt.savefig(self.figures_path + 'participant_' + str(participant_number) + '/successful_refinements.pdf')

        plt.figure()


    def plotExperimentData(self, *args, **kwargs):
        if "participant_number" in kwargs:
            participant_number = kwargs["participant_number"]
            path = self.data_path + "participant_" + str(participant_number) + '/after_experiment/data.txt'
        else:
            path = self.data_path + 'before_experiment/dishwasher2/data.txt'

        num_object_positions = 6
        object_labels = ["1", "2", "3", "4", "5", "6"]
        
        with open(path, "r") as infile:
            outfile = ast.literal_eval(infile.read())
            data_before_experiment = outfile

            success_list = []
            object_missed_list = []
            obstacle_hit_list = []
            object_kicked_over_list = []

            for position in range(1,num_object_positions+1):
                success = 0
                object_missed = 0
                obstacle_hit = 0
                object_kicked_over = 0

                for trial in range(1,self.num_trials+1):
                    success += int(data_before_experiment[position]['trial'][trial]['predicted_trajectory']['success'])
                    object_missed += int(data_before_experiment[position]['trial'][trial]['predicted_trajectory']['object_missed'])
                    obstacle_hit += int(data_before_experiment[position]['trial'][trial]['predicted_trajectory']['obstacle_hit'])
                    object_kicked_over += int(data_before_experiment[position]['trial'][trial]['predicted_trajectory']['object_kicked_over'])
                
                success_list.append(success)
                object_missed_list.append(object_missed)
                obstacle_hit_list.append(obstacle_hit)
                object_kicked_over_list.append(object_kicked_over)
            
            fig = plt.figure()
            
            if "participant_number" in kwargs:
                fig.suptitle("Model after experiment")
            else:
                fig.suptitle("Initial model")

            plt.subplot(1,4,1)
            plt.bar(object_labels, np.asarray(success_list)/self.num_trials*100)
            plt.xlabel("Object position [-]")
            plt.ylabel("Success [True/False]")
            plt.ylim([0,100])

            plt.subplot(1,4,2)
            plt.bar(object_labels, np.asarray(object_missed_list)/self.num_trials * 100)
            plt.xlabel("Object position [-]")
            plt.ylabel("Object missed [True/False]")
            plt.ylim([0,100])

            plt.subplot(1,4,3)
            plt.bar(object_labels, np.asarray(obstacle_hit_list)/self.num_trials * 100)
            plt.xlabel("Object position [-]")
            plt.ylabel("Obstacle hit [True/False]")
            plt.ylim([0,100])

            plt.subplot(1,4,4)
            plt.bar(object_labels, np.asarray(object_kicked_over_list)/self.num_trials * 100)
            plt.xlabel("Object position [-]")
            plt.ylabel("Object kicked over [True/False]")
            plt.ylim([0,100])

            plt.tight_layout()
            fig.subplots_adjust(top=0.88)

        if "participant_number" in kwargs:
            plt.savefig(self.figures_path + "participant_" + str(participant_number) + "/after_experiment/success.pdf")            
        else:
            plt.savefig(self.figures_path + "/before_experiment/success.pdf")

if __name__ == "__main__":
    data_analysis = DataAnalysis()
    number = 7
    data_analysis.loadData(number)
    # data_analysis.plotPrediction(1, 4, 1, 1, 1)
    # data_analysis.plotRefinement(1, 4, 1, 1, 1)
    # print(data_analysis.getTime(1, 3, 1, 1, 1))
    # data_analysis.calculateRefinementTime(1, 3, 1)
    data_analysis.plotRefinementTime(number)
    data_analysis.plotNumberOfRefinements(number)

    data_analysis.plotSuccesfullPredictions(number)
    data_analysis.plotSuccesfullRefinements(number)

    data_analysis.plotNumberOfObstaclesHit(number, 'refinement')
    data_analysis.plotNumberOfObstaclesHit(number, 'prediction')

    data_analysis.plotNumberOfObjectMissed(number, 'refinement')
    data_analysis.plotNumberOfObjectMissed(number, 'prediction')

    data_analysis.plotNumberOfObjectKickedOver(number, 'refinement')
    data_analysis.plotNumberOfObjectKickedOver(number, 'prediction')

    # data_analysis.plotExperimentData()
    data_analysis.plotExperimentData(participant_number = number)