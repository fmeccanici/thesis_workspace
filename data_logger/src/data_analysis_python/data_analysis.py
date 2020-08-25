import ast, os, sys
from data_logger_python.data_logger_python import ParticipantData
import matplotlib.pyplot as plt
import numpy as np
from experiment_variables.experiment_variables import ExperimentVariables
import seaborn as sns
import copy
import pandas as pd

class DataAnalysis(object):
    def __init__(self):
        self.experiment_variables = ExperimentVariables()
        self.data_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/'
        self.figures_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/figures/'
        self.data = {}
        self.num_methods = self.experiment_variables.num_methods
        self.num_object_positions = self.experiment_variables.num_object_positions
        self.num_trials = self.experiment_variables.num_trials

        self.methods_labels = ['online+omni', 'offline+omni', 'online+pendant', 'offline+pendant']
        self.object_positions_labels = [ str(int(x)) for x in range(1, self.num_object_positions+1) ]
        self.method_mapping = self.experiment_variables.method_mapping_str_to_number

        self.createDataTemplate()

        self.refinement_time_data = copy.deepcopy(self.data_template)
        self.number_of_refinements_data = copy.deepcopy(self.data_template)
        self.time_data = copy.deepcopy(self.data_template)
        self.time_per_refinement_data = copy.deepcopy(self.data_template)
        self.correct_predictions_data = copy.deepcopy(self.data_template)
        self.correct_refinements_data = copy.deepcopy(self.data_template)

        self.time_per_correct_prediction_data = copy.deepcopy(self.data_template)
        self.time_per_correct_refinement_data = copy.deepcopy(self.data_template)

        self.teleop_experience_data = []
        self.keyboard_experience_data = []
        self.age_data = []
        
        self.success_after_experiment_data = copy.deepcopy(self.data_template)
        self.object_missed_after_experiment_data = copy.deepcopy(self.data_template)
        self.object_kicked_over_after_experiment_data = copy.deepcopy(self.data_template)
        self.obstacle_hit_after_experiment_data = copy.deepcopy(self.data_template)

    def loadData(self, participant_number, what_to_plot):
        
        # dummy variables zeros --> data gets loaded in participantData class nonetheless
        participant = ParticipantData(participant_number, 0, 0, 0, 0, 0)
        self.data[participant_number] = participant        

        if what_to_plot == 'experiment':
            self.parseDataExperiment(participant_number)
        elif what_to_plot == 'after':
            self.parseDataAfterExperiment(participant_number)
        
        self.fillCompleteDataTemplate()

    def parseDataAfterExperiment(self, participant_number):
        for i, method_str in enumerate(self.methods_labels):
            path = self.data_path + "participant_" + str(participant_number) + "/after_experiment/"
            path += method_str + "/data.txt" 

            with open(path, "r") as infile:
                outfile = ast.literal_eval(infile.read())
                data_after_experiment = outfile

                for position in range(1,self.num_object_positions+1):
                    success = 0
                    object_missed = 0
                    obstacle_hit = 0
                    object_kicked_over = 0

                    for trial in range(1,self.num_trials+1):
                        success += int(data_after_experiment[position]['trial'][trial]['predicted_trajectory']['success'])
                        object_missed += int(data_after_experiment[position]['trial'][trial]['predicted_trajectory']['object_missed'])
                        obstacle_hit += int(data_after_experiment[position]['trial'][trial]['predicted_trajectory']['obstacle_hit'])
                        object_kicked_over += int(data_after_experiment[position]['trial'][trial]['predicted_trajectory']['object_kicked_over'])
                    
                    self.success_after_experiment_data[i+1][position].append(success)
                    self.object_missed_after_experiment_data[i+1][position].append(object_missed)
                    self.object_kicked_over_after_experiment_data[i+1][position].append(object_kicked_over)
                    self.obstacle_hit_after_experiment_data[i+1][position].append(obstacle_hit)

    def parseDataExperiment(self, participant_number):
        for method in range(1,self.num_methods + 1):
            time_per_object = self.calculateRefinementTime(self.data[participant_number].getNumber(), method)[1]
            number_of_refinements_per_object = self.getNumberOfRefinements(self.data[participant_number].getNumber(), method)
            
            number_of_refinements = np.asarray(self.getNumberOfRefinements(self.data[participant_number].getNumber(), method))
            time = np.asarray(self.calculateRefinementTime(self.data[participant_number].getNumber(), method)[1])
            time_per_refinement_per_object = time / number_of_refinements 
            correct_predictions_per_object = np.asarray(self.getSuccessfulTrials('prediction', self.data[participant_number].getNumber(), method))
            correct_refinements_per_object = np.asarray(self.getSuccessfulTrials('refinement', self.data[participant_number].getNumber(), method))

            time_per_correct_prediction = time / correct_predictions_per_object
            time_per_correct_refinement = time / correct_refinements_per_object

            correct_predictions_per_object = correct_predictions_per_object / self.experiment_variables.num_trials * 100
            correct_refinements_per_object = correct_refinements_per_object / self.experiment_variables.num_trials * 100

            teleop_experience = self.data[participant_number].getTeleopExperience()
            keyboard_experience = self.data[participant_number].getKeyboardExperience()
            age = self.data[participant_number].getAge()

            for obj in range(1, self.num_object_positions + 1):
                self.refinement_time_data[method][obj].append(time_per_object[obj-1])
                self.number_of_refinements_data[method][obj].append(number_of_refinements_per_object[obj-1])

                if number_of_refinements[obj-1] != 0:
                    self.time_per_refinement_data[method][obj].append(time_per_refinement_per_object[obj-1])
                else: 
                    self.time_per_refinement_data[method][obj].append(0.0)

                self.correct_predictions_data[method][obj].append(correct_predictions_per_object[obj-1])
                self.correct_refinements_data[method][obj].append(correct_refinements_per_object[obj-1])
                
                if correct_predictions_per_object[obj-1] != 0:
                    self.time_per_correct_prediction_data[method][obj].append(time_per_correct_prediction[obj-1])
                else:
                    self.time_per_correct_prediction_data[method][obj].append(0.0)

                if correct_refinements_per_object[obj-1] != 0:
                    self.time_per_correct_refinement_data[method][obj].append(time_per_correct_refinement[obj-1])
                else:
                    self.time_per_correct_refinement_data[method][obj].append(0.0)


            self.teleop_experience_data.append(teleop_experience)
            self.keyboard_experience_data.append(keyboard_experience)
            self.age_data.append(age)

    def fillCompleteDataTemplate(self):
        self.parsed_for_statistics_data = {'refinement_time':self.refinement_time_data,
        'number_of_refinement': self.number_of_refinements_data, 'time': self.time_data,
        'time_per_refinement': self.time_per_refinement_data, 'correct_predictions': self.correct_predictions_data,
        'correct_refinements': self.correct_refinements_data}

    def createDataTemplate(self):
        obj_dict = {}
        method_dict = {}
        for i in range(1,self.num_object_positions+1):
            obj_dict[i] = []
        
        for i in range(1,self.num_methods+1):
            method_dict[i] = copy.deepcopy(obj_dict)

        self.data_template = method_dict

    def createFiguresPaths(self, participant_number):
        path = self.getFiguresPathParticipant(participant_number)

        if not self.isPath(path):
            self.createPath(path)

        for method in self.methods_labels:
            if not self.isPath(path):
                self.createPath(path + '/' + str(method) + '/')

    def getFiguresPathParticipant(self, participant_number):
        return self.figures_path + 'participant_' + str(participant_number) + '/'

    def isPath(self, path):
        return os.path.exists(path)

    def createPath(self, path):
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
                    
                    if trajectory == 'refined_trajectory' and methods[method]['object_position'][object_position]['trial'][trial]['number_of_refinements'] == 0:
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
            plt.ylim((0,500))
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
            plt.ylim((0,100))

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
            plt.ylim((0,100))

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
            plt.ylim((0,100))

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
            plt.ylim([0,100])
            plt.tight_layout()
        
        plt.savefig(self.figures_path + 'participant_' + str(participant_number) + '/successful_predictions.pdf')

        plt.figure()

    def plotTimePerSuccesfullPredictions(self, participant_number):
        plt.figure()
        for method in range(1, self.num_methods+1):
            plt.subplot(2, 2, method)
            number_of_success = self.getSuccessfulTrials(participant_number=participant_number, refinement_or_prediction='prediction', method=method)
            time_per_succesfull_prediction = np.asarray(self.calculateRefinementTime(self.data[participant_number].getNumber(), method)[1]) / np.asarray(number_of_success)
            
            plt.bar(self.object_positions_labels, time_per_succesfull_prediction)
            plt.title(self.methods_labels[method-1])
            plt.xlabel("Object position [-]")
            plt.ylabel("Time [s]")
            plt.ylim([0,100])
            plt.tight_layout()
        
        plt.savefig(self.figures_path + 'participant_' + str(participant_number) + '/time_per_successful_prediction.pdf')

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
            path = self.data_path + "participant_" + str(participant_number) + '/after_experiment/' + str(method) + '/data.txt'
        else:
            path = self.data_path + 'before_experiment/dishwasher2/data.txt'

        object_labels = [ str(int(x)) for x in range(1, self.num_object_positions+1) ]
        
        with open(path, "r") as infile:
            outfile = ast.literal_eval(infile.read())
            data_before_experiment = outfile

            success_list = []
            object_missed_list = []
            obstacle_hit_list = []
            object_kicked_over_list = []

            for position in range(1,self.num_object_positions+1):
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
            path = self.figures_path + "participant_" + str(participant_number) + "/after_experiment/" + str(method) + "/"
            if not os.path.exists(path):
                os.makedirs(path)
            
            plt.savefig(path + "success.pdf")            
        
        else:
            plt.savefig(self.figures_path + "/before_experiment/success.pdf")

    def generateBoxPlots(self, path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/figures/overall/'):
        
        fig = plt.figure()
        for i in range(1,self.num_methods+1):
            to_plot = []

            for j in range(1,self.num_object_positions+1):
                to_plot.append(self.refinement_time_data[i][j])
            
            plt.subplot(2,2,i)
            plt.boxplot(to_plot)
            plt.title(self.methods_labels[i-1])
            plt.ylabel('Time [s]')
            plt.xlabel('Object position [-]')
            plt.tight_layout()
            plt.ylim([0,500])
            fig.subplots_adjust(top=0.88)

        plt.suptitle("Refinement time")
        
        plt.savefig(path+'refinement_time_per_object_per_method.pdf')

        fig = plt.figure()

        for i in range(1,self.num_methods+1):
            to_plot = []

            for j in range(1,self.num_object_positions+1):
                to_plot.append(self.number_of_refinements_data[i][j])

            plt.subplot(2,2,i)
            plt.boxplot(to_plot)
            plt.title(self.methods_labels[i-1])
            plt.ylabel('Amount [-]')
            plt.xlabel('Object position [-]')
            plt.tight_layout()
            fig.subplots_adjust(top=0.88)

        plt.suptitle("Number of refinements")
        plt.savefig(path+'number_of_refinements_per_object_per_method.pdf')

        fig = plt.figure()

        for i in range(1,self.num_methods+1):
            to_plot = []

            for j in range(1,self.num_object_positions+1):
                to_plot.append(self.time_per_refinement_data[i][j])
            

            plt.subplot(2,2,i)
            plt.boxplot(to_plot)
            plt.title(self.methods_labels[i-1])
            plt.ylabel('Time [s]')
            plt.xlabel('Object position [-]')
            plt.tight_layout()
            fig.subplots_adjust(top=0.88)

        plt.suptitle("Time per refinement")
        plt.savefig(path+'time_per_refinement_per_object_per_method.pdf')

        fig = plt.figure()

        for i in range(1,self.num_methods+1):
            to_plot = []

            for j in range(1,self.num_object_positions+1):
                to_plot.append(self.correct_predictions_data[i][j])
            

            plt.subplot(2,2,i)
            plt.boxplot(to_plot)
            plt.title(self.methods_labels[i-1])
            plt.ylabel('Amount [%]')
            plt.xlabel('Object position [-]')
            plt.tight_layout()
            fig.subplots_adjust(top=0.88)

        plt.suptitle("Correct predictions")
        plt.savefig(path+'correct_predictions_per_object_per_method.pdf')

        fig = plt.figure()

        for i in range(1,self.num_methods+1):
            to_plot = []

            for j in range(1,self.num_object_positions+1):
                to_plot.append(self.correct_refinements_data[i][j])
            

            plt.subplot(2,2,i)
            plt.boxplot(to_plot)
            plt.title(self.methods_labels[i-1])
            plt.ylabel('Amount [%]')
            plt.xlabel('Object position [-]')
            plt.tight_layout()
            fig.subplots_adjust(top=0.88)

        plt.suptitle("Correct refinements")
        plt.savefig(path+'correct_refinements_per_object_per_method.pdf')

        for i in range(1,self.num_methods+1):
            to_plot = []

            for j in range(1,self.num_object_positions+1):
                to_plot.append(self.success_after_experiment_data[i][j])

            print(to_plot)
            plt.subplot(2,2,i)
            plt.boxplot(to_plot)
            plt.title(self.methods_labels[i-1])
            plt.ylabel('Success [%]')
            plt.xlabel('Object position [-]')
            plt.tight_layout()
            fig.subplots_adjust(top=0.88)

        plt.suptitle("Success after experiment")
        plt.savefig(path+'success_after_experiment_per_object_per_method.pdf')

        fig = plt.figure()

        for i in range(1,self.num_methods+1):
            to_plot = []

            for j in range(1,self.num_object_positions+1):
                to_plot.append(self.time_per_correct_prediction_data[i][j])
            
            plt.subplot(2,2,i)
            plt.boxplot(to_plot)
            plt.title(self.methods_labels[i-1])
            plt.ylabel('Time [s]')
            plt.xlabel('Object position [-]')
            plt.ylim([0,500])
            plt.tight_layout()
            
            fig.subplots_adjust(top=0.88)

        plt.suptitle("Time / amount of correct predictions")
        plt.savefig(path+'time_per_succesfull_prediction_per_object_per_method.pdf')

        fig = plt.figure()

        to_plot_mean = []

        for i in range(1,self.num_methods+1):
            to_plot = []

            for object_position in self.time_per_correct_prediction_data[i]:
                to_plot.append(np.mean(self.time_per_correct_prediction_data[i][object_position]))

            to_plot_mean.append(to_plot)
        
        plt.boxplot(to_plot_mean, labels=self.methods_labels)
        plt.title("Time / amount of succesfull predictions")
        plt.ylabel('Time [s]')
        plt.ylim([0,250])
        plt.tight_layout()
        plt.savefig(path+'time_per_succesfull_prediction_per_method.pdf')

        fig = plt.figure()

        to_plot_mean = []

        for i in range(1,self.num_methods+1):
            to_plot = []

            for object_position in self.time_per_correct_prediction_data[i]:
                to_plot.append(np.mean(self.time_per_correct_refinement_data[i][object_position]))

            to_plot_mean.append(to_plot)

        plt.boxplot(to_plot_mean, labels = self.methods_labels)

        plt.title("Time / amount of correct refinements")
        plt.ylabel('Time/succesfull refinements [s]')
        plt.ylim([0,250])
        plt.tight_layout()
        plt.savefig(path+'time_per_succesfull_refinement_per_method.pdf')

        fig = plt.figure()

        plt.subplot(2,2,1)
        plt.boxplot(self.teleop_experience_data)
        plt.title("Teloperation experience")
        plt.ylabel('Experience [1-5]')
        plt.ylim([0,5])

        plt.tight_layout()
        fig.subplots_adjust(top=0.88)

        plt.subplot(2,2,2)
        plt.boxplot(self.keyboard_experience_data)
        plt.title("Keyboard experience")
        plt.ylabel('Experience [1-5]')
        plt.ylim([0,5])
        plt.tight_layout()
        fig.subplots_adjust(top=0.88)

        plt.subplot(2,2,3)
        plt.boxplot(self.age_data)
        plt.title("Age")
        plt.ylabel('Age [-]')
        plt.ylim([20,70])

        plt.tight_layout()
        fig.subplots_adjust(top=0.88)
        plt.savefig(path+'teleop_keyboard_experience_age.pdf')

        plt.show()

if __name__ == "__main__":
    data_analysis = DataAnalysis()
    numbers = sys.argv[1]
    what_to_plot = sys.argv[2]
    numbers = ast.literal_eval(numbers)

    if isinstance(numbers, int):
        numbers = [numbers]

    for number in numbers:

        data_analysis.createFiguresPaths(number)
        data_analysis.loadData(number, what_to_plot)
        data_analysis.generateBoxPlots()
    
        if what_to_plot == 'experiment':
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
            data_analysis.plotTimePerSuccesfullPredictions(number)

            print('Figures of experiment stored')
        
        elif what_to_plot == 'after':
            for method in data_analysis.experiment_variables.method_mapping_str_to_number:
                try:
                    data_analysis.plotExperimentData(participant_number = number, method = method)   
                except Exception:
                    print('Data for method ' + str(method) + ' not found')
                    continue 
                print('Data for method ' + str(method) + ' plotted')

        elif what_to_plot == 'before':
            data_analysis.plotExperimentData()
            print('Figures before experiment stored')
    