# import own packages
from data_logger_python.data_logger_python import ParticipantData
from experiment_variables.experiment_variables import ExperimentVariables

# import external dependencies
import pandas as pd
import ast
import numpy as np

class DataAnalysis(object):
    def __init__(self):
        self.rows_list = []
        self.data_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/'
        self.figures_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/figures/'
        self.experiment_variables = ExperimentVariables()

    def loadData(self, participant_number):
        participant = ParticipantData(participant_number, 0, 0, 0, 0, 0, 0)

        for method_str in self.experiment_variables.method_mapping_str_to_number:
            nasa_tlx_path = self.data_path + "participant_" + str(participant_number) + '/nasa_tlx/' + method_str + '/data.txt'
            with open(nasa_tlx_path, 'r') as f:
                workload = float(ast.literal_eval(f.read())['Workload'])

            refinement_time_per_model = self.calculateRefinementTime(participant, self.experiment_variables.method_mapping_str_to_number[method_str])
            online_offline = method_str.split('+')[0]
            omni_keyboard = method_str.split('+')[1]

            number_of_refinements_per_model = self.getNumberOfRefinements(participant, self.experiment_variables.method_mapping_str_to_number[method_str])
            number_of_updates_per_model = self.getNumberOfUpdates(participant, self.experiment_variables.method_mapping_str_to_number[method_str])
            

            # TODO
            # add function that calculates if model is adapted
            is_adapted_per_model = [False, False, False]
            
            for i in range(len(refinement_time_per_model)):
                data_dictionary = {'online_offline' : online_offline, 'omni_keyboard' : omni_keyboard, 'is_adapted' : is_adapted_per_model[i], 'number_of_refinements': number_of_refinements_per_model[i], 'number_of_updates' : number_of_updates_per_model[i], 'participant_number' : participant_number, 'model' : str(i+1), 'keyboard_experience' : participant.getKeyboardExperience(), 'teleop_experience' : participant.getTeleopExperience(), 'field_of_study': participant.getFieldOfStudy(), 'workload' : workload}
                self.rows_list.append(data_dictionary)

    def createDataFrame(self):
        self.df = pd.DataFrame(self.rows_list)
        
    def calculateRefinementTime(self, participant, method):
        methods = participant.getMethods()
        
        adaptation_time = []

        for model in methods[method]['model']:
            time = 0
            for object_position in methods[method]['model'][model]['object_position']:
                for trial in methods[method]['model'][model]['object_position'][object_position]['trial']:
                    
                    time += self.getTime(participant.getNumber(), method, model, object_position, trial)
            adaptation_time.append(time)

        return adaptation_time

    def getTime(self, participant, method, model, object_position, trial):
        methods = participant.getMethods()
        time = methods[method]['model'][model]['object_position'][object_position]['trial'][trial]['time']
        return time

    def getNumberOfRefinements(self, participant, method):
        methods = participant.getMethods()
        refinements_per_object_position = []
        refinements_per_model = []

        for model in methods[method]['model']:
            refinements = 0

            for object_position in methods[method]['model'][model]['object_position']:

                for trial in methods[method]['model'][model]['object_position'][object_position]['trial']:
                    refinements += methods[method]['model'][model]['object_position'][object_position]['trial'][trial]['number_of_refinements']

                refinements_per_object_position.append(refinements)
            
            refinements_per_model.append(refinements)

        return refinements_per_model

    def getNumberOfUpdates(self, participant, method):
        methods = participant.getMethods()

        number_of_updates_per_model = []

        for model in methods[method]['model']:
            number_of_updates = 0

            for object_position in methods[method]['model'][model]['object_position']:
                for trial in methods[method]['model'][model]['object_position'][object_position]['trial']:
                    if methods[method]['model'][model]['object_position'][object_position]['trial'][trial]['number_of_refinements'] != 0:
                        number_of_updates += 1
            
            number_of_updates_per_model.append(number_of_updates)

        return number_of_updates_per_model
