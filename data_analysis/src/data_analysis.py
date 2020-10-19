# import own packages
from data_logger_python.data_logger_python import ParticipantData
from experiment_variables.experiment_variables import ExperimentVariables

# import external dependencies
import pandas as pd
import ast
import numpy as np
from scipy.stats import ttest_rel, wilcoxon
import seaborn as sns
import matplotlib.pyplot as plt

class DataAnalysis(object):
    def __init__(self):
        self.rows_list = []
        self.data_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/'
        self.figures_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/figures/'
        self.experiment_variables = ExperimentVariables()
        self.participant_data = {}
        self.statistics_values = {'refinement_time': {'mechanism': {'p' : None, 't' : None}, 'interface': {'p' : None, 't': None}}, 
                                'workload': {'mechanism': {'p' : None, 't': None}, 'interface': {'p' : None, 't': None}}}
    
    def isAdapted(self, participant_number, model, method):
        methods = self.participant_data[participant_number].getMethods()

        for object_position in methods[method]['model'][model]['object_position']:
            for trial in methods[method]['model'][model]['object_position'][object_position]['trial']:
                try:
                    if methods[method]['model'][model]['object_position'][object_position]['trial'][trial]['predicted_trajectory']['success'] == True:
                        # print("Model " + str(model) + " adapted")
                        return True

                except KeyError:
                    
                    # participant did not pass the training
                    if trial == 1:
                        print("Method " + str(self.experiment_variables.method_mapping_number_to_str[method])) 
                        print("Did not pass training")
                        return False

                    # model is already adapted --> No prediction made for other trials
                    else:
                        # print("Model " + str(model) + " adapted")
                        return True

        print("Method " + str(self.experiment_variables.method_mapping_number_to_str[method]))        
        print("Model " + str(model) + " not adapted")
        return False

    def loadMultipleParticipantsData(self, participant_numbers=[]):
        for participant_number in participant_numbers:
            self.AddAndConvertParticipantDataToDictionary(participant_number)
        
        self.createDataFrame()

    def AddAndConvertParticipantDataToDictionary(self, participant_number):
        self.addParticipantData(participant_number)
        self.convertParticipantDataToDictionary(self.participant_data[participant_number])

    def addParticipantData(self, participant_number):
        print("Added participant " + str(participant_number))
        participant = ParticipantData(participant_number, 0, 0, 0, 0, 0, 0)
        self.participant_data[participant_number] = participant

    def convertParticipantDataToDictionary(self, participant):        
        for method_str in self.experiment_variables.method_mapping_str_to_number:
            nasa_tlx_path = self.data_path + "participant_" + str(participant.getNumber()) + '/nasa_tlx/' + method_str + '/data.txt'
            with open(nasa_tlx_path, 'r') as f:
                workload = float(ast.literal_eval(f.read())['Workload'])

            refinement_time_per_model = self.calculateRefinementTime(participant, self.experiment_variables.method_mapping_str_to_number[method_str])
            mechanism = method_str.split('+')[0]
            interface = method_str.split('+')[1]

            if interface == 'pendant':
                interface = 'keyboard'
                
            number_of_refinements_per_model = self.getNumberOfRefinements(participant, self.experiment_variables.method_mapping_str_to_number[method_str])
            number_of_updates_per_model = self.getNumberOfUpdates(participant, self.experiment_variables.method_mapping_str_to_number[method_str])

            is_adapted_per_model = [self.isAdapted(participant.getNumber(), i, self.experiment_variables.method_mapping_str_to_number[method_str]) for i in range(1, len(refinement_time_per_model) + 1)]
            
            for i in range(len(refinement_time_per_model)):
                if is_adapted_per_model[i]:
                    data_dictionary = {'mechanism' : mechanism, 'interface' : interface, 'is_adapted' : is_adapted_per_model[i], 'refinement_time': refinement_time_per_model[i], 'number_of_refinements': number_of_refinements_per_model[i], 'number_of_updates' : number_of_updates_per_model[i], 'participant_number' : participant.getNumber(), 'model' : str(i+1), 'keyboard_experience' : participant.getKeyboardExperience(), 'teleop_experience' : participant.getTeleopExperience(), 'field_of_study': participant.getFieldOfStudy(), 'workload' : workload}
                    self.rows_list.append(data_dictionary)
                else:
                    drop_data = True
                    data_dictionary = {'mechanism' : mechanism, 'interface' : interface, 'is_adapted' : is_adapted_per_model[i], 'refinement_time': np.nan, 'number_of_refinements': number_of_refinements_per_model[i], 'number_of_updates' : number_of_updates_per_model[i], 'participant_number' : participant.getNumber(), 'model' : str(i+1), 'keyboard_experience' : participant.getKeyboardExperience(), 'teleop_experience' : participant.getTeleopExperience(), 'field_of_study': participant.getFieldOfStudy(), 'workload' : workload}
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
                    
                    time += self.getTime(participant, method, model, object_position, trial)
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

    def useValidParticipants(self):
        invalid_participants = list(set(list(self.df.loc[self.df['refinement_time'].isna()]['participant_number'])))
        print("Participants " + str(invalid_participants) + " don't have all models adapted")
        for invalid_participant in invalid_participants:
            self.df = self.df.loc[self.df['participant_number'] != invalid_participant]

    def calculateAndStoreTtestValues(self):

        refinement_time_omni = self.df.loc[self.df['interface'] == 'omni'].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_keyboard = self.df.loc[self.df['interface'] == 'keyboard'].sort_values(by = ['model', 'mechanism'])['refinement_time']

        # t, p = ttest_rel(refinement_time_keyboard, refinement_time_omni, nan_policy='omit')
        t, p = wilcoxon(refinement_time_keyboard, refinement_time_omni)

        self.statistics_values['refinement_time']['interface']['p'] = p
        self.statistics_values['refinement_time']['interface']['t'] = t

        refinement_time_online = list(self.df.loc[self.df['mechanism'] == 'online'].sort_values(by = ['model', 'interface'])['refinement_time'])
        refinement_time_offline = list(self.df.loc[self.df['mechanism'] == 'offline'].sort_values(by = ['model', 'interface'])['refinement_time'])

        # t, p = ttest_rel(refinement_time_online, refinement_time_offline, nan_policy='omit')
        t, p = wilcoxon(refinement_time_online, refinement_time_offline)
        
        self.statistics_values['refinement_time']['mechanism']['p'] = p
        self.statistics_values['refinement_time']['mechanism']['t'] = t

        workload_omni = list(self.df.loc[self.df['interface'] == 'omni'].sort_values(by = ['model', 'mechanism'])['workload'])
        workload_keyboard = list(self.df.loc[self.df['interface'] == 'keyboard'].sort_values(by = ['model', 'mechanism'])['workload'])

        # t, p = ttest_rel(workload_keyboard, workload_omni, nan_policy='omit')
        t, p = wilcoxon(workload_keyboard, workload_omni)
        
        self.statistics_values['workload']['interface']['p'] = p
        self.statistics_values['workload']['interface']['t'] = t

        workload_online = list(self.df.loc[self.df['mechanism'] == 'online'].sort_values(by = ['model', 'interface'])['workload'])
        workload_offline = list(self.df.loc[self.df['mechanism'] == 'offline'].sort_values(by = ['model', 'interface'])['workload'])

        # t, p = ttest_rel(workload_online, workload_offline, nan_policy='omit')                
        t, p = wilcoxon(workload_online, workload_offline)

        self.statistics_values['workload']['mechanism']['p'] = p
        self.statistics_values['workload']['mechanism']['t'] = t

    def printStatisticValues(self):
        print("Refinement time (Paired T-Test)")
        print('--------------------------------')
        print("H0: Online == Offline")
        print("p = " + str(self.statistics_values['refinement_time']['mechanism']['p'])) 
        print("t = " + str(self.statistics_values['refinement_time']['mechanism']['t'])) 
        print('\n')
        print("H0: Omni == Keyboard")
        print("p = " + str(self.statistics_values['refinement_time']['interface']['p'])) 
        print("t = " + str(self.statistics_values['refinement_time']['interface']['t'])) 
        print('\n')
        print("Workload (Wilcoxon)")
        print('--------------------------------')
        print("H0: Online == Offline")
        print("p = " + str(self.statistics_values['workload']['mechanism']['p'])) 
        print("t = " + str(self.statistics_values['workload']['mechanism']['t'])) 
        print('\n')
        print("H0: Omni == Keyboard")
        print("p = " + str(self.statistics_values['workload']['interface']['p'])) 
        print("t = " + str(self.statistics_values['workload']['interface']['t'])) 

    def plotDistributions(self):
        refinement_time_omni = self.df.loc[self.df['interface'] == 'omni']['refinement_time']
        refinement_time_keyboard = self.df.loc[self.df['interface'] == 'keyboard']['refinement_time']
        
        refinement_time_online = self.df.loc[self.df['mechanism'] == 'online']['refinement_time']
        refinement_time_offline = self.df.loc[self.df['mechanism'] == 'offline']['refinement_time']

        workload_omni = self.df.loc[self.df['interface'] == 'omni']['workload']
        workload_keyboard = self.df.loc[self.df['interface'] == 'keyboard']['workload']

        workload_online = self.df.loc[self.df['mechanism'] == 'online']['workload']
        workload_offline = self.df.loc[self.df['mechanism'] == 'offline']['workload']

        plt.figure()
        plt.suptitle("Refinement time")

        plt.subplot(2,2,1)
        plt.title("Omni")
        refinement_time_omni.hist()

        plt.subplot(2,2,2)
        plt.title("Keyboard")
        refinement_time_keyboard.hist()

        plt.subplot(2,2,3)
        plt.title("Online")
        refinement_time_online.hist()

        plt.subplot(2,2,4)
        plt.title("Offline")
        refinement_time_offline.hist()

        plt.tight_layout(h_pad=2)

        plt.figure()
        plt.suptitle("Workload")
        
        plt.subplot(2,2,1)
        plt.title("Omni")
        workload_omni.hist()
        
        plt.subplot(2,2,2)
        plt.title("Keyboard")
        workload_keyboard.hist()

        plt.subplot(2,2,3)
        plt.title("Online")
        workload_online.hist()

        plt.subplot(2,2,4)
        plt.title("Offline")
        workload_offline.hist()
        
        plt.tight_layout(h_pad=2)
    
    def plotAndSaveRefinementTimeAndWorkload(self):
        p_dummy = 0.999

        plt.figure()
        plt.title("Refinement time")
        ax = sns.boxplot(x="mechanism", y="refinement_time", hue="interface", data=self.df)
        plt.ylabel("Time [s]")
        # plt.xlabel("p = " + str(round(self.statistics_values['refinement_time']['mechanism']['p'], 4)))
        plt.xlabel("p = " + str(p_dummy))

        plt.savefig('refinement_time_mechanism.png')

        plt.figure()
        plt.title("Workload")
        ax = sns.boxplot(x="mechanism", y="workload", hue="interface", data=self.df)
        plt.ylabel("Workload [0-100]")
        # plt.xlabel("p = " + str(round(self.statistics_values['workload']['mechanism']['p'], 4)))
        plt.xlabel("p = " + str(p_dummy))

        plt.savefig('workload_mechanism.png')
        
        plt.figure()
        plt.title("Refinement time")
        ax = sns.boxplot(x="interface", y="refinement_time", hue="mechanism", data=self.df)
        plt.ylabel("Time [s]")
        # plt.xlabel("p = " + str(round(self.statistics_values['refinement_time']['interface']['p'], 4)))
        plt.xlabel("p = " + str(p_dummy))

        plt.savefig('refinement_time_interface.png')

        plt.figure()
        plt.title("Workload")
        ax = sns.boxplot(x="interface", y="workload", hue="mechanism", data=self.df)
        plt.ylabel("Workload [0-100]")
        # plt.xlabel("p = " + str(round(self.statistics_values['workload']['interface']['p'], 4)))
        plt.xlabel("p = " + str(p_dummy))

        plt.savefig('workload_interface.png')
    
    def plotAmountOfAdaptedModelsPerMethod(self):
        adapted_models_online_omni = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['is_adapted'] == True)]
        adapted_models_online_keyboard = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'keyboard') & (self.df['is_adapted'] == True)]
        adapted_models_offline_omni = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'omni') & (self.df['is_adapted'] == True)]
        adapted_models_offline_keyboard = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'keyboard') & (self.df['is_adapted'] == True)]
        

        plt.figure()
        plt.title("Adapted models")
        plt.bar(['online+omni', 'online+keyboard', 'offline+omni', 'offline+keyboard'], [len(adapted_models_online_omni), len(adapted_models_online_keyboard), len(adapted_models_offline_omni), len(adapted_models_offline_keyboard)])
        plt.savefig('adapted_models.png')

if __name__ == "__main__":
    data_analysis = DataAnalysis()
    data_analysis.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15])

    print("Workload values are only valid (no nans in refinement time dropped)")
    print("Also checking for normality and amount of successfully adapted models is valid here")
    data_analysis.calculateAndStoreTtestValues()
    data_analysis.printStatisticValues()
    
    data_analysis.plotAndSaveRefinementTimeAndWorkload()
    data_analysis.plotDistributions()
    data_analysis.plotAmountOfAdaptedModelsPerMethod()
    
    print("Refinement time values are the only thing that's valid")
    data_analysis.useValidParticipants()
    data_analysis.calculateAndStoreTtestValues()
    data_analysis.printStatisticValues()
    
    data_analysis.plotAndSaveRefinementTimeAndWorkload()
    data_analysis.plotDistributions()
    data_analysis.plotAmountOfAdaptedModelsPerMethod()
    
    plt.show()
    
