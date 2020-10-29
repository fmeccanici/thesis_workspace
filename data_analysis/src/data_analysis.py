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

        self.methods = self.experiment_variables.method_mapping_number_to_str
        self.best_methods = {"Participant 1": "offline+omni", "Participant 2": "offline+omni", "Participant 3": "online+omni", "Participant 4": "offline+keyboard", "Participant 5": "online+omni",
                            "Participant 6": "online+omni", "Participant 7": "online+keyboard", "Participant 8": "online+keyboard", "Participant 9": "online+omni", "Participant 10": "offline+keyboard",
                            "Participant 11": "online+keyboard", "Participant 12": "online+omni","Participant 13": "offline+omni", "Participant 14": "online+omni", "Participant 15": "offline+keyboard",
                            "Participant 16": "online+omni", "Participant 17": "online+keyboard", "Participant 18": "online+omni"}

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
            training_path = self.data_path + "participant_" + str(participant.getNumber()) + '/training/' + method_str + '/data.txt'
            with open(nasa_tlx_path, 'r') as f:
                workload = float(ast.literal_eval(f.read())['Workload'])
            try:
                with open(training_path, 'r') as f:
                    training_time = ast.literal_eval(f.read())['time']
            except FileNotFoundError:
                training_time = np.nan

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
                    data_dictionary = {'mechanism' : mechanism, 'interface' : interface, 'is_adapted' : is_adapted_per_model[i], 'refinement_time': refinement_time_per_model[i], 'number_of_refinements': number_of_refinements_per_model[i], 'number_of_updates' : number_of_updates_per_model[i], 'participant_number' : participant.getNumber(), 'model' : str(i+1), 'keyboard_experience' : participant.getKeyboardExperience(), 'teleop_experience' : participant.getTeleopExperience(), 'field_of_study': participant.getFieldOfStudy(), 'workload' : workload, 'training_time': training_time}
                    self.rows_list.append(data_dictionary)
                else:
                    drop_data = True
                    data_dictionary = {'mechanism' : mechanism, 'interface' : interface, 'is_adapted' : is_adapted_per_model[i], 'refinement_time': np.nan, 'number_of_refinements': number_of_refinements_per_model[i], 'number_of_updates' : number_of_updates_per_model[i], 'participant_number' : participant.getNumber(), 'model' : str(i+1), 'keyboard_experience' : participant.getKeyboardExperience(), 'teleop_experience' : participant.getTeleopExperience(), 'field_of_study': participant.getFieldOfStudy(), 'workload' : workload, 'training_time': training_time}
                    self.rows_list.append(data_dictionary)
    
    def plotMethodOpinions(self):
        plt.figure()
        plt.title("Method that was liked the most")
        plt.ylabel('Amount of participants')

        best_methods = list(self.best_methods.values())
        online_omni = best_methods.count('online+omni')            
        online_keyboard = best_methods.count('online+keyboard')            
        offline_omni = best_methods.count('offline+omni')            
        offline_keyboard = best_methods.count('offline+keyboard') 
        plt.bar(['online+omni', 'online+keyboard', 'offline+omni', 'offline+keyboard'], [online_omni, online_keyboard, offline_omni, offline_keyboard])           
        plt.savefig('method_opinion.png')

    def plotTrainingTime(self):
        training_time_online_omni = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'online')]['training_time']
        training_time_offline_omni = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'offline')]['training_time']
        training_time_online_keyboard = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'online')]['training_time']
        training_time_offline_keyboard = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'offline')]['training_time']

        fig = plt.figure()
        fig.suptitle("Training time per method")
        plt.subplot(2, 2, 1)
        sns.boxplot(x=training_time_online_omni, orient='v')
        plt.xlabel("online + omni")
        plt.ylabel("Time [s]")
        plt.ylim([0, 1500])

        plt.subplot(2, 2, 2)
        sns.boxplot(x=training_time_offline_omni, orient='v')
        plt.xlabel("offline + omni")
        plt.ylabel("Time [s]")
        plt.ylim([0, 1500])

        plt.subplot(2, 2, 3)
        sns.boxplot(x=training_time_online_keyboard, orient='v')
        plt.xlabel("online + keyboard")
        plt.ylabel("Time [s]")
        plt.ylim([0, 1500])

        plt.subplot(2, 2, 4)
        sns.boxplot(x=training_time_offline_keyboard, orient='v')
        plt.xlabel("offline + keyboard")
        plt.ylabel("Time [s]")
        plt.ylim([0, 1500])

        plt.tight_layout()
        fig.subplots_adjust(top=0.88)
        plt.savefig('training_time.png')

    def plotTeleopGameExperience(self):
        models_1 = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'online') & (self.df['model'] == '1')]
        game_experience = models_1['keyboard_experience']
        teleop_experience = models_1['teleop_experience']
        binwidth = 1

        fig = plt.figure()

        data = game_experience
        plt.subplot(2,1,1)
        plt.title("Game (WASD) experience")
        plt.xticks(np.arange(6), ("None", "1 hour", "10 hours", "1 day", "10 weeks", "More"))
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        data = teleop_experience
        plt.subplot(2,1,2)
        plt.title("Teleoperation experience")
        plt.xticks(np.arange(6), ("None", "1 hour", "10 hours", "1 day", "10 weeks", "More"))
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.tight_layout()
        fig.subplots_adjust(top=0.88)

        plt.savefig('teleop_game_experience.png')
    
    def printFieldOfStudy(self):
        models_1 = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'online') & (self.df['model'] == '1')]
        field_of_study = models_1[['field_of_study', 'participant_number']]
        print(field_of_study)
        technical_strings = ["Robotics", "Robotics Engineering", "BME", "BMD", "BMD-HI", "Electrical Engineering", "DCSC", "Mechatronics", "Mechanical Engineering", "Software", "IPO"]
        technical = []
        non_technical = []

        for i, data in enumerate(list(field_of_study['field_of_study'])):
            if data in technical_strings:                
                technical.append(i+1)
            else: 
                non_technical.append(i+1)
        
        fig = plt.figure()
        plt.title("Field of work/study")
        plt.bar(x=["Technical", "Non-technical"], height = [len(technical), len(non_technical)])
        plt.savefig("technical_non_technical.png")

    def calculateStatisticsValuesTeleopGameExperience(self):

        refinement_time_keyboard_large_game_experience = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_keyboard_low_game_experience = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_omni_large_teleop_experience = self.df.loc[(self.df['interface'] == 'omni') & (self.df['teleop_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_omni_low_teleop_experience = self.df.loc[(self.df['interface'] == 'omni') & (self.df['teleop_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        
        workload_keyboard_large_game_experience = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_keyboard_low_game_experience = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_omni_large_teleop_experience = self.df.loc[(self.df['interface'] == 'omni') & (self.df['teleop_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_omni_low_teleop_experience = self.df.loc[(self.df['interface'] == 'omni') & (self.df['teleop_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['workload']

        print("keyboard refinement time low game experience median = " + str(np.median(refinement_time_keyboard_low_game_experience)))
        print("keyboard refinement time low game experience 25 = " + str(np.percentile(refinement_time_keyboard_low_game_experience, 25)))
        print("keyboard refinement time low game experience 75 = " + str(np.percentile(refinement_time_keyboard_low_game_experience, 75)))
        print("sample size = " + str(len(refinement_time_keyboard_low_game_experience)))

        print("keyboard refinement time high game experience median = " + str(np.median(refinement_time_keyboard_large_game_experience)))
        print("keyboard refinement time high game experience 25 = " + str(np.percentile(refinement_time_keyboard_large_game_experience, 25)))
        print("keyboard refinement time high game experience 75 = " + str(np.percentile(refinement_time_keyboard_large_game_experience, 75)))
        print("sample size = " + str(len(refinement_time_keyboard_large_game_experience)))

        print("omni refinement time low teleop experience median = " + str(np.median(refinement_time_omni_low_teleop_experience)))
        print("omni refinement time low teleop experience 25 = " + str(np.percentile(refinement_time_omni_low_teleop_experience, 25)))
        print("omni refinement time low teleop experience 75 = " + str(np.percentile(refinement_time_omni_low_teleop_experience, 75)))
        print("sample size = " + str(len(refinement_time_omni_low_teleop_experience)))

        print("omni refinement time high teleop experience median = " + str(np.median(refinement_time_omni_large_teleop_experience)))
        print("omni refinement time high teleop experience 25 = " + str(np.percentile(refinement_time_omni_large_teleop_experience, 25)))
        print("omni refinement time high teleop experience 75 = " + str(np.percentile(refinement_time_omni_large_teleop_experience, 75)))
        print("sample size = " + str(len(refinement_time_omni_large_teleop_experience)))
        
        print("keyboard workload low game experience median = " + str(np.median(workload_keyboard_low_game_experience)))
        print("keyboard workload low game experience 25 = " + str(np.percentile(workload_keyboard_low_game_experience, 25)))
        print("keyboard workload low game experience 75 = " + str(np.percentile(workload_keyboard_low_game_experience, 75)))
        print("sample size = " + str(len(workload_keyboard_low_game_experience)))

        print("keyboard workload high game experience median = " + str(np.median(workload_keyboard_large_game_experience)))
        print("keyboard workload high game experience 25 = " + str(np.percentile(workload_keyboard_large_game_experience, 25)))
        print("keyboard workload high game experience 75 = " + str(np.percentile(workload_keyboard_large_game_experience, 75)))
        print("sample size = " + str(len(workload_keyboard_large_game_experience)))
        
        print("omni workload low teleop experience median = " + str(np.median(workload_omni_low_teleop_experience)))
        print("omni workload low teleop experience 25 = " + str(np.percentile(workload_omni_low_teleop_experience, 25)))
        print("omni workload low teleop experience 75 = " + str(np.percentile(workload_omni_low_teleop_experience, 75)))
        print("sample size = " + str(len(workload_omni_low_teleop_experience)))

        print("omni workload high teleop experience median = " + str(np.median(workload_omni_large_teleop_experience)))        
        print("omni workload high teleop experience 25 = " + str(np.percentile(workload_omni_large_teleop_experience, 25)))        
        print("omni workload high teleop experience 75 = " + str(np.percentile(workload_omni_large_teleop_experience, 75)))        
        print("sample size = " + str(len(workload_omni_large_teleop_experience)))

    def plotNumberOfUpdates(self):

        binwidth = 1

        # omni/keyboard
        fig = plt.figure()
        number_of_updates_omni = self.df.loc[self.df['interface'] == 'omni']['number_of_updates']
        number_of_updates_keyboard = self.df.loc[self.df['interface'] == 'keyboard']['number_of_updates']

        mid = (fig.subplotpars.right + fig.subplotpars.left)/2

        plt.suptitle("Number of updates needed to adapt the model", x=mid)
        plt.subplot(2,1,1)
        plt.title("Omni")
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        data = number_of_updates_keyboard
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,1,2)
        plt.title("Keyboard")
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        data = number_of_updates_omni
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.tight_layout()
        fig.subplots_adjust(top=0.88)

        plt.savefig('number_of_updates_interface.pdf')

        # online/offline
        fig = plt.figure()
        number_of_updates_online = self.df.loc[self.df['mechanism'] == 'online']['number_of_updates']
        number_of_updates_offline = self.df.loc[self.df['mechanism'] == 'offline']['number_of_updates']

        mid = (fig.subplotpars.right + fig.subplotpars.left)/2

        plt.suptitle("Number of updates needed to adapt the model", x=mid)
        plt.subplot(2,1,1)
        plt.title("Online")
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        data = number_of_updates_online
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,1,2)
        plt.title("Offline")
        plt.xlim([1, 8])
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        data = number_of_updates_offline
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.tight_layout()
        fig.subplots_adjust(top=0.88)

        plt.savefig('number_of_updates_mechanism.pdf')

        # per method
        fig = plt.figure()
        plt.suptitle("Number of updates needed to adapt the model")
        plt.subplot(2,2,1)
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        plt.title("Online + Omni")
        number_of_updates_online_omni = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni')]['number_of_updates']
        print("online+omni median number of updates = " + str(np.median(list(number_of_updates_online_omni))))
        print("online+omni 25 percentile number of updates = " + str(np.percentile(list(number_of_updates_online_omni), 25)))
        print("online+omni 75 percentile number of updates = " + str(np.percentile(list(number_of_updates_online_omni), 75)))
        

        data = number_of_updates_online_omni
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,2,2)
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        plt.title("Online + Keyboard")

        number_of_updates_online_keyboard = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'keyboard')]['number_of_updates']
        print("online+keyboard median number of updates = " + str(np.median(list(number_of_updates_online_keyboard))))
        print("online+keyboard 25 percentile number of updates = " + str(np.percentile(list(number_of_updates_online_keyboard), 25)))
        print("online+keyboard 75 percentile number of updates = " + str(np.percentile(list(number_of_updates_online_keyboard), 75)))
        
        data = number_of_updates_online_keyboard
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,2,3)
        plt.ylabel("Occurence")
        plt.xlabel("Amount of updates")
        plt.title("Offline + Omni")

        number_of_updates_offline_omni = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'omni')]['number_of_updates']
        print("offline+omni median number of updates = " + str(np.median(list(number_of_updates_offline_omni))))
        print("offline+omni 25 percentile number of updates = " + str(np.percentile(list(number_of_updates_offline_omni), 25)))
        print("offline+omni 75 percentile number of updates = " + str(np.percentile(list(number_of_updates_offline_omni), 75)))
        
        data = number_of_updates_offline_omni
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,2,4)
        plt.ylabel("Occurence")
        plt.xlabel("Amount of updates")
        plt.title("Offline + Keyboard")
        number_of_updates_offline_keyboard = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'keyboard')]['number_of_updates']
        print("offline+keyboard median number of updates = " + str(np.median(list(number_of_updates_offline_keyboard))))
        print("offline+keyboard 25 percentile number of updates = " + str(np.percentile(list(number_of_updates_offline_keyboard), 25)))
        print("offline+keyboard 75 percentile number of updates = " + str(np.percentile(list(number_of_updates_offline_keyboard), 75)))
        
        data = number_of_updates_offline_keyboard
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))
        
        plt.tight_layout()
        fig.subplots_adjust(top=0.88)

        plt.savefig('number_of_updates_methods.pdf')

    def plotNumberOfRefinements(self):

        binwidth = 1

        # keyboard/omni & online/offline 
        fig = plt.figure()
        number_of_refinements_omni = self.df.loc[self.df['interface'] == 'omni']['number_of_refinements']
        number_of_refinements_keyboard = self.df.loc[self.df['interface'] == 'keyboard']['number_of_refinements']

        mid = (fig.subplotpars.right + fig.subplotpars.left)/2
        plt.suptitle("Number of refinements needed to adapt the model", x=mid)

        plt.subplot(2,1,1)
        plt.title("Omni")
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        data = number_of_refinements_omni
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,1,2)
        plt.title("Keyboard")
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        data = number_of_refinements_keyboard
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.tight_layout()
        fig.subplots_adjust(top=0.88)

        plt.savefig('number_of_refinements_interface.pdf')

        fig = plt.figure()
        number_of_refinements_online = self.df.loc[self.df['mechanism'] == 'online']['number_of_refinements']
        number_of_refinements_offline = self.df.loc[self.df['mechanism'] == 'offline']['number_of_refinements']

        mid = (fig.subplotpars.right + fig.subplotpars.left)/2
        plt.suptitle("Number of refinements needed to adapt the model", x=mid)
        
        plt.subplot(2,1,1)

        plt.title("Online")
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        plt.ylim([0, 30])
        data = number_of_refinements_online
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,1,2)
        plt.title("Offline")
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        plt.ylim([0, 30])
        data = number_of_refinements_offline
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.tight_layout()
        fig.subplots_adjust(top=0.88)

        plt.savefig('number_of_refinements_mechanism.pdf')
    
        fig = plt.figure()
        plt.suptitle("Number of refinements needed to adapt the model")
        plt.subplot(2,2,1)
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        plt.title("Online + Omni")
        number_of_refinements_online_omni = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni')]['number_of_refinements']
        print("online+omni median number of refinements = " + str(np.median(list(number_of_refinements_online_omni))))
        print("online+omni 25 percentile number of refinements = " + str(np.percentile(list(number_of_refinements_online_omni), 25)))
        print("online+omni 75 percentile number of refinements = " + str(np.percentile(list(number_of_refinements_online_omni), 75)))
        
        data = number_of_refinements_online_omni
        x = range(min(data), max(data) + 1)
        # plt.xticks(x)
        plt.xlim([0,23])
        plt.ylim([0, 20])

        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,2,2)
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        plt.title("Online + Keyboard")
        plt.ylim([0, 20])
        number_of_refinements_online_keyboard = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'keyboard')]['number_of_refinements']
        print("online+keyboard median number of refinements = " + str(np.median(list(number_of_refinements_online_keyboard))))
        print("online+keyboard 25 percentile number of refinements = " + str(np.percentile(list(number_of_refinements_online_keyboard), 25)))
        print("online+keyboard 75 percentile number of refinements = " + str(np.percentile(list(number_of_refinements_online_keyboard), 75)))
        
        data = number_of_refinements_online_keyboard
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))
        
        plt.subplot(2,2,3)
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        plt.title("Offline + Omni")
        plt.ylim([0, 20])

        number_of_refinements_offline_omni = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'omni')]['number_of_refinements']
        print("offline+omni median number of refinements = " + str(np.median(list(number_of_refinements_offline_omni))))
        print("offline+omni 25 percentile number of refinements = " + str(np.percentile(list(number_of_refinements_offline_omni), 25)))
        print("offline+omni 75 percentile number of refinements = " + str(np.percentile(list(number_of_refinements_offline_omni), 75)))
        
        data = number_of_refinements_offline_omni
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,2,4)
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        plt.title("Offline + Keyboard")
        plt.ylim([0, 20])

        number_of_refinements_offline_keyboard = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'keyboard')]['number_of_refinements']
        print("offline+keyboard median number of refinements = " + str(np.median(list(number_of_refinements_offline_keyboard))))
        print("offline+keyboard 25 percentile number of refinements = " + str(np.percentile(list(number_of_refinements_offline_keyboard), 25)))
        print("offline+keyboard 75 percentile number of refinements = " + str(np.percentile(list(number_of_refinements_offline_keyboard), 75)))
        
        data = number_of_refinements_offline_keyboard
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))
        
        plt.tight_layout()
        fig.subplots_adjust(top=0.88)

        plt.savefig('number_of_refinements_methods.pdf')

    def createDataFrame(self):
        self.df = pd.DataFrame(self.rows_list)
        self.df.to_csv('./data.csv')

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

        plt.savefig('distributions_refinement_time.png')

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
        plt.savefig('distributions_workload.png')
    
    def plotAndSaveRefinementTimeAndWorkload(self):
        p_dummy = 0.999

        plt.figure()
        plt.title("Refinement time")
        ax = sns.boxplot(x="mechanism", y="refinement_time", hue="interface", data=self.df)
        plt.ylabel("Time [s]")
        plt.xlabel("p = " + str(round(self.statistics_values['refinement_time']['mechanism']['p'], 4)))
        # plt.xlabel("p = " + str(p_dummy))

        plt.savefig('refinement_time_mechanism.png')

        plt.figure()
        plt.title("Workload")
        ax = sns.boxplot(x="mechanism", y="workload", hue="interface", data=self.df)
        plt.ylabel("Workload [0-100]")
        plt.xlabel("p = " + str(round(self.statistics_values['workload']['mechanism']['p'], 4)))
        # plt.xlabel("p = " + str(p_dummy))

        plt.savefig('workload_mechanism.png')
        
        plt.figure()
        plt.title("Refinement time")
        ax = sns.boxplot(x="interface", y="refinement_time", hue="mechanism", data=self.df)
        plt.ylabel("Time [s]")
        plt.xlabel("p = " + str(round(self.statistics_values['refinement_time']['interface']['p'], 4)))
        # plt.xlabel("p = " + str(p_dummy))

        plt.savefig('refinement_time_interface.png')

        plt.figure()
        plt.title("Workload")
        ax = sns.boxplot(x="interface", y="workload", hue="mechanism", data=self.df)
        plt.ylabel("Workload [0-100]")
        plt.xlabel("p = " + str(round(self.statistics_values['workload']['interface']['p'], 4)))
        # plt.xlabel("p = " + str(p_dummy))

        plt.savefig('workload_interface.png')
    

        refinement_time_online_omni = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'online')]['refinement_time']
        refinement_time_online_keyboard = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'online')]['refinement_time']
        refinement_time_offline_omni = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'offline')]['refinement_time']
        refinement_time_offline_keyboard = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'offline')]['refinement_time']

        workload_online_omni = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'online')]['workload']
        workload_online_keyboard = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'online')]['workload']
        workload_offline_omni = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'offline')]['workload']
        workload_offline_keyboard = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'offline')]['workload']

        fig = plt.figure()
        plt.suptitle("Refinement time per method")
        plt.subplot(2,2,1)
        plt.ylim([0, 1000])
        plt.xlabel("Online + Omni")
        plt.ylabel("Refinement time")

        print("online+omni median refinement time = " + str(np.median(list(refinement_time_online_omni))))
        print("online+omni 25 percentile refinement time = " + str(np.percentile(list(refinement_time_online_omni), 25)))
        print("online+omni 75 percentile refinement time = " + str(np.percentile(list(refinement_time_online_omni), 75)))
        
        sns.boxplot(list(refinement_time_online_omni), orient='v')


        plt.subplot(2,2,2)
        plt.ylim([0, 1000])
        plt.xlabel("Online + Keyboard")
        plt.ylabel("Refinement time")

        print("online+keyboard median refinement time = " + str(np.median(list(refinement_time_online_keyboard))))
        print("online+keyboard 25 percentile refinement time = " + str(np.percentile(list(refinement_time_online_keyboard), 25)))
        print("online+keyboard 75 percentile refinement time = " + str(np.percentile(list(refinement_time_online_keyboard), 75)))
        
        sns.boxplot(list(refinement_time_online_keyboard), orient='v')

        plt.subplot(2,2,3)
        plt.ylim([0, 1000])
        plt.xlabel("Offline + Omni")
        plt.ylabel("Refinement time")

        print("offline+omni median refinement time = " + str(np.median(list(refinement_time_offline_omni))))
        print("offline+omni 25 percentile refinement time = " + str(np.percentile(list(refinement_time_offline_omni), 25)))
        print("offline+omni 75 percentile refinement time = " + str(np.percentile(list(refinement_time_offline_omni), 75)))
        
        sns.boxplot(list(refinement_time_offline_omni), orient='v')

        plt.subplot(2,2,4)
        plt.ylim([0, 1000])
        plt.xlabel("Offline + Keyboard")
        plt.ylabel("Refinement time")

        print("offline+keyboard median = " + str(np.median(list(refinement_time_offline_keyboard))))
        print("offline+keyboard 25 percentile refinement time = " + str(np.percentile(list(refinement_time_offline_keyboard), 25)))
        print("offline+keyboard 75 percentile refinement time = " + str(np.percentile(list(refinement_time_offline_keyboard), 75)))
        
        sns.boxplot(list(refinement_time_offline_keyboard), orient='v')

        plt.tight_layout()
        fig.subplots_adjust(top=0.88)

        plt.savefig('refinement_time_methods.png')

        fig = plt.figure()
        plt.suptitle("Workload per method")
        plt.subplot(2,2,1)
        plt.ylim([0, 100])
        plt.xlabel("Online + Omni")
        plt.ylabel("Workload")

        print("online+omni median workload = " + str(np.median(list(workload_online_omni))))
        print("online+omni 25 percentile workload = " + str(np.percentile(list(workload_online_omni), 25)))
        print("online+omni 75 percentile workload = " + str(np.percentile(list(workload_online_omni), 75)))
        
        sns.boxplot(list(workload_online_omni), orient='v')

        plt.subplot(2,2,2)
        plt.ylim([0, 100])
        plt.xlabel("Online + Keyboard")
        plt.ylabel("Workload")
        print("online+keyboard median workload = " + str(np.median(list(workload_online_keyboard))))
        print("online+keyboard 25 percentile workload = " + str(np.percentile(list(workload_online_keyboard), 25)))
        print("online+keyboard 75 percentile workload = " + str(np.percentile(list(workload_online_keyboard), 75)))
        
        sns.boxplot(list(workload_online_keyboard), orient='v')

        plt.subplot(2,2,3)
        plt.ylim([0, 100])
        plt.xlabel("Offline + Omni")
        plt.ylabel("Workload")

        print("offline+omni median workload = " + str(np.median(list(workload_offline_omni))))
        print("offline+omni 25 percentile workload = " + str(np.percentile(list(workload_offline_omni), 25)))
        print("offline+omni 75 percentile workload = " + str(np.percentile(list(workload_offline_omni), 75)))
        
        sns.boxplot(list(workload_offline_omni), orient='v')

        plt.subplot(2,2,4)
        plt.ylim([0, 100])
        plt.xlabel("Offline + Keyboard")
        plt.ylabel("Workload")
        print("offline+keyboard median workload = " + str(np.median(list(workload_offline_keyboard))))
        print("offline+keyboard 25 percentile workload = " + str(np.percentile(list(workload_offline_keyboard), 25)))
        print("offline+keyboard 75 percentile workload = " + str(np.percentile(list(workload_offline_keyboard), 75)))
        
        sns.boxplot(list(workload_offline_keyboard), orient='v')

        plt.tight_layout()
        fig.subplots_adjust(top=0.88)

        plt.savefig('workload_methods.png')

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
    data_analysis.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

    data_analysis.plotTrainingTime()
    data_analysis.plotTeleopGameExperience()
    data_analysis.printFieldOfStudy()

    # print("Workload values are only valid (no nans in refinement time dropped)")
    # print("Also checking for normality and amount of successfully adapted models is valid here")
    # data_analysis.useValidParticipants()
    # data_analysis.calculateAndStoreTtestValues()
    # data_analysis.printStatisticValues()
    
    # data_analysis.plotAndSaveRefinementTimeAndWorkload()
    # data_analysis.plotDistributions()
    # data_analysis.plotAmountOfAdaptedModelsPerMethod()
    
    # print("Refinement time values are the only thing that's valid")
    data_analysis.useValidParticipants()
    # data_analysis.calculateAndStoreTtestValues()
    # data_analysis.printStatisticValues()
    
    # data_analysis.plotAndSaveRefinementTimeAndWorkload()
    # data_analysis.plotDistributions()
    # data_analysis.plotAmountOfAdaptedModelsPerMethod()
    
    # # plt.show()
    # data_analysis.plotNumberOfUpdates()
    # data_analysis.plotNumberOfRefinements()
    # data_analysis.plotMethodOpinions()


    data_analysis.calculateStatisticsValuesTeleopGameExperience()