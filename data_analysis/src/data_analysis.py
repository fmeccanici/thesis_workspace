# import own packages
from data_logger_python.data_logger_python import ParticipantData
from experiment_variables.experiment_variables import ExperimentVariables

# import external dependencies
import pandas as pd
import ast
import numpy as np
from scipy.stats import ttest_rel, wilcoxon, mannwhitneyu
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

        """
        # Set plot style
        plot_style = {
            'axes.facecolor': '0.98',
            'axes.edgecolor': '.6',
            'axes.grid': True,
            'axes.axisbelow': True,
            'axes.labelcolor': '.3',
            'figure.facecolor': 'white',
            'grid.color': '.85',
            'grid.linestyle': '-',
            'text.color': '.4',
            'xtick.color': '.4',
            'ytick.color': '.4',
            'xtick.direction': 'out',
            'ytick.direction': 'out',
            'lines.solid_capstyle': 'round',
            'patch.edgecolor': 'w',
            'image.cmap': 'rocket',
            'font.family': ['sans-serif'],
            'font.sans-serif': ['Helvetica',
                'Arial',
                'DejaVu Sans',
                'Liberation Sans',
                'sans-serif'],
            'font.weight': '300',
            'patch.force_edgecolor': True,
            'xtick.bottom': True,
            'xtick.top': False,
            'ytick.left': True,
            'ytick.right': False,
            'axes.spines.left': True,
            'axes.spines.bottom': True,
            'axes.spines.right': False,
            'axes.spines.top': False,
            'figure.autolayout': True}

        plot_context = {
            'axes.linewidth': 0.6,
            'grid.linewidth': 0.6,
            'xtick.major.width': 0.6,
            'xtick.minor.width': 0.2,
            'ytick.major.width': 0.6,
            'ytick.minor.width': 0.2,
            'font.weight': 300
        }

        sns.set()
        sns.set_style("white", rc=plot_style)
        sns.set_context("notebook", rc=plot_context)
        """

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
    
    def getTopScore(self):
        # top_score = self.df.loc[(self.df['participant_number'] == 11) | (self.df['participant_number'] == 9)]
        # print(top_score[['refinement_time', 'participant_number', 'method']])
        top_score = self.df.loc[self.df.refinement_time == self.df.refinement_time.min()]
        print(top_score)

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

            technical_strings = ["Robotics", "Robotics Engineering", "BME", "BMD", "BMD-HI", "Electrical Engineering", "DCSC", "Mechatronics", "Mechanical Engineering", "Software", "IPO"]

            if participant.getFieldOfStudy() in technical_strings:
                technical = True
            else: technical = False

            if participant.getKeyboardExperience() > 3:
                high_keyboard_experience = True
            else: high_keyboard_experience = False

            if participant.getTeleopExperience() > 3:
                high_teleop_experience = True
            else: high_teleop_experience = False

            # method_str = mechanism + "+" + interface
            if method_str == 'online+omni':
                method_str = 'OnOm'
            elif method_str == 'online+pendant':
                method_str = 'OnKey'
            elif method_str == 'offline+omni':
                method_str = 'OffOm'            
            if method_str == 'offline+pendant':
                method_str = 'OffKey'
            
            if participant.getAge() > 30:
                old = True
            else: old = False

            for i in range(len(refinement_time_per_model)):
                if is_adapted_per_model[i]:
                    data_dictionary = {'method': method_str, 'mechanism' : mechanism, 'interface' : interface, 'is_adapted' : is_adapted_per_model[i], 'refinement_time': refinement_time_per_model[i], 'number_of_refinements': number_of_refinements_per_model[i], 'number_of_updates' : number_of_updates_per_model[i], 'participant_number' : participant.getNumber(), 'model' : str(i+1), 'keyboard_experience' : participant.getKeyboardExperience(), 'teleop_experience' : participant.getTeleopExperience(), 'field_of_study': participant.getFieldOfStudy(), 'technical': technical, 'high_teleop_experience': high_teleop_experience, 'high_keyboard_experience': high_keyboard_experience, 'workload' : workload, 'training_time': training_time, 'gender': participant.getGender(), 'right_handed': participant.getRightHanded(), 'age': participant.getAge(), 'old': old}
                    self.rows_list.append(data_dictionary)
                else:
                    drop_data = True
                    data_dictionary = {'method': method_str, 'mechanism' : mechanism, 'interface' : interface, 'is_adapted' : is_adapted_per_model[i], 'refinement_time': np.nan, 'number_of_refinements': number_of_refinements_per_model[i], 'number_of_updates' : number_of_updates_per_model[i], 'participant_number' : participant.getNumber(), 'model' : str(i+1), 'keyboard_experience' : participant.getKeyboardExperience(), 'teleop_experience' : participant.getTeleopExperience(), 'field_of_study': participant.getFieldOfStudy(), 'technical': technical, 'high_teleop_experience': high_teleop_experience, 'high_keyboard_experience': high_keyboard_experience, 'workload' : workload, 'training_time': training_time, 'gender': participant.getGender(), 'right_handed': participant.getRightHanded(), 'age': participant.getAge(), 'old': old}
                    self.rows_list.append(data_dictionary)
    
    def plotMethodOpinions(self):
        plt.figure()
        # plt.title("Method that was liked the most")
        plt.ylabel('Amount of participants [-]', fontsize=20)

        best_methods = list(self.best_methods.values())
        online_omni = best_methods.count('online+omni')            
        online_keyboard = best_methods.count('online+keyboard')            
        offline_omni = best_methods.count('offline+omni')            
        offline_keyboard = best_methods.count('offline+keyboard') 
        plt.bar(['OnOm', 'OnKey', 'OffOm', 'OffKey'], [online_omni, online_keyboard, offline_omni, offline_keyboard])           
        plt.xticks(fontsize=20, rotation=30)
        plt.yticks(fontsize=20)
        plt.tight_layout()
        plt.savefig('method_opinion.pdf')

    def plotTrainingTime(self):
        training_time_online_omni = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'online')]['training_time']
        training_time_offline_omni = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'offline')]['training_time']
        training_time_online_keyboard = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'online')]['training_time']
        training_time_offline_keyboard = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'offline')]['training_time']

        fig = plt.figure()
        # fig.suptitle("Training time per method")
        # plt.subplot(2, 2, 1)
        # print(self.df.loc[(self.df['model'] == '1') & (self.df['is_adapted'] == False)][['training_time', 'participant_number', 'method']])
        ax = sns.boxplot(data=self.df.loc[self.df['model'] == '1'], x='method', y='training_time')
        ax.set_xticklabels(ax.get_xticklabels(), rotation=30)

        # plt.xlabel("online + omni")
        plt.ylabel("Training time [s]", fontsize=20)
        plt.ylim([0, 1500])
        plt.xticks(fontsize=20, rotation=30)
        plt.yticks(fontsize=20)
        plt.xlabel("")
        # plt.subplot(2, 2, 2)
        # sns.boxplot(x=training_time_offline_omni, orient='v')
        # plt.xlabel("offline + omni")
        # plt.ylabel("Time [s]")
        # plt.ylim([0, 1500])

        # # plt.subplot(2, 2, 3)
        # sns.boxplot(x=training_time_online_keyboard, orient='v')
        # plt.xlabel("online + keyboard")
        # plt.ylabel("Time [s]")
        # plt.ylim([0, 1500])

        # # plt.subplot(2, 2, 4)
        # sns.boxplot(x=training_time_offline_keyboard, orient='v')
        # plt.xlabel("offline + keyboard")
        # plt.ylabel("Time [s]")
        # plt.ylim([0, 1500])

        # plt.tight_layout()
        # fig.subplots_adjust(top=0.88)
        plt.tight_layout()
        plt.savefig('training_time.pdf')

    def plotBackgroundInfo(self):
        models_1 = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'online') & (self.df['model'] == '1')]
        game_experience = models_1['keyboard_experience']
        teleop_experience = models_1['teleop_experience']
        technical = models_1['technical']
        right_handed = models_1['right_handed']
        gender = models_1['gender']
        age = models_1['age']
        binwidth = 1

        fig = plt.figure()
        data = technical
        plt.subplot(3,2,1)
        plt.title("Field of work")
        plt.ylabel("Participants [-]")
        plt.xticks(np.arange(2), ("Non-technical", "Technical"))
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        data = game_experience        
        plt.subplot(3,2,2)
        plt.title("Game (WASD) experience")
        plt.ylabel("Participants [-]")
        plt.xticks(np.arange(6), ("None", "1h", "10h", "1d", "10w", "More"), rotation=30)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        data = teleop_experience
        plt.subplot(3,2,3)
        plt.title("Teleoperation experience")
        plt.ylabel("Participants [-]")
        plt.xticks(np.arange(6), ("None", "1h", "10h", "1d", "10w", "More"), rotation=30)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        data = right_handed
        plt.subplot(3,2,4)
        plt.ylabel("Participants [-]")
        plt.title("Left/right handed")
        plt.xticks(np.arange(2), ("Left", "Right"))
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        data = gender
        plt.subplot(3,2,5)
        plt.title("Gender")
        plt.ylabel("Participants [-]")
        plt.xticks(np.arange(2), ("Female", "Male"))
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        data = age
        plt.subplot(3,2,6)
        plt.title("Age")
        plt.ylabel("Participants [-]")
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.tight_layout()
        fig.subplots_adjust(top=0.88)

        plt.savefig('background_info.pdf')
    
    def printFieldOfStudy(self):
        models_1 = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'online') & (self.df['model'] == '1')]
        field_of_study = models_1[['field_of_study', 'participant_number']]
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

    def plotTechnicalNonTechnical(self):
        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        workload_technical_omni = self.df.loc[(self.df['technical'] == True) & (self.df['interface'] == 'omni')]['workload']
        workload_technical_keyboard = self.df.loc[(self.df['technical'] == True) & (self.df['interface'] == 'keyboard')]['workload']
        workload_non_technical_omni = self.df.loc[(self.df['technical'] == False) & (self.df['interface'] == 'omni')]['workload']
        workload_non_technical_keyboard = self.df.loc[(self.df['technical'] == False) & (self.df['interface'] == 'keyboard')]['workload']

        self.useValidParticipants()

        # for values
        refinement_time_technical_omni = self.df.loc[(self.df['technical'] == True) & (self.df['interface'] == 'omni')]['refinement_time']
        refinement_time_technical_keyboard = self.df.loc[(self.df['technical'] == True) & (self.df['interface'] == 'keyboard')]['refinement_time']
        refinement_time_non_technical_omni = self.df.loc[(self.df['technical'] == False) & (self.df['interface'] == 'omni')]['refinement_time']
        refinement_time_non_technical_keyboard = self.df.loc[(self.df['technical'] == False) & (self.df['interface'] == 'keyboard')]['refinement_time']

        print("Refinement time")
        print('---------------')
        print("Technical")
        print("Omni")
        print("Median = " + str(np.median(refinement_time_technical_omni)))
        print("25 = " + str(np.percentile(refinement_time_technical_omni, 25)))
        print("75 = " + str(np.percentile(refinement_time_technical_omni, 75)))
        print("N = " + str(len(refinement_time_technical_omni)))
        print("Keyboard")
        print("Median = " + str(np.median(refinement_time_technical_keyboard)))
        print("25 = " + str(np.percentile(refinement_time_technical_keyboard, 25)))
        print("75 = " + str(np.percentile(refinement_time_technical_keyboard, 75)))
        print("N = " + str(len(refinement_time_technical_keyboard)))
        print()
        print("Non-Technical")
        print("Omni")
        print("Median = " + str(np.median(refinement_time_non_technical_omni)))
        print("25 = " + str(np.percentile(refinement_time_non_technical_omni, 25)))
        print("75 = " + str(np.percentile(refinement_time_non_technical_omni, 75)))
        print("N = " + str(len(refinement_time_technical_omni)))
        print("Keyboard")
        print("Median = " + str(np.median(refinement_time_non_technical_keyboard)))
        print("25 = " + str(np.percentile(refinement_time_non_technical_keyboard, 25)))
        print("75 = " + str(np.percentile(refinement_time_non_technical_keyboard, 75)))
        print("N = " + str(len(refinement_time_non_technical_keyboard)))
        print()
        print('Workload')
        print('---------------')
        print("Technical")
        print("Omni")
        print("Median = " + str(np.median(workload_technical_omni)))
        print("25 = " + str(np.percentile(workload_technical_omni, 25)))
        print("75 = " + str(np.percentile(workload_technical_omni, 75)))
        print("N = " + str(len(workload_technical_omni)))
        print("Keyboard")
        print("Median = " + str(np.median(workload_technical_keyboard)))
        print("25 = " + str(np.percentile(workload_technical_keyboard, 25)))
        print("75 = " + str(np.percentile(workload_technical_keyboard, 75)))
        print("N = " + str(len(workload_technical_keyboard)))
        print()
        print("Non-Technical")
        print("Omni")
        print("Median = " + str(np.median(workload_non_technical_omni)))
        print("25 = " + str(np.percentile(workload_non_technical_omni, 25)))
        print("75 = " + str(np.percentile(workload_non_technical_omni, 75)))
        print("N = " + str(len(workload_non_technical_omni)))
        print("Keyboard")
        print("Median = " + str(np.median(workload_non_technical_keyboard)))
        print("25 = " + str(np.percentile(workload_non_technical_keyboard, 25)))
        print("75 = " + str(np.percentile(workload_non_technical_keyboard, 75)))
        print("N = " + str(len(workload_non_technical_keyboard)))

        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        workload_technical_online_omni = self.df.loc[(self.df['technical'] == True) & (self.df['interface'] == 'omni') & (self.df['mechanism'] == 'online')]['workload']
        workload_technical_offline_omni = self.df.loc[(self.df['technical'] == True) & (self.df['interface'] == 'omni') & (self.df['mechanism'] == 'offline')]['workload']
        workload_technical_online_keyboard = self.df.loc[(self.df['technical'] == True) & (self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'online')]['workload']
        workload_technical_offline_keyboard = self.df.loc[(self.df['technical'] == True) & (self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'offline')]['workload']
        workload_non_technical_online_omni = self.df.loc[(self.df['technical'] == False) & (self.df['interface'] == 'omni') & (self.df['mechanism'] == 'online')]['workload']
        workload_non_technical_offline_omni = self.df.loc[(self.df['technical'] == False) & (self.df['interface'] == 'omni') & (self.df['mechanism'] == 'offline')]['workload']
        workload_non_technical_online_keyboard = self.df.loc[(self.df['technical'] == False) & (self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'online')]['workload']
        workload_non_technical_offline_keyboard = self.df.loc[(self.df['technical'] == False) & (self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'offline')]['workload']

        self.useValidParticipants()

        refinement_time_technical_online_omni = self.df.loc[(self.df['technical'] == True) & (self.df['interface'] == 'omni') & (self.df['mechanism'] == 'online')]['refinement_time']
        refinement_time_technical_offline_omni = self.df.loc[(self.df['technical'] == True) & (self.df['interface'] == 'omni') & (self.df['mechanism'] == 'offline')]['refinement_time']
        refinement_time_technical_online_keyboard = self.df.loc[(self.df['technical'] == True) & (self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'online')]['refinement_time']
        refinement_time_technical_offline_keyboard = self.df.loc[(self.df['technical'] == True) & (self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'offline')]['refinement_time']
        refinement_time_non_technical_online_omni = self.df.loc[(self.df['technical'] == False) & (self.df['interface'] == 'omni') & (self.df['mechanism'] == 'online')]['refinement_time']
        refinement_time_non_technical_offline_omni = self.df.loc[(self.df['technical'] == False) & (self.df['interface'] == 'omni') & (self.df['mechanism'] == 'offline')]['refinement_time']
        refinement_time_non_technical_online_keyboard = self.df.loc[(self.df['technical'] == False) & (self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'online')]['refinement_time']
        refinement_time_non_technical_offline_keyboard = self.df.loc[(self.df['technical'] == False) & (self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'offline')]['refinement_time']



        print("Refinement time")
        print('---------------')
        print("Technical")
        print("Online + Omni")
        print("Median = " + str(np.median(refinement_time_technical_online_omni)))
        print("25 = " + str(np.percentile(refinement_time_technical_online_omni, 25)))
        print("75 = " + str(np.percentile(refinement_time_technical_online_omni, 75)))
        print("N = " + str(len(refinement_time_technical_online_omni)))

        print("Offline + Omni")
        print("Median = " + str(np.median(refinement_time_technical_offline_omni)))
        print("25 = " + str(np.percentile(refinement_time_technical_offline_omni, 25)))
        print("75 = " + str(np.percentile(refinement_time_technical_offline_omni, 75)))
        print("N = " + str(len(refinement_time_technical_offline_omni)))

        print("Online + Keyboard")
        print("Median = " + str(np.median(refinement_time_technical_online_keyboard)))
        print("25 = " + str(np.percentile(refinement_time_technical_online_keyboard, 25)))
        print("75 = " + str(np.percentile(refinement_time_technical_online_keyboard, 75)))
        print("N = " + str(len(refinement_time_technical_online_keyboard)))

        print("Offline + Keyboard")
        print("Median = " + str(np.median(refinement_time_technical_offline_keyboard)))
        print("25 = " + str(np.percentile(refinement_time_technical_offline_keyboard, 25)))
        print("75 = " + str(np.percentile(refinement_time_technical_offline_keyboard, 75)))
        print("N = " + str(len(refinement_time_technical_offline_keyboard)))

        print()
        print("Non-technical")
        print("Online + Omni")
        print("Median = " + str(np.median(refinement_time_non_technical_online_omni)))
        print("25 = " + str(np.percentile(refinement_time_non_technical_online_omni, 25)))
        print("75 = " + str(np.percentile(refinement_time_non_technical_online_omni, 75)))
        print("N = " + str(len(refinement_time_non_technical_online_omni)))

        print("Offline + Omni")
        print("Median = " + str(np.median(refinement_time_non_technical_offline_omni)))
        print("25 = " + str(np.percentile(refinement_time_non_technical_offline_omni, 25)))
        print("75 = " + str(np.percentile(refinement_time_non_technical_offline_omni, 75)))
        print("N = " + str(len(refinement_time_non_technical_offline_omni)))

        print("Online + Keyboard")
        print("Median = " + str(np.median(refinement_time_non_technical_online_keyboard)))
        print("25 = " + str(np.percentile(refinement_time_non_technical_online_keyboard, 25)))
        print("75 = " + str(np.percentile(refinement_time_non_technical_online_keyboard, 75)))
        print("N = " + str(len(refinement_time_non_technical_online_keyboard)))

        print("Offline + Keyboard")
        print("Median = " + str(np.median(refinement_time_non_technical_offline_keyboard)))
        print("25 = " + str(np.percentile(refinement_time_non_technical_offline_keyboard, 25)))
        print("75 = " + str(np.percentile(refinement_time_non_technical_offline_keyboard, 75)))
        print("N = " + str(len(refinement_time_non_technical_offline_keyboard)))

        print()
        print("Workload")
        print('---------------')
        print("Technical")
        print("Online + Omni")
        print("Median = " + str(np.median(workload_technical_online_omni)))
        print("25 = " + str(np.percentile(workload_technical_online_omni, 25)))
        print("75 = " + str(np.percentile(workload_technical_online_omni, 75)))
        print("N = " + str(len(workload_technical_online_omni)))

        print("Offline + Omni")
        print("Median = " + str(np.median(workload_technical_offline_omni)))
        print("25 = " + str(np.percentile(workload_technical_offline_omni, 25)))
        print("75 = " + str(np.percentile(workload_technical_offline_omni, 75)))
        print("N = " + str(len(workload_technical_offline_omni)))

        print("Online + Keyboard")
        print("Median = " + str(np.median(workload_technical_online_keyboard)))
        print("25 = " + str(np.percentile(workload_technical_online_keyboard, 25)))
        print("75 = " + str(np.percentile(workload_technical_online_keyboard, 75)))
        print("N = " + str(len(workload_technical_online_keyboard)))

        print("Offline + Keyboard")
        print("Median = " + str(np.median(workload_technical_offline_keyboard)))
        print("25 = " + str(np.percentile(workload_technical_offline_keyboard, 25)))
        print("75 = " + str(np.percentile(workload_technical_offline_keyboard, 75)))
        print("N = " + str(len(workload_technical_offline_keyboard)))

        print()
        print("Non-technical")
        print("Online + Omni")
        print("Median = " + str(np.median(workload_non_technical_online_omni)))
        print("25 = " + str(np.percentile(workload_non_technical_online_omni, 25)))
        print("75 = " + str(np.percentile(workload_non_technical_online_omni, 75)))
        print("N = " + str(len(workload_non_technical_online_omni)))

        print("Offline + Omni")
        print("Median = " + str(np.median(workload_non_technical_offline_omni)))
        print("25 = " + str(np.percentile(workload_non_technical_offline_omni, 25)))
        print("75 = " + str(np.percentile(workload_non_technical_offline_omni, 75)))
        print("N = " + str(len(workload_non_technical_offline_omni)))

        print("Online + Keyboard")
        print("Median = " + str(np.median(workload_non_technical_online_keyboard)))
        print("25 = " + str(np.percentile(workload_non_technical_online_keyboard, 25)))
        print("75 = " + str(np.percentile(workload_non_technical_online_keyboard, 75)))
        print("N = " + str(len(workload_non_technical_online_keyboard)))

        print("Offline + Keyboard")
        print("Median = " + str(np.median(workload_non_technical_offline_keyboard)))
        print("25 = " + str(np.percentile(workload_non_technical_offline_keyboard, 25)))
        print("75 = " + str(np.percentile(workload_non_technical_offline_keyboard, 75)))
        print("N = " + str(len(workload_non_technical_offline_keyboard)))




        ## for plots
        # fig = plt.figure()
        # plt.title("Field of study/work")
        # sns.boxplot(x="technical", y="refinement_time", data=self.df)
        # plt.xticks(np.arange(2), ["Non-technical", "Technical"])
        # plt.xlabel("")

        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        fig = plt.figure()
        plt.suptitle("Field of work/study")
        legend_labels = ["NT", "T"]

        # workload
        plt.subplot(2,2,3)
        g = sns.boxplot(x='interface', y='workload', data=self.df, hue='technical')
        g.legend_.remove()
        plt.ylabel('Workload [0-100]')
        plt.xlabel("")
        plt.title("")
        
        plt.subplot(2,2,4)
        g = sns.boxplot(x='mechanism', y='workload', data=self.df, hue='technical')
        g.legend_.remove()
        plt.ylabel('Workload [0-100]')
        plt.xlabel("")
        plt.title("")

        self.useValidParticipants()

        # refinement time
        plt.subplot(2,2,1)
        g = sns.boxplot(x='interface', y='refinement_time', data=self.df, hue='technical')
        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)
        plt.ylabel('Refinement time [s]')
        plt.xlabel("")
        plt.title("")

        plt.subplot(2,2,2)
        g = sns.boxplot(x='mechanism', y='refinement_time', data=self.df, hue='technical')
        g.legend_.remove()
        plt.ylabel('Refinement time [s]')
        plt.xlabel("")
        plt.title("")
        plt.tight_layout()
        fig.subplots_adjust(top=0.88)
        plt.savefig('field_of_work_omni_keyboard_offline_online.pdf')

        plt.show()



    def calculateStatisticsValuesAndPlotBackgroundInfo(self):
        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        ## for plots
        fig = plt.figure()
        plt.suptitle("Teleoperation experience")
        legend_labels = ["L", "H"]

        # teleop experience
        # workload
        plt.subplot(2,2,3)
        g = sns.boxplot(x='interface', y='workload', data=self.df, hue='high_teleop_experience')
        g.legend_.remove()
        plt.ylabel('Workload [0-100]')
        plt.xlabel("")
        plt.title("")
        
        plt.subplot(2,2,4)
        g = sns.boxplot(x='mechanism', y='workload', data=self.df, hue='high_teleop_experience')
        g.legend_.remove()
        plt.ylabel('Workload [0-100]')
        plt.xlabel("")
        plt.title("")

        self.useValidParticipants()

        # refinement time
        plt.subplot(2,2,1)
        g = sns.boxplot(x='interface', y='refinement_time', data=self.df, hue='high_teleop_experience')
        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)
        plt.ylabel('Refinement time [s]')
        plt.xlabel("")
        plt.title("")

        plt.subplot(2,2,2)
        g = sns.boxplot(x='mechanism', y='refinement_time', data=self.df, hue='high_teleop_experience')
        g.legend_.remove()
        plt.ylabel('Refinement time [s]')
        plt.xlabel("")
        plt.title("")
        plt.tight_layout()
        fig.subplots_adjust(top=0.88)
        plt.savefig('teleop_experience_omni_keyboard_offline_online.pdf')
        
        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])


        # keyboard experience
        fig = plt.figure()
        plt.suptitle("Keyboard experience")
        legend_labels = ["L", "H"]

        # workload
        plt.subplot(2,2,3)
        g = sns.boxplot(x='interface', y='workload', data=self.df, hue='high_keyboard_experience')
        g.legend_.remove()
        plt.ylabel('Workload [0-100]')
        plt.xlabel("")
        plt.title("")
        
        plt.subplot(2,2,4)
        g = sns.boxplot(x='mechanism', y='workload', data=self.df, hue='high_keyboard_experience')
        g.legend_.remove()
        plt.ylabel('Workload [0-100]')
        plt.xlabel("")
        plt.title("")

        self.useValidParticipants()

        # refinement time
        plt.subplot(2,2,1)
        g = sns.boxplot(x='interface', y='refinement_time', data=self.df, hue='high_keyboard_experience')
        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)
        plt.ylabel('Refinement time [s]')
        plt.xlabel("")
        plt.title("")

        plt.subplot(2,2,2)
        g = sns.boxplot(x='mechanism', y='refinement_time', data=self.df, hue='high_keyboard_experience')
        g.legend_.remove()
        plt.ylabel('Refinement time [s]')
        plt.xlabel("")
        plt.title("")
        plt.tight_layout()
        fig.subplots_adjust(top=0.88)
        plt.savefig('keyboard_experience_omni_keyboard_offline_online.pdf')

        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        ## for values
        workload_keyboard_large_game_experience = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_keyboard_low_game_experience = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_omni_large_game_experience = self.df.loc[(self.df['interface'] == 'omni') & (self.df['keyboard_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_omni_low_game_experience = self.df.loc[(self.df['interface'] == 'omni') & (self.df['keyboard_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['workload']

        workload_omni_large_teleop_experience = self.df.loc[(self.df['interface'] == 'omni') & (self.df['teleop_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_omni_low_teleop_experience = self.df.loc[(self.df['interface'] == 'omni') & (self.df['teleop_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_keyboard_large_teleop_experience = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['teleop_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_keyboard_low_teleop_experience = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['teleop_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['workload']
        
        self.useValidParticipants()

        refinement_time_keyboard_large_game_experience = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_keyboard_low_game_experience = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_omni_large_game_experience = self.df.loc[(self.df['interface'] == 'omni') & (self.df['keyboard_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_omni_low_game_experience = self.df.loc[(self.df['interface'] == 'omni') & (self.df['keyboard_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']

        refinement_time_omni_large_teleop_experience = self.df.loc[(self.df['interface'] == 'omni') & (self.df['teleop_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_omni_low_teleop_experience = self.df.loc[(self.df['interface'] == 'omni') & (self.df['teleop_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_keyboard_large_teleop_experience = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['teleop_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_keyboard_low_teleop_experience = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['teleop_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        

        print("keyboard refinement time low game experience median = " + str(np.median(refinement_time_keyboard_low_game_experience)))
        print("keyboard refinement time low game experience 25 = " + str(np.percentile(refinement_time_keyboard_low_game_experience, 25)))
        print("keyboard refinement time low game experience 75 = " + str(np.percentile(refinement_time_keyboard_low_game_experience, 75)))
        print("sample size = " + str(len(refinement_time_keyboard_low_game_experience)))

        print("keyboard refinement time high game experience median = " + str(np.median(refinement_time_keyboard_large_game_experience)))
        print("keyboard refinement time high game experience 25 = " + str(np.percentile(refinement_time_keyboard_large_game_experience, 25)))
        print("keyboard refinement time high game experience 75 = " + str(np.percentile(refinement_time_keyboard_large_game_experience, 75)))
        print("sample size = " + str(len(refinement_time_keyboard_large_game_experience)))

        print()

        print("omni refinement time high game experience median = " + str(np.median(refinement_time_omni_large_game_experience)))
        print("omni refinement time high game experience 25 = " + str(np.percentile(refinement_time_omni_large_game_experience, 25)))
        print("omni refinement time high game experience 75 = " + str(np.percentile(refinement_time_omni_large_game_experience, 75)))
        print("sample size = " + str(len(refinement_time_omni_large_game_experience)))

        print("omni refinement time low game experience median = " + str(np.median(refinement_time_omni_low_game_experience)))
        print("omni refinement time low game experience 25 = " + str(np.percentile(refinement_time_omni_low_game_experience, 25)))
        print("omni refinement time low game experience 75 = " + str(np.percentile(refinement_time_omni_low_game_experience, 75)))
        print("sample size = " + str(len(refinement_time_omni_low_game_experience)))

        print()



        print("omni refinement time low teleop experience median = " + str(np.median(refinement_time_omni_low_teleop_experience)))
        print("omni refinement time low teleop experience 25 = " + str(np.percentile(refinement_time_omni_low_teleop_experience, 25)))
        print("omni refinement time low teleop experience 75 = " + str(np.percentile(refinement_time_omni_low_teleop_experience, 75)))
        print("sample size = " + str(len(refinement_time_omni_low_teleop_experience)))

        print("omni refinement time high teleop experience median = " + str(np.median(refinement_time_omni_large_teleop_experience)))
        print("omni refinement time high teleop experience 25 = " + str(np.percentile(refinement_time_omni_large_teleop_experience, 25)))
        print("omni refinement time high teleop experience 75 = " + str(np.percentile(refinement_time_omni_large_teleop_experience, 75)))
        print("sample size = " + str(len(refinement_time_omni_large_teleop_experience)))
        

        print()



        print("keyboard refinement time low teleop experience median = " + str(np.median(refinement_time_keyboard_low_teleop_experience)))
        print("keyboard refinement time low teleop experience 25 = " + str(np.percentile(refinement_time_keyboard_low_teleop_experience, 25)))
        print("keyboard refinement time low teleop experience 75 = " + str(np.percentile(refinement_time_keyboard_low_teleop_experience, 75)))
        print("sample size = " + str(len(refinement_time_keyboard_low_teleop_experience)))

        print("keyboard refinement time high teleop experience median = " + str(np.median(refinement_time_keyboard_large_teleop_experience)))
        print("keyboard refinement time high teleop experience 25 = " + str(np.percentile(refinement_time_keyboard_large_teleop_experience, 25)))
        print("keyboard refinement time high teleop experience 75 = " + str(np.percentile(refinement_time_keyboard_large_teleop_experience, 75)))
        print("sample size = " + str(len(refinement_time_keyboard_large_teleop_experience)))

        print()


        print("keyboard workload low game experience median = " + str(np.median(workload_keyboard_low_game_experience)))
        print("keyboard workload low game experience 25 = " + str(np.percentile(workload_keyboard_low_game_experience, 25)))
        print("keyboard workload low game experience 75 = " + str(np.percentile(workload_keyboard_low_game_experience, 75)))
        print("sample size = " + str(len(workload_keyboard_low_game_experience)))

        print("keyboard workload high game experience median = " + str(np.median(workload_keyboard_large_game_experience)))
        print("keyboard workload high game experience 25 = " + str(np.percentile(workload_keyboard_large_game_experience, 25)))
        print("keyboard workload high game experience 75 = " + str(np.percentile(workload_keyboard_large_game_experience, 75)))
        print("sample size = " + str(len(workload_keyboard_large_game_experience)))
        
        print()


        print("omni workload low game experience median = " + str(np.median(workload_omni_low_game_experience)))
        print("omni workload low game experience 25 = " + str(np.percentile(workload_omni_low_game_experience, 25)))
        print("omni workload low game experience 75 = " + str(np.percentile(workload_omni_low_game_experience, 75)))
        print("sample size = " + str(len(workload_omni_low_game_experience)))

        print("omni workload high game experience median = " + str(np.median(workload_omni_large_game_experience)))
        print("omni workload high game experience 25 = " + str(np.percentile(workload_omni_large_game_experience, 25)))
        print("omni workload high game experience 75 = " + str(np.percentile(workload_omni_large_game_experience, 75)))
        print("sample size = " + str(len(workload_omni_large_game_experience)))

        print()


        print("omni workload low teleop experience median = " + str(np.median(workload_omni_low_teleop_experience)))
        print("omni workload low teleop experience 25 = " + str(np.percentile(workload_omni_low_teleop_experience, 25)))
        print("omni workload low teleop experience 75 = " + str(np.percentile(workload_omni_low_teleop_experience, 75)))
        print("sample size = " + str(len(workload_omni_low_teleop_experience)))

        print("omni workload high teleop experience median = " + str(np.median(workload_omni_large_teleop_experience)))        
        print("omni workload high teleop experience 25 = " + str(np.percentile(workload_omni_large_teleop_experience, 25)))        
        print("omni workload high teleop experience 75 = " + str(np.percentile(workload_omni_large_teleop_experience, 75)))        
        print("sample size = " + str(len(workload_omni_large_teleop_experience)))


        print()



        print("keyboard workload low teleop experience median = " + str(np.median(workload_keyboard_low_teleop_experience)))
        print("keyboard workload low teleop experience 25 = " + str(np.percentile(workload_keyboard_low_teleop_experience, 25)))
        print("keyboard workload low teleop experience 75 = " + str(np.percentile(workload_keyboard_low_teleop_experience, 75)))
        print("sample size = " + str(len(workload_keyboard_low_teleop_experience)))

        print("keyboard workload high teleop experience median = " + str(np.median(workload_keyboard_large_teleop_experience)))        
        print("keyboard workload high teleop experience 25 = " + str(np.percentile(workload_keyboard_large_teleop_experience, 25)))        
        print("keyboard workload high teleop experience 75 = " + str(np.percentile(workload_keyboard_large_teleop_experience, 75)))        
        print("sample size = " + str(len(workload_keyboard_large_teleop_experience)))

        
        # refinement time per method
        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        # workload per method
        workload_online_keyboard_large_game_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_online_keyboard_low_game_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_online_keyboard_large_teleop_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'keyboard') & (self.df['teleop_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_online_keyboard_low_teleop_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'keyboard') & (self.df['teleop_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['workload']

        workload_offline_keyboard_large_game_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_offline_keyboard_low_game_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_offline_keyboard_large_teleop_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'keyboard') & (self.df['teleop_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_offline_keyboard_low_teleop_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'keyboard') & (self.df['teleop_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['workload']


        workload_online_omni_large_game_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['keyboard_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_online_omni_low_game_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['keyboard_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_online_omni_large_teleop_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['teleop_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_online_omni_low_teleop_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['teleop_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['workload']

        workload_offline_omni_large_game_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'omni') & (self.df['keyboard_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_offline_omni_low_game_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'omni') & (self.df['keyboard_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_offline_omni_large_teleop_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'omni') & (self.df['teleop_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['workload']
        workload_offline_omni_low_teleop_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'omni') & (self.df['teleop_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['workload']

        print("WORKLOAD PER METHOD")
        print("Online + Keyboard")
        print('-----------------')
        print("Low game experience")
        print("Median = " + str(np.median(workload_online_keyboard_low_game_experience)))
        print("25 = " + str(np.percentile(workload_online_keyboard_low_game_experience, 25)))
        print("75 = " + str(np.percentile(workload_online_keyboard_low_game_experience, 75)))
        print("N = " + str(len(workload_online_keyboard_low_game_experience)))
        print()

        print("High game experience")
        print("Median = " + str(np.median(workload_online_keyboard_large_game_experience)))
        print("25 = " + str(np.percentile(workload_online_keyboard_large_game_experience, 25)))
        print("75 = " + str(np.percentile(workload_online_keyboard_large_game_experience, 75)))
        print("N = " + str(len(workload_online_keyboard_large_game_experience)))
        print()


        print("Low teleop experience")
        print("Median = " + str(np.median(workload_online_keyboard_low_teleop_experience)))
        print("25 = " + str(np.percentile(workload_online_keyboard_low_teleop_experience, 25)))
        print("75 = " + str(np.percentile(workload_online_keyboard_low_teleop_experience, 75)))
        print("N = " + str(len(workload_online_keyboard_low_teleop_experience)))
        print()

        print("High teleop experience")
        print("Median = " + str(np.median(workload_online_keyboard_large_teleop_experience)))
        print("25 = " + str(np.percentile(workload_online_keyboard_large_teleop_experience, 25)))
        print("75 = " + str(np.percentile(workload_online_keyboard_large_teleop_experience, 75)))
        print("N = " + str(len(workload_online_keyboard_large_teleop_experience)))
        print()

        
        print()
        print("Offline + Keyboard")
        print('-----------------')
        print("Low game experience")
        print("Median = " + str(np.median(workload_offline_keyboard_low_game_experience)))
        print("25 = " + str(np.percentile(workload_offline_keyboard_low_game_experience, 25)))
        print("75 = " + str(np.percentile(workload_offline_keyboard_low_game_experience, 75)))
        print("N = " + str(len(workload_offline_keyboard_low_game_experience)))
        print()

        print("High game experience")
        print("Median = " + str(np.median(workload_offline_keyboard_large_game_experience)))
        print("25 = " + str(np.percentile(workload_offline_keyboard_large_game_experience, 25)))
        print("75 = " + str(np.percentile(workload_offline_keyboard_large_game_experience, 75)))
        print("N = " + str(len(workload_offline_keyboard_large_game_experience)))
        print()

        print("Low teleop experience")
        print("Median = " + str(np.median(workload_offline_keyboard_low_teleop_experience)))
        print("25 = " + str(np.percentile(workload_offline_keyboard_low_teleop_experience, 25)))
        print("75 = " + str(np.percentile(workload_offline_keyboard_low_teleop_experience, 75)))
        print("N = " + str(len(workload_offline_keyboard_low_teleop_experience)))
        print()

        print("High teleop experience")
        print("Median = " + str(np.median(workload_offline_keyboard_large_teleop_experience)))
        print("25 = " + str(np.percentile(workload_offline_keyboard_large_teleop_experience, 25)))
        print("75 = " + str(np.percentile(workload_offline_keyboard_large_teleop_experience, 75)))
        print("N = " + str(len(workload_offline_keyboard_large_teleop_experience)))
        print()


        print()
        print("Online + Omni")
        print('-----------------')
        print("Low game experience")
        print("Median = " + str(np.median(workload_online_omni_low_game_experience)))
        print("25 = " + str(np.percentile(workload_online_omni_low_game_experience, 25)))
        print("75 = " + str(np.percentile(workload_online_omni_low_game_experience, 75)))
        print("N = " + str(len(workload_online_omni_low_game_experience)))
        print()

        print("High game experience")
        print("Median = " + str(np.median(workload_online_omni_large_game_experience)))
        print("25 = " + str(np.percentile(workload_online_omni_large_game_experience, 25)))
        print("75 = " + str(np.percentile(workload_online_omni_large_game_experience, 75)))
        print("N = " + str(len(workload_online_omni_large_game_experience)))
        print()

        print("Low teleop experience")
        print("Median = " + str(np.median(workload_online_omni_low_teleop_experience)))
        print("25 = " + str(np.percentile(workload_online_omni_low_teleop_experience, 25)))
        print("75 = " + str(np.percentile(workload_online_omni_low_teleop_experience, 75)))
        print("N = " + str(len(workload_online_omni_low_teleop_experience)))
        print()

        
        print("High teleop experience")
        print("Median = " + str(np.median(workload_online_omni_large_teleop_experience)))
        print("25 = " + str(np.percentile(workload_online_omni_large_teleop_experience, 25)))
        print("75 = " + str(np.percentile(workload_online_omni_large_teleop_experience, 75)))
        print("N = " + str(len(workload_online_omni_large_teleop_experience)))
        print()


        print()
        print("Offline + Omni")
        print('-----------------')
        print("Low game experience")
        print("Median = " + str(np.median(workload_offline_omni_low_game_experience)))
        print("25 = " + str(np.percentile(workload_offline_omni_low_game_experience, 25)))
        print("75 = " + str(np.percentile(workload_offline_omni_low_game_experience, 75)))
        print("N = " + str(len(workload_offline_omni_low_game_experience)))
        print()

        
        print("High game experience")
        print("Median = " + str(np.median(workload_offline_omni_large_game_experience)))
        print("25 = " + str(np.percentile(workload_offline_omni_large_game_experience, 25)))
        print("75 = " + str(np.percentile(workload_offline_omni_large_game_experience, 75)))
        print("N = " + str(len(workload_offline_omni_large_game_experience)))
        print()


        print("Low teleop experience")
        print("Median = " + str(np.median(workload_offline_omni_low_teleop_experience)))
        print("25 = " + str(np.percentile(workload_offline_omni_low_teleop_experience, 25)))
        print("75 = " + str(np.percentile(workload_offline_omni_low_teleop_experience, 75)))
        print("N = " + str(len(workload_offline_omni_low_teleop_experience)))
        print()

        
        print("High teleop experience")
        print("Median = " + str(np.median(workload_offline_omni_large_teleop_experience)))
        print("25 = " + str(np.percentile(workload_offline_omni_large_teleop_experience, 25)))
        print("75 = " + str(np.percentile(workload_offline_omni_large_teleop_experience, 75)))
        print("N = " + str(len(workload_offline_omni_large_teleop_experience)))
        print()

        self.useValidParticipants()

        refinement_time_online_keyboard_large_game_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_online_keyboard_low_game_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_online_keyboard_large_teleop_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'keyboard') & (self.df['teleop_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_online_keyboard_low_teleop_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'keyboard') & (self.df['teleop_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']

        refinement_time_offline_keyboard_large_game_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_offline_keyboard_low_game_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'keyboard') & (self.df['keyboard_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_offline_keyboard_large_teleop_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'keyboard') & (self.df['teleop_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_offline_keyboard_low_teleop_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'keyboard') & (self.df['teleop_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']


        refinement_time_online_omni_large_game_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['keyboard_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_online_omni_low_game_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['keyboard_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_online_omni_large_teleop_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['teleop_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_online_omni_low_teleop_experience = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['teleop_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']

        refinement_time_offline_omni_large_game_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'omni') & (self.df['keyboard_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_offline_omni_low_game_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'omni') & (self.df['keyboard_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_offline_omni_large_teleop_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'omni') & (self.df['teleop_experience'] > 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_offline_omni_low_teleop_experience = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'omni') & (self.df['teleop_experience'] <= 3)].sort_values(by = ['model', 'mechanism'])['refinement_time']

        print("REFINEMENT TIME PER METHOD")
        print("Online + Keyboard")
        print('-----------------')
        print("Low game experience")
        print("Median = " + str(np.median(refinement_time_online_keyboard_low_game_experience)))
        print("25 = " + str(np.percentile(refinement_time_online_keyboard_low_game_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_online_keyboard_low_game_experience, 75)))
        print("N = " + str(len(refinement_time_online_keyboard_low_game_experience)))
        print()

        print("High game experience")
        print("Median = " + str(np.median(refinement_time_online_keyboard_large_game_experience)))
        print("25 = " + str(np.percentile(refinement_time_online_keyboard_large_game_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_online_keyboard_large_game_experience, 75)))
        print("N = " + str(len(refinement_time_online_keyboard_large_game_experience)))
        print()
        
        print("Low teleop experience")
        print("Median = " + str(np.median(refinement_time_online_keyboard_low_teleop_experience)))
        print("25 = " + str(np.percentile(refinement_time_online_keyboard_low_teleop_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_online_keyboard_low_teleop_experience, 75)))
        print("N = " + str(len(refinement_time_online_keyboard_low_teleop_experience)))
        print()
        
        print("High teleop experience")
        print("Median = " + str(np.median(refinement_time_online_keyboard_large_teleop_experience)))
        print("25 = " + str(np.percentile(refinement_time_online_keyboard_large_teleop_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_online_keyboard_large_teleop_experience, 75)))
        print("N = " + str(len(refinement_time_online_keyboard_large_teleop_experience)))
        print()
        
        print()
        print("Offline + Keyboard")
        print('-----------------')
        print("Low game experience")
        print("Median = " + str(np.median(refinement_time_offline_keyboard_low_game_experience)))
        print("25 = " + str(np.percentile(refinement_time_offline_keyboard_low_game_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_offline_keyboard_low_game_experience, 75)))
        print("N = " + str(len(refinement_time_offline_keyboard_low_game_experience)))
        print()
        
        print("High game experience")
        print("Median = " + str(np.median(refinement_time_offline_keyboard_large_game_experience)))
        print("25 = " + str(np.percentile(refinement_time_offline_keyboard_large_game_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_offline_keyboard_large_game_experience, 75)))
        print("N = " + str(len(refinement_time_offline_keyboard_large_game_experience)))
        print()

        print("Low teleop experience")
        print("Median = " + str(np.median(refinement_time_offline_keyboard_low_teleop_experience)))
        print("25 = " + str(np.percentile(refinement_time_offline_keyboard_low_teleop_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_offline_keyboard_low_teleop_experience, 75)))
        print("N = " + str(len(refinement_time_offline_keyboard_low_teleop_experience)))
        print()
        
        print("High teleop experience")
        print("Median = " + str(np.median(refinement_time_offline_keyboard_large_teleop_experience)))
        print("25 = " + str(np.percentile(refinement_time_offline_keyboard_large_teleop_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_offline_keyboard_large_teleop_experience, 75)))
        print("N = " + str(len(refinement_time_offline_keyboard_large_teleop_experience)))
        print()

        print()
        print("Online + Omni")
        print('-----------------')
        print("Low game experience")
        print("Median = " + str(np.median(refinement_time_online_omni_low_game_experience)))
        print("25 = " + str(np.percentile(refinement_time_online_omni_low_game_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_online_omni_low_game_experience, 75)))
        print("N = " + str(len(refinement_time_online_omni_low_game_experience)))
        print()
        
        print("High game experience")
        print("Median = " + str(np.median(refinement_time_online_omni_large_game_experience)))
        print("25 = " + str(np.percentile(refinement_time_online_omni_large_game_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_online_omni_large_game_experience, 75)))
        print("N = " + str(len(refinement_time_online_omni_large_game_experience)))
        print()
        

        print("Low teleop experience")
        print("Median = " + str(np.median(refinement_time_online_omni_low_teleop_experience)))
        print("25 = " + str(np.percentile(refinement_time_online_omni_low_teleop_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_online_omni_low_teleop_experience, 75)))
        print("N = " + str(len(refinement_time_online_omni_low_teleop_experience)))
        print()        

        
        print("High teleop experience")
        print("Median = " + str(np.median(refinement_time_online_omni_large_teleop_experience)))
        print("25 = " + str(np.percentile(refinement_time_online_omni_large_teleop_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_online_omni_large_teleop_experience, 75)))
        print("N = " + str(len(refinement_time_online_omni_large_teleop_experience)))
        print()        


        print()
        print("Offline + Omni")
        print('-----------------')
        print("Low game experience")
        print("Median = " + str(np.median(refinement_time_offline_omni_low_game_experience)))
        print("25 = " + str(np.percentile(refinement_time_offline_omni_low_game_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_offline_omni_low_game_experience, 75)))
        print("N = " + str(len(refinement_time_offline_omni_low_game_experience)))
        
        print("High game experience")
        print("Median = " + str(np.median(refinement_time_offline_omni_large_game_experience)))
        print("25 = " + str(np.percentile(refinement_time_offline_omni_large_game_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_offline_omni_large_game_experience, 75)))
        print("N = " + str(len(refinement_time_offline_omni_large_game_experience)))

        print("Low teleop experience")
        print("Median = " + str(np.median(refinement_time_offline_omni_low_teleop_experience)))
        print("25 = " + str(np.percentile(refinement_time_offline_omni_low_teleop_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_offline_omni_low_teleop_experience, 75)))
        print("N = " + str(len(refinement_time_offline_omni_low_teleop_experience)))
        
        print("High teleop experience")
        print("Median = " + str(np.median(refinement_time_offline_omni_large_teleop_experience)))
        print("25 = " + str(np.percentile(refinement_time_offline_omni_large_teleop_experience, 25)))
        print("75 = " + str(np.percentile(refinement_time_offline_omni_large_teleop_experience, 75)))
        print("N = " + str(len(refinement_time_offline_omni_large_teleop_experience)))

        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

    def plotBackgroundInfoPerMethod(self):
        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        fig = plt.figure()
        plt.suptitle("Teleoperation experience")
        plt.subplot(1,2,2)
        L = len(self.df.loc[(self.df['model'] == '1') & (self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['high_teleop_experience'] == False)])
        H = len(self.df.loc[(self.df['model'] == '1') & (self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['high_teleop_experience'] == True)])

        legend_labels = ["L (N={})".format(L), "H (N={})".format(H)]
        g = sns.boxplot(data=self.df.loc[self.df['model'] == '1'], x='method', y='workload', hue='high_teleop_experience')
        plt.ylabel("Workload [0-100]")
        plt.ylim([0, 100])

        plt.xlabel("")
        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)
        g.set_xticklabels(g.get_xticklabels(), rotation=30)

        self.useValidParticipants()

        plt.subplot(1,2,1)
        L = len(self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['high_teleop_experience'] == False)])
        H = len(self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['high_teleop_experience'] == True)])

        legend_labels = ["L (N={})".format(L), "H (N={})".format(H)]
        g = sns.boxplot(data=self.df, x='method', y='refinement_time', hue='high_teleop_experience')
        plt.xlabel("")
        plt.ylabel("Refinement time [s]")
        g.set_xticklabels(g.get_xticklabels(), rotation=30)
        plt.tight_layout()
        plt.subplots_adjust(top=0.88)
        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)
        g.set_xticklabels(g.get_xticklabels(), rotation=30)
        plt.savefig('teleop_experience_methods.pdf')

        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        fig = plt.figure()
        plt.suptitle("Gaming experience")
        plt.subplot(1,2,2)
        L = len(self.df.loc[(self.df['model'] == '1') & (self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['high_keyboard_experience'] == False)])
        H = len(self.df.loc[(self.df['model'] == '1') & (self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['high_keyboard_experience'] == True)])

        legend_labels = ["L (N={})".format(L), "H (N={})".format(H)]        
        g = sns.boxplot(data=self.df.loc[self.df['model'] == '1'], x='method', y='workload', hue='high_keyboard_experience')
        plt.ylabel("Workload [0-100]")
        plt.ylim([0, 100])

        plt.xlabel("")
        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)
        g.set_xticklabels(g.get_xticklabels(), rotation=30)

        self.useValidParticipants()

        plt.subplot(1,2,1)
        g = sns.boxplot(data=self.df, x='method', y='refinement_time', hue='high_keyboard_experience')
        plt.xlabel("")

        L = len(self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['high_keyboard_experience'] == False)])
        H = len(self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['high_keyboard_experience'] == True)])

        legend_labels = ["L (N={})".format(L), "H (N={})".format(H)]
        g.set_xticklabels(g.get_xticklabels(), rotation=30)

        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)

        plt.tight_layout()
        plt.subplots_adjust(top=0.88)
        plt.savefig('gaming_experience_methods.pdf')

        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        fig = plt.figure()
        plt.suptitle("Field of work/study")
        plt.subplot(1,2,2)
        
        NT = len(self.df.loc[(self.df['model'] == '1') & (self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['technical'] == False)])
        T = len(self.df.loc[(self.df['model'] == '1') & (self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['technical'] == True)])

        legend_labels = ["NT (N={})".format(NT), "T (N={})".format(T)]

        g = sns.boxplot(data=(self.df.loc[self.df['model'] == '1']), x='method', y='workload', hue='technical')
        g.set_xticklabels(g.get_xticklabels(), rotation=30)

        plt.ylabel("Workload [0-100]")
        plt.ylim([0, 100])

        plt.xlabel("")

        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)

        self.useValidParticipants()

        plt.subplot(1,2,1)
        g = sns.boxplot(data=self.df, x='method', y='refinement_time', hue='technical')
        plt.xlabel("")
        plt.ylabel("Refinement time [s]")
        g.set_xticklabels(g.get_xticklabels(), rotation=30)

        NT = len(self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['technical'] == False)])
        T = len(self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['technical'] == True)])
        legend_labels = ["NT (N={})".format(NT), "T (N={})".format(T)]

        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)
        g.set_xticklabels(g.get_xticklabels(), rotation=30)

        plt.tight_layout()
        plt.subplots_adjust(top=0.88)
        plt.savefig('field_of_work_methods.pdf')

        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        fig = plt.figure()
        plt.suptitle("Gender")
        plt.subplot(1,2,2)
        F = len(self.df.loc[(self.df['model'] == '1') & (self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['gender'] == False)])
        M = len(self.df.loc[(self.df['model'] == '1') & (self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['gender'] == True)])
      
        legend_labels = ["F (N={})".format(F), "M (N={})".format(M)]
        g = sns.boxplot(data=self.df.loc[self.df['model'] == '1'], x='method', y='workload', hue='gender')
        g.set_xticklabels(g.get_xticklabels(), rotation=30)

        plt.ylabel("Workload [0-100]")
        plt.ylim([0, 100])
        plt.xlabel("")

        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)
        
        self.useValidParticipants()

        plt.subplot(1,2,1)
        g = sns.boxplot(data=self.df, x='method', y='refinement_time', hue='gender')
        plt.xlabel("")
        plt.ylabel("Refinement time [s]")
        F = len(self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['gender'] == False)])
        M = len(self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['gender'] == True)])
        legend_labels = ["F (N={})".format(F), "M (N={})".format(M)]

        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)
        g.set_xticklabels(g.get_xticklabels(), rotation=30)

        plt.tight_layout()
        plt.subplots_adjust(top=0.88)
        plt.savefig('gender_methods.pdf')

        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        fig = plt.figure()
        R = len(self.df.loc[(self.df['model'] == '1') & (self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['right_handed'] == True)])
        L = len(self.df.loc[(self.df['model'] == '1') & (self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['right_handed'] == False)])
        
        legend_labels = ["R (N={})".format(R), "L (N={})".format(L)]

        plt.suptitle("Handedness")
        plt.subplot(1,2,2)
        g = sns.boxplot(data=self.df.loc[self.df['model'] == '1'], x='method', y='workload', hue='right_handed')
        g.set_xticklabels(g.get_xticklabels(), rotation=30)

        plt.ylabel("Workload [0-100]")
        plt.ylim([0, 100])
        plt.xlabel("")

        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)
        self.useValidParticipants()

        plt.subplot(1,2,1)
        g = sns.boxplot(data=self.df, x='method', y='refinement_time', hue='right_handed')
        plt.xlabel("")
        plt.ylabel("Refinement time [s]")
        R = len(self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['right_handed'] == True)])
        L = len(self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['right_handed'] == False)])

        legend_labels = ["R (N={})".format(R), "L (N={})".format(L)]

        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)
        g.set_xticklabels(g.get_xticklabels(), rotation=30)

        plt.tight_layout()
        plt.subplots_adjust(top=0.88)
        plt.savefig('handedness_methods.pdf')

        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        fig = plt.figure()
        O = len(self.df.loc[(self.df['model'] == '1') & (self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['old'] == True)])
        Y = len(self.df.loc[(self.df['model'] == '1') & (self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['old'] == False)])

        legend_labels = ["Y (N={})".format(Y), "O (N={})".format(O)]


        plt.suptitle("Age")
        plt.subplot(1,2,2)
        g = sns.boxplot(data=self.df.loc[self.df['model'] == '1'], x='method', y='workload', hue='old')
        g.set_xticklabels(g.get_xticklabels(), rotation=30)

        plt.ylabel("Workload [0-100]")
        plt.ylim([0, 100])
        plt.xlabel("")

        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)
        self.useValidParticipants()

        plt.subplot(1,2,1)
        g = sns.boxplot(data=self.df, x='method', y='refinement_time', hue='old')
        plt.xlabel("")
        plt.ylabel("Refinement time [s]")
        O = len(self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['old'] == True)])
        Y = len(self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['old'] == False)])

        legend_labels = ["Y (N={})".format(Y), "O (N={})".format(O)]

        g.legend_.set_title("")
        for t, l in zip(g.legend_.texts, legend_labels): t.set_text(l)
        g.set_xticklabels(g.get_xticklabels(), rotation=30)

        plt.tight_layout()
        plt.subplots_adjust(top=0.88)
        plt.savefig('age_methods.pdf')

    def plotNumberOfUpdates(self):
        
        self.useValidParticipants()

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
        # plt.suptitle("Number of updates needed to adapt the model")
        plt.subplot(2,2,1)
        plt.xlabel("Updates [-]", fontsize=20)
        plt.ylabel("Occurence [-]", fontsize=20)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)

        plt.title("OnOm", fontsize=20)
        number_of_updates_online_omni = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni')]['number_of_updates']
        print("online+omni median number of updates = " + str(np.median(list(number_of_updates_online_omni))))
        print("online+omni 25 percentile number of updates = " + str(np.percentile(list(number_of_updates_online_omni), 25)))
        print("online+omni 75 percentile number of updates = " + str(np.percentile(list(number_of_updates_online_omni), 75)))
        

        data = number_of_updates_online_omni
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,2,2)
        plt.xlabel("Updates [-]", fontsize=20)
        plt.ylabel("Occurence [-]", fontsize=20)
        plt.title("OnKey", fontsize=20)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)


        number_of_updates_online_keyboard = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'keyboard')]['number_of_updates']
        print("online+keyboard median number of updates = " + str(np.median(list(number_of_updates_online_keyboard))))
        print("online+keyboard 25 percentile number of updates = " + str(np.percentile(list(number_of_updates_online_keyboard), 25)))
        print("online+keyboard 75 percentile number of updates = " + str(np.percentile(list(number_of_updates_online_keyboard), 75)))
        
        data = number_of_updates_online_keyboard
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,2,3)
        plt.ylabel("Occurence", fontsize=20)
        plt.xlabel("Updates [-]", fontsize=20)
        plt.title("OffOm", fontsize=20)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)

        number_of_updates_offline_omni = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'omni')]['number_of_updates']
        print("offline+omni median number of updates = " + str(np.median(list(number_of_updates_offline_omni))))
        print("offline+omni 25 percentile number of updates = " + str(np.percentile(list(number_of_updates_offline_omni), 25)))
        print("offline+omni 75 percentile number of updates = " + str(np.percentile(list(number_of_updates_offline_omni), 75)))
        
        data = number_of_updates_offline_omni
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,2,4)
        plt.ylabel("Occurence [-]", fontsize=20)
        plt.xlabel("Updates [-]", fontsize=20)
        plt.title("OffKey", fontsize=20)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)

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

        self.useValidParticipants()

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
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)

        data = number_of_refinements_omni
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,1,2)
        plt.title("Keyboard")
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)

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
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)

        data = number_of_refinements_online
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,1,2)
        plt.title("Offline")
        plt.xlabel("Amount [-]")
        plt.ylabel("Occurence [-]")
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)

        plt.ylim([0, 30])
        data = number_of_refinements_offline
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.tight_layout()
        fig.subplots_adjust(top=0.88)

        plt.savefig('number_of_refinements_mechanism.pdf')
    
        fig = plt.figure()
        # plt.suptitle("Number of refinements needed to adapt the model")
        plt.subplot(2,2,1)
        plt.xlabel("Refinements [-]", fontsize=20)
        plt.ylabel("Occurence [-]", fontsize=20)
        plt.title("OnOm", fontsize=20)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)

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
        plt.xlabel("Refinements [-]", fontsize=20)
        plt.ylabel("Occurence [-]", fontsize=20)
        plt.title("OnKey", fontsize=20)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)

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
        plt.xlabel("Refinements [-]", fontsize=20)
        plt.ylabel("Occurence [-]", fontsize=20)
        plt.title("OffOm", fontsize=20)
        plt.ylim([0, 20])
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)

        number_of_refinements_offline_omni = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'omni')]['number_of_refinements']
        print("offline+omni median number of refinements = " + str(np.median(list(number_of_refinements_offline_omni))))
        print("offline+omni 25 percentile number of refinements = " + str(np.percentile(list(number_of_refinements_offline_omni), 25)))
        print("offline+omni 75 percentile number of refinements = " + str(np.percentile(list(number_of_refinements_offline_omni), 75)))
        
        data = number_of_refinements_offline_omni
        x = range(min(data), max(data) + 1)
        plt.xticks(x)
        plt.hist(x=data, bins=np.arange(min(data) - binwidth/2, max(data) + binwidth, binwidth))

        plt.subplot(2,2,4)
        plt.xlabel("Refinements [-]", fontsize=20)
        plt.ylabel("Occurence [-]", fontsize=20)
        plt.title("OffKey", fontsize=20)
        plt.ylim([0, 20])
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)

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
        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
        
        workload_omni = list(self.df.loc[self.df['interface'] == 'omni'].sort_values(by = ['model', 'mechanism'])['workload'])
        workload_keyboard = list(self.df.loc[self.df['interface'] == 'keyboard'].sort_values(by = ['model', 'mechanism'])['workload'])

        # workload_omni = list(self.df.loc[(self.df['interface'] == 'omni') & (self.df['technical'] == True)].sort_values(by = ['model', 'mechanism'])['workload'])
        # workload_keyboard = list(self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['technical'] == True)].sort_values(by = ['model', 'mechanism'])['workload'])

        # t, p = ttest_rel(workload_keyboard, workload_omni, nan_policy='omit')
        t, p = wilcoxon(workload_keyboard, workload_omni, alternative='greater')
        
        self.statistics_values['workload']['interface']['p'] = p
        self.statistics_values['workload']['interface']['t'] = t

        workload_online = list(self.df.loc[self.df['mechanism'] == 'online'].sort_values(by = ['model', 'interface'])['workload'])
        workload_offline = list(self.df.loc[self.df['mechanism'] == 'offline'].sort_values(by = ['model', 'interface'])['workload'])
        
        # t, p = ttest_rel(workload_online, workload_offline, nan_policy='omit')                
        t, p = wilcoxon(workload_online, workload_offline, alternative='less')

        self.statistics_values['workload']['mechanism']['p'] = p
        self.statistics_values['workload']['mechanism']['t'] = t
        
        self.useValidParticipants()

        refinement_time_omni = self.df.loc[self.df['interface'] == 'omni'].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_keyboard = self.df.loc[self.df['interface'] == 'keyboard'].sort_values(by = ['model', 'mechanism'])['refinement_time']

        # t, p = ttest_rel(refinement_time_keyboard, refinement_time_omni, nan_policy='omit')
        t, p = wilcoxon(refinement_time_omni, refinement_time_keyboard, alternative='less')

        self.statistics_values['refinement_time']['interface']['p'] = p
        self.statistics_values['refinement_time']['interface']['t'] = t

        refinement_time_online = list(self.df.loc[self.df['mechanism'] == 'online'].sort_values(by = ['model', 'interface'])['refinement_time'])
        refinement_time_offline = list(self.df.loc[self.df['mechanism'] == 'offline'].sort_values(by = ['model', 'interface'])['refinement_time'])

        # t, p = ttest_rel(refinement_time_online, refinement_time_offline, nan_policy='omit')
        t, p = wilcoxon(refinement_time_online, refinement_time_offline, alternative='less')
        
        self.statistics_values['refinement_time']['mechanism']['p'] = p
        self.statistics_values['refinement_time']['mechanism']['t'] = t

        
        number_of_refinements_online = self.df.loc[self.df['mechanism'] == 'online']['number_of_refinements']
        number_of_refinements_offline = self.df.loc[self.df['mechanism'] == 'offline']['number_of_refinements']
        number_of_refinements_keyboard = self.df.loc[self.df['interface'] == 'keyboard']['number_of_refinements']
        number_of_refinements_omni = self.df.loc[self.df['interface'] == 'omni']['number_of_refinements']

        number_of_updates_online = self.df.loc[self.df['mechanism'] == 'online']['number_of_updates']
        number_of_updates_offline = self.df.loc[self.df['mechanism'] == 'offline']['number_of_updates']
        number_of_updates_keyboard = self.df.loc[self.df['interface'] == 'keyboard']['number_of_updates']
        number_of_updates_omni = self.df.loc[self.df['interface'] == 'omni']['number_of_updates']
        
        # p_values_refinements_updates = []
        t, p = wilcoxon(number_of_refinements_online, number_of_refinements_offline, alternative='less')
        # p_values_refinements_updates.append([[p, t]])
        print(p)
        t, p = wilcoxon(number_of_refinements_keyboard, number_of_refinements_omni, alternative='less')
        # p_values_refinements_updates.append([[p, t]])
        print(p)

        t, p = wilcoxon(number_of_updates_online, number_of_updates_offline, alternative='less')
        # p_values_refinements_updates.append([[p, t]])
        print(p)

        t, p = wilcoxon(number_of_updates_keyboard, number_of_updates_omni, alternative='less')
        # p_values_refinements_updates.append([[p, t]])
        print(p)

        # print(p_values_refinements_updates)


        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
        
        workload_high_game_experience = list(self.df.loc[self.df['high_keyboard_experience'] == True].sort_values(by = ['model', 'interface'])['refinement_time'])
        workload_low_game_experience = list(self.df.loc[self.df['high_keyboard_experience'] == False].sort_values(by = ['model', 'interface'])['refinement_time'])

        # print(len(workload_low_game_experience))
        # print(len(workload_high_game_experience))
        t, p = wilcoxon(workload_high_game_experience, workload_low_game_experience, alternative='less')
        print(p)
        self.useValidParticipants()

        refinement_time_high_game_experience = list(self.df.loc[self.df['high_keyboard_experience'] == True].sort_values(by = ['model', 'interface'])['refinement_time'])
        refinement_time_low_game_experience = list(self.df.loc[self.df['high_keyboard_experience'] == False].sort_values(by = ['model', 'interface'])['refinement_time'])

        t, p = wilcoxon(refinement_time_high_game_experience, refinement_time_low_game_experience, alternative='less')
        print(p)

        workload_high_game_experience_keyboard = list(self.df.loc[(self.df['high_keyboard_experience'] == True) & (self.df['interface'] == 'keyboard')].sort_values(by = ['model', 'interface'])['workload'])
        workload_low_game_experience_keyboard = list(self.df.loc[(self.df['high_keyboard_experience'] == False) & (self.df['interface'] == 'keyboard')].sort_values(by = ['model', 'interface'])['workload'])
        t, p = wilcoxon(workload_high_game_experience_keyboard, workload_low_game_experience_keyboard, alternative='less')
        print(p)

        # refinement_time_high_game_experience_keyboard = list(self.df.loc[(self.df['high_keyboard_experience'] == True) & (self.df['interface'] == 'keyboard')].sort_values(by = ['model', 'interface'])['refinement_time'])
        # refinement_time_low_game_experience_keyboard = list(self.df.loc[(self.df['high_keyboard_experience'] == False) & (self.df['interface'] == 'keyboard')].sort_values(by = ['model', 'interface'])['refinement_time'])
        # t, p = wilcoxon(refinement_time_high_game_experience_keyboard, refinement_time_low_game_experience_keyboard, alternative='less')
        # print(p)

        # refinement_time_high_teleop_experience_omni = list(self.df.loc[(self.df['high_teleop_experience'] == True) & (self.df['interface'] == 'omni')].sort_values(by = ['model', 'interface'])['refinement_time'])
        # refinement_time_low_teleop_experience_omni = list(self.df.loc[(self.df['high_teleop_experience'] == False) & (self.df['interface'] == 'omni')].sort_values(by = ['model', 'interface'])['refinement_time'])
        # t, p = wilcoxon(refinement_time_high_teleop_experience_omni, refinement_time_low_teleop_experience_omni, alternative='less')
        # print(p)
        
        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        workload_online_omni = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'online')].sort_values(by = ['model', 'mechanism'])['workload']
        workload_offline_omni = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'offline')].sort_values(by = ['model', 'mechanism'])['workload']
        workload_online_keyboard = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'online')].sort_values(by = ['model', 'mechanism'])['workload']
        workload_offline_keyboard = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'offline')].sort_values(by = ['model', 'mechanism'])['workload']

        self.useValidParticipants()
        refinement_time_online_omni = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'online')].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_offline_omni = self.df.loc[(self.df['interface'] == 'omni') & (self.df['mechanism'] == 'offline')].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_online_keyboard = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'online')].sort_values(by = ['model', 'mechanism'])['refinement_time']
        refinement_time_offline_keyboard = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['mechanism'] == 'offline')].sort_values(by = ['model', 'mechanism'])['refinement_time']

        print()
        print("P values per method")
        t, p = wilcoxon(refinement_time_offline_omni, refinement_time_offline_keyboard, alternative='less')
        print(p)

        t, p = wilcoxon(workload_offline_omni, workload_offline_keyboard, alternative='less')
        print(p)

        t, p = wilcoxon(refinement_time_online_omni, refinement_time_offline_omni, alternative='less')
        print(p)

        t, p = wilcoxon(refinement_time_online_omni, refinement_time_online_keyboard, alternative='less')
        print(p)

        t, p = wilcoxon(refinement_time_online_omni, refinement_time_offline_keyboard, alternative='less')
        print(p)
        


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

        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
        
        workload_omni = self.df.loc[(self.df['interface'] == 'omni') & (self.df['model'] == '1')]['workload']
        workload_keyboard = self.df.loc[(self.df['interface'] == 'keyboard') & (self.df['model'] == '1')]['workload']

        workload_online = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['model'] == '1')]['workload']
        workload_offline = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['model'] == '1')]['workload']

        self.useValidParticipants()

        refinement_time_omni = self.df.loc[self.df['interface'] == 'omni']['refinement_time']
        refinement_time_keyboard = self.df.loc[self.df['interface'] == 'keyboard']['refinement_time']
        
        refinement_time_online = self.df.loc[self.df['mechanism'] == 'online']['refinement_time']
        refinement_time_offline = self.df.loc[self.df['mechanism'] == 'offline']['refinement_time']

        plt.figure()

        ax = plt.gca()

        # plt.suptitle("Refinement time")

        plt.subplot(2,2,1)
        plt.title("Omni", fontsize=30)
        refinement_time_omni.hist(grid=False)
        plt.xticks(fontsize=20, rotation=30)
        plt.yticks(fontsize=20)

        plt.subplot(2,2,2)
        plt.title("Keyboard", fontsize=30)
        refinement_time_keyboard.hist(grid=False)
        plt.xticks(fontsize=20, rotation=30)
        plt.yticks(fontsize=20)
        
        plt.subplot(2,2,3)
        plt.title("Online", fontsize=30)
        refinement_time_online.hist(grid=False)
        plt.xticks(fontsize=20, rotation=30)
        plt.yticks(fontsize=20)
        
        plt.subplot(2,2,4)
        plt.title("Offline", fontsize=30)
        refinement_time_offline.hist(grid=False)

        plt.xticks(fontsize=20, rotation=30)
        plt.yticks(fontsize=20)
        plt.tight_layout(h_pad=2)

        plt.savefig('distributions_refinement_time.pdf')

        plt.figure()
        # plt.suptitle("Workload")
        
        plt.subplot(2,2,1)
        plt.title("Omni", fontsize=30)
        workload_omni.hist(grid=False)
        plt.xticks(fontsize=20, rotation=30)
        plt.yticks(fontsize=20)
        
        plt.subplot(2,2,2)
        plt.title("Keyboard", fontsize=30)
        workload_keyboard.hist(grid=False)
        plt.xticks(fontsize=20, rotation=30)
        plt.yticks(fontsize=20)

        plt.subplot(2,2,3)
        plt.title("Online", fontsize=30)
        workload_online.hist(grid=False)
        plt.xticks(fontsize=20, rotation=30)
        plt.yticks(fontsize=20)

        plt.subplot(2,2,4)
        plt.title("Offline", fontsize=30)
        workload_offline.hist(grid=False)
        plt.xticks(fontsize=20, rotation=30)
        plt.yticks(fontsize=20)

        plt.tight_layout(h_pad=2)
        plt.savefig('distributions_workload.pdf')
    
    def plotAndSaveRefinementTimeAndWorkload(self):
        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
        self.calculateAndStoreTtestValues()

        p_dummy = 0.999

        plt.figure()

        plt.subplot(2,2,3)
        # plt.title("Workload")
        ax = sns.boxplot(x="mechanism", y="workload", hue="interface", data=self.df)
        plt.ylabel("Workload [0-100]")
        plt.xlabel("p = " + '%.6f' % self.statistics_values['workload']['mechanism']['p'])
        # plt.xlabel("p = " + '%.4f' % 0.0103)

        ax.legend_.set_title("")
        ax.legend_.remove()
        
        plt.subplot(2,2,4)
        # plt.title("Workload")
        ax = sns.boxplot(x="interface", y="workload", hue="mechanism", data=self.df)
        plt.ylabel("Workload [0-100]")
        plt.xlabel("p = " + '%.3f' % self.statistics_values['workload']['interface']['p'])
        # plt.xlabel("p = " + str(p_dummy))
        ax.legend_.set_title("")
        ax.legend_.remove()

        self.useValidParticipants()
        self.calculateAndStoreTtestValues()

        plt.subplot(2,2,1)
        # plt.title("Refinement time")
        ax = sns.boxplot(x="mechanism", y="refinement_time", hue="interface", data=self.df)
        plt.ylabel("Refinement time [s]")
        plt.xlabel("p = " + '{:0.2e}'.format(self.statistics_values["refinement_time"]["mechanism"]["p"]))
        # plt.xlabel("p = " + str(p_dummy))
        ax.legend_.set_title("")
        # ax.legend_.remove()
        # plt.savefig('refinement_time_mechanism.png')

        plt.subplot(2,2,2)

        # plt.title("Refinement time")
        ax = sns.boxplot(x="interface", y="refinement_time", hue="mechanism", data=self.df)
        plt.ylabel("Refinement time [s]")
        plt.xlabel("p = " + '%.3f' % self.statistics_values['refinement_time']['interface']['p'])
        # plt.xlabel("p = " + str(0.490))

        ax.legend_.set_title("")

        plt.tight_layout()
        plt.subplots_adjust(top=0.88)
        plt.savefig('p_values.pdf')

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
        sns.boxplot(data=self.df, x='method', y='refinement_time', orient='v')

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

        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        ######### better boxplots #############
        fig = plt.figure()
        # plt.suptitle("Refinement time and workload per method")
        plt.subplot(1,2,2)
        ax = sns.boxplot(data=self.df, x='method', y='workload', orient='v')
        ax.set_xticklabels(ax.get_xticklabels(), rotation=30)

        plt.xlabel("")
        plt.ylabel("Workload [0-100]")

        self.useValidParticipants()

        plt.subplot(1,2,1)
        ax = sns.boxplot(data=self.df, x='method', y='refinement_time', orient='v')
        ax.set_xticklabels(ax.get_xticklabels(), rotation=30)
        plt.xlabel("")
        plt.ylabel("Refinement time [s]")

        plt.tight_layout()
        plt.subplots_adjust(top=0.88)
        plt.savefig('workload_refinement_time_methods.pdf')

    def plotAmountOfAdaptedModelsPerMethod(self):
        adapted_models_online_omni = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'omni') & (self.df['is_adapted'] == True)]
        adapted_models_online_keyboard = self.df.loc[(self.df['mechanism'] == 'online') & (self.df['interface'] == 'keyboard') & (self.df['is_adapted'] == True)]
        adapted_models_offline_omni = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'omni') & (self.df['is_adapted'] == True)]
        adapted_models_offline_keyboard = self.df.loc[(self.df['mechanism'] == 'offline') & (self.df['interface'] == 'keyboard') & (self.df['is_adapted'] == True)]

        plt.figure()
        # plt.title("Adapted models")
        plt.bar(['OnOm', 'OnKey', 'OffOm', 'OffKey'], [len(adapted_models_online_omni), len(adapted_models_online_keyboard), len(adapted_models_offline_omni), len(adapted_models_offline_keyboard)])
        plt.xticks(fontsize=20, rotation=30)
        plt.yticks(fontsize=20)
        plt.tight_layout()
        plt.savefig('adapted_models.pdf')

    def plotRefinementTimePerModel(self):
        self.rows_list = []
        self.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
        self.useValidParticipants()

        ax = sns.boxplot(data=self.df, x='method', y='refinement_time', hue='model')
        ax.set_xticklabels(ax.get_xticklabels(), fontsize=20, rotation=30)
        plt.xlabel("")
        plt.ylabel("Refinement time [s]", fontsize=20)
        plt.yticks(fontsize=20)
        plt.legend(title="model", title_fontsize=20, fontsize=20)
        plt.tight_layout()
 
        plt.savefig('learning_effect.pdf')

if __name__ == "__main__":
    data_analysis = DataAnalysis()
    data_analysis.loadMultipleParticipantsData([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

    # data_analysis.plotTrainingTime()
    # data_analysis.plotBackgroundInfo()
    # data_analysis.plotTeleopGameExperience()
    # data_analysis.printFieldOfStudy()

    # print("Workload values are only valid (no nans in refinement time dropped)")
    # print("Also checking for normality and amount of successfully adapted models is valid here")
    # data_analysis.useValidParticipants()
    # data_analysis.calculateAndStoreTtestValues()
    # data_analysis.getTopScore()
    # data_analysis.printStatisticValues()
    
    # data_analysis.plotAndSaveRefinementTimeAndWorkload()
    # data_analysis.plotDistributions()
    # data_analysis.plotAmountOfAdaptedModelsPerMethod()
    
    # print("Refinement time values are the only thing that's valid")
    # data_analysis.useValidParticipants()
    # data_analysis.calculateAndStoreTtestValues()
    # data_analysis.printStatisticValues()
    
    # data_analysis.plotAndSaveRefinementTimeAndWorkload()
    # data_analysis.plotDistributions()
    # data_analysis.plotAmountOfAdaptedModelsPerMethod()
    
    # plt.show()
    # data_analysis.plotNumberOfUpdates()
    # data_analysis.plotNumberOfRefinements()
    # data_analysis.plotMethodOpinions()


    # data_analysis.calculateStatisticsValuesAndPlotBackgroundInfo()
    # data_analysis.plotBackgroundInfoPerMethod()
    # data_analysis.plotTechnicalNonTechnical()
    # data_analysis.plotAndSaveRefinementTimeAndWorkload()
    # data_analysis.calculateStatisticsValuesAndPlotTeleopGameExperience()
    data_analysis.plotRefinementTimePerModel()