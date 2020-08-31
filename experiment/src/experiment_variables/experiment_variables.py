import random

class ExperimentVariables(object):
    def __init__(self):
        self.method_mapping_str_to_number = {'online+omni':1, 'offline+omni':2, 'online+pendant':3, 'offline+pendant':4}
        self.method_mapping_number_to_str = {1:'online+omni', 2:'offline+omni', 3:'online+pendant', 4: 'offline+pendant'}
        self.T_desired = 20
        self.num_object_positions = 2
        self.num_trials = 3
        self.num_methods = 4
        self.y_position_step_dict = {1: 0.0, 2: 2*0.1}

        self.y0 = 0.2
        self.object_positions = { 1: [0.8, self.y0 - self.y_position_step_dict[1], 0.7], 2: [0.8, self.y0 - self.y_position_step_dict[2], 0.7]}
        self.max_refinements = 5
        self.num_updates = 1
        self.desired_datapoints = 75
        
if __name__ == "__main__":
    experiment_variabes = ExperimentVariables()
