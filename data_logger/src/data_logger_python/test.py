
import ast

path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/participant_3/'
data_path = path + 'data.txt'
print(data_path)
with open(data_path, 'r') as f:
    # print(ast.literal_eval(f.read()))
    outfile = ast.literal_eval(f.read())