
class TextUpdater(object):
    def __init__(self):
        self.text_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/'
        self.text_file = self.text_path + 'text.txt'
    
    def update(self, text):
        with open(self.text_file, 'w') as f:
            f.write(text)
    
    def empty(self):
        with open(self.text_file, 'w') as f:
                f.write("")
    
    def append(self, text):
        with open(self.text_file, 'a') as f:
            f.write(" " + str(text))