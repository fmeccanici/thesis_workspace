
class TextUpdater(object):
    def __init__(self, text_path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/', text_file='text.txt'):
        self.text_path = text_path
        self.text_file = text_path + text_file
    
    def update(self, text):
        with open(self.text_file, 'w') as f:
            f.write(text)
    
    def empty(self):
        with open(self.text_file, 'w') as f:
                f.write("")
    
    def append(self, text):
        with open(self.text_file, 'a') as f:
            f.write(" " + str(text))