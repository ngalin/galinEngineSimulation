import sys
old_stdout = sys.stdout


class Logger:

  def __init__(self, filename):
    self.log_file = open(filename, "w")
    sys.stdout = self.log_file

  @staticmethod
  def log(data):
    print(data)

  def write(self):
    self.log_file.close()
