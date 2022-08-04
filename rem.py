class REM:

  def __init__(self):
    self.moment = 0
    self.moment_gradient = 0
    self.max_moment = 0

  def set_moment(self, moment):
    self.moment = moment

  def set_moment_gradient(self, gradient):
    self.moment_gradient = gradient

  def get_moment(self):
    return self.moment

  def get_moment_gradient(self):
    return self.moment_gradient
