import numpy as np
from math import pi
import helpers as h
import rem


class Shaft:
  accel = 0
  speed = 0
  loc = 0

  delta_loc = 0
  delta_speed = 0
  steel_density = 7800  # [kg/m^3]

  def __init__(self, vane_angle, vane_volume, attach_rem, rem_j=0):  # sets the vane geometry
    self.angularWidthOfVane = h.deg2rad(vane_angle)
    self.ssv = vane_volume  # [L], volume of adjacent chambers
    if attach_rem:
      self.rem = rem.REM()
      self.set_moment_of_inertia(self.moment_of_inertia() + rem_j)
      self.rem_attached = True
    else:
      self.set_moment_of_inertia(self.moment_of_inertia()+ rem_j)
      self.rem_attached = False

  def has_rem(self):
    return self.rem_attached

  def shaft_radius(self):  # based on assumption that width of vane is twice the shaft radius:
    return np.power(self.ssv / (pi - 2 * self.angularWidthOfVane), 1 / 3) / 2  # [m],

  def vane_radius(self):
    return 3 * self.shaft_radius()  # [m], assumed relationship

  def ssa(self):  # [rad], # summed angular width of two neighbouring chambers
    return pi - 2 * self.angularWidthOfVane

  def angle2volume(self, angle):
    return 8 * np.power(self.shaft_radius(), 3) * angle

  def vane_area(self):
    return 4 * self.shaft_radius() ** 2

  def lever_arm(self):
    return 2 * self.shaft_radius()

  def moment_of_inertia(self):
    jk = pi * self.shaft_radius() * self.steel_density * (
        self.vane_radius() ** 4 - self.shaft_radius() ** 4)
    return 2 * (jk * 2 * self.angularWidthOfVane) / pi

  def get_moment_of_inertia(self):
    return self.J

  def set_moment_of_inertia(self, value):
    self.J =  value

  def print_log(self):
    print("accel [rad/s^2]: %f" % self.accel)
    print("speed [rad/s]: %f" % self.speed)
    print("loc [degrees]: %f" % (self.loc * 180 / pi))

  def apply_torque(self, direction, torque, time):
    self.accel = direction * torque / self.J
    self.delta_speed = self.accel * time
    ave_speed = self.speed + self.delta_speed / 2.
    self.delta_loc = ave_speed * time
    self.loc += self.delta_loc
    self.speed += self.delta_speed

  def init(self):
    self.accel = 0
    self.delta_speed = 0
    self.delta_loc = 0
    self.loc = 0
    self.speed = 0

  def get_rem_moment(self, stroke_time=None, time_passed=0):

    if stroke_time is None:
      if self.has_rem():
        return self.rem.get_moment()
      else:
        return 0
    else:
      if self.has_rem():
        if time_passed < (stroke_time / 2):
          self.rem.max_moment = self.rem.get_moment_gradient() * time_passed
          return self.rem.get_moment_gradient() * time_passed
        else:
          return self.rem.get_moment_gradient() * (stroke_time) - self.rem.get_moment_gradient() * time_passed
      else:
        return 0
