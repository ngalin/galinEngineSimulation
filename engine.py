from math import pi
import shaft
import logger as lg
import helpers as hp
import sys

# definition of stroke - length of time to full expansion
# equal speeds, or when speeds change direction
# check if intake, and exhaust chambers are always open... if not, will cause probs.

class Engine:
  iac = 1.3  # polytropic constant of contraction
  iic = iac - 1
  iax = 1.3  # polytropic constant of expansion
  iix = iax - 1

  initial_temperature = 300  # [K]
  initial_pressure = 1e5  # [Pa] atmospheric pressure
  step_ignition_temperature = 2000  # [K]

  ignition_pressure = 0
  ignition_temperature = 0
  ignition_work = 0
  compression_pressure = 0
  compression_temperature = 0
  compression_work = 0

  expansion_torque = 0
  compression_torque = 0

  stroke_time = 0
  theta = 0

  act = 0
  axt = 0

  logger = None

  # sets the vane geometry, and attaches REMs to shafts
  def __init__(self, vane_angle, vane_volume, rems, rems_j, init_compression_angle, log=None):
    self.vane_angle = vane_angle * pi / 180.
    self.shaft1 = shaft.Shaft(vane_angle, vane_volume, attach_rem=rems[0], rem_j=rems_j[0])
    self.shaft2 = shaft.Shaft(vane_angle, vane_volume, attach_rem=rems[1], rem_j=rems_j[1])
    self.bisector = shaft.Shaft(0, 0, attach_rem=0)
    self.bisector.set_moment_of_inertia(self.shaft1.get_moment_of_inertia() + self.shaft2.get_moment_of_inertia())
    self.min_angular_width = hp.deg2rad(init_compression_angle)
    self.max_angular_width = self.calc_max_angular_width()

    self.axt = self.min_angular_width
    self.act = self.max_angular_width

    self.compression_ratio = self.max_angular_width / self.min_angular_width  # 9
    self.set_rem_moment()

    self.time = 0
    self.logger = log

  def set_rem_moment(self, direction=1):
    moment = self.moment_of_rem()
    if self.shaft1.has_rem() and self.shaft2.has_rem():
      self.shaft1.rem.set_moment(-direction * moment / 2)
      self.shaft2.rem.set_moment(direction * moment / 2)
    elif self.shaft1.has_rem():
      self.shaft1.rem.set_moment(-direction * moment)
    elif self.shaft2.has_rem():
      self.shaft2.rem.set_moment(direction * moment)
    else:
      print('WTF')

  # TODO def set_thermodynamic_parameters(self, ignition_temp_increase etc
  def calc_min_angular_width(self):
    return self.shaft1.ssa() / (self.compression_ratio + 1)

  def calc_max_angular_width(self):
    return (2 * pi - 2 * self.min_angular_width - 4 * self.vane_angle) / 2.0

  def volume_exhaust_intake(self):
    return self.shaft1.angle2volume(self.max_angular_width - self.min_angular_width)

  def pressure2moment(self, pressure):
    return self.shaft1.vane_area() * self.shaft1.lever_arm() * pressure

  def moment2pressure(self, moment):
    return moment / (self.shaft1.vane_area() * self.shaft1.lever_arm())

  def max_volume(self):
    return self.shaft1.angle2volume(self.max_angular_width)

  def min_volume(self):
    return self.shaft1.angle2volume(self.min_angular_width)

  # def calculate_thermodynamic_properties(self): #TODO

  def perform_compression(self):
    self.compression_pressure = self.initial_pressure * (self.compression_ratio ** self.iac)
    self.compression_temperature = self.initial_temperature * (self.compression_ratio ** self.iic)
    self.compression_work = ((self.initial_pressure * self.max_volume()) / self.iic) * (
        1 - (self.compression_ratio ** self.iic))
    return self.compression_work

  def perform_ignition(self):  # returns work performed during expansion due to ignition
    self.ignition_temperature = self.compression_temperature + self.step_ignition_temperature
    self.ignition_pressure = self.compression_pressure * (
        self.ignition_temperature / self.compression_temperature)
    self.ignition_work = ((self.ignition_pressure * self.min_volume()) / self.iix) * (
        1 - (1 / self.compression_ratio) ** self.iix)
    return self.ignition_work

  def moment_of_rem(self):
    self.perform_compression()
    self.perform_ignition()
    return 2. * (self.perform_ignition() + self.perform_compression()) / (
        self.max_angular_width - self.min_angular_width)  # + 5

  def vmax(self):
    return self.pressure2moment(self.ignition_pressure * (self.min_angular_width ** self.iax))

  def vmac(self):
    return self.pressure2moment(self.initial_pressure * (self.max_angular_width ** self.iac))

  def get_moment_of_gases(self):
    return self.pressure2moment(self.ignition_pressure - self.compression_pressure)

  def swap_axt_act(self):
    self.axt = self.act

  def init_shafts(self, init_speed):
    self.axt = self.min_angular_width
    self.act = self.max_angular_width

    self.shaft1.init()
    self.shaft2.init()
    self.bisector.init()

    self.shaft1.loc = self.min_angular_width / 2.
    self.shaft2.loc = -self.min_angular_width / 2.

    self.bisector.loc = (self.shaft1.loc + self.shaft2.loc) / 2.
    self.shaft1.speed = init_speed
    self.shaft2.speed = init_speed
    self.bisector.speed = init_speed

  def run(self, num_strokes, init_speed, leading_vane, log=None):
    self.logger = log

    if leading_vane == 1:
      vpk = 1
    else:
      vpk = -1

    self.init_shafts(init_speed)

    for stroke in range(num_strokes):
      _ = self.execute_stroke(vpk)
      vpk *= -1
      self.swap_axt_act()
      print('stroke', stroke, file=sys.stderr)

  def execute_stroke(self, vpk):
    filename = "data/first_stroke.csv"
    log_file = open(filename, "w")
    moment_of_gases = self.get_moment_of_gases()

    # changing this value is sometimes helpful in achieving accurate stroke times and also for confirming
    # that calculated stroke time converged appropriately
    delta_t = 0.1e-6 #0.13e-7#0.2e-7

    n = 0.
    t1s = 0.

    new_delta_loc1 = 0
    new_delta_loc2 = 0

    work_shaft1 = 0
    work_shaft2 = 0

    flag_45_deg = True

    while n < 1000000:
      if n % 2000 == 0:
        new_delta_loc1 = self.shaft1.loc - new_delta_loc1
        new_delta_loc2 = self.shaft2.loc - new_delta_loc2
        self.log_to_file(new_delta_loc1, new_delta_loc2, work_shaft1, work_shaft2, direction=vpk)
        new_delta_loc1 = self.shaft1.loc
        new_delta_loc2 = self.shaft2.loc
        vars_to_file = str(t1s) + " , " + str(self.shaft1.loc) + " , " + str(self.shaft2.loc) + "\n"
        log_file.write(vars_to_file)

      n += 1

      # apply torques to shafts
      self.shaft1.apply_torque(vpk, (moment_of_gases + self.shaft1.get_rem_moment() - 0), time=delta_t)
      self.shaft2.apply_torque(vpk, (self.shaft2.get_rem_moment() - moment_of_gases + 0), time=delta_t)
      self.bisector.apply_torque(vpk, self.shaft2.get_rem_moment() + self.shaft1.get_rem_moment(), time=delta_t)

      delta_angle = vpk * (self.shaft1.delta_loc - self.shaft2.delta_loc)

      work_shaft1 += self.shaft1.delta_loc * (moment_of_gases + self.shaft1.get_rem_moment())
      work_shaft2 += self.shaft2.delta_loc * (self.shaft2.get_rem_moment() - moment_of_gases)

      self.axt += delta_angle
      self.act = self.shaft1.ssa() - self.axt

      self.expansion_torque = self.vmax() / (self.axt ** self.iax)
      self.compression_torque = self.vmac() / (self.act ** self.iac)
      moment_of_gases = self.expansion_torque - self.compression_torque

      t1s += delta_t
      self.time += delta_t
      #print out time when chamber expands to +/-45 degrees:
      if not flag_45_deg and n > 100 and hp.rad2deg(self.shaft1.loc) > 45 and hp.rad2deg(self.shaft2.loc) < -45: #print out time
        print('time when working chamber opens to 90deg: %f', t1s, file=sys.stderr)
        print('speed shaft1: %f', self.shaft1.speed)
        print('speed shaft2: %f', self.shaft2.speed)
        flag_45_deg = True

      # when delta between angular separation changes from increasing to decreasing
      # location of bisector as: 0, 90, 180, 270 degrees
      if n > 100 and abs(self.shaft1.speed - self.shaft2.speed) < 0.001:  # time end of stroke
        print('%f', t1s, file=sys.stderr)
        break

    self.stroke_time = t1s
    self.theta = self.bisector.loc

    log_file.close()
    return self.stroke_time, self.theta

  def log_to_file(self, prev_loc1, prev_loc2, work_shaft1, work_shaft2, direction=1):
    if self.logger is not None:
      self.logger.log(
        '%.10e,%.10e,%.10e,%.10e,%.10e,%.2e,%.2e,%.2e,%.2e,%.2e,%.2e,%.2e,%.2e,%.2e,%.2e,%.2e,%.2e' %
        (self.time,
         work_shaft1, work_shaft2,
         prev_loc1, prev_loc2,
         self.shaft1.loc, self.shaft2.loc,
         self.bisector.loc,
         self.shaft1.speed * 60 / (2 * pi), self.shaft2.speed * 60 / (2 * pi),
         self.bisector.speed * 60 / (2 * pi),
         direction*self.expansion_torque, direction*self.compression_torque,
         self.shaft1.accel, self.shaft2.accel,
         direction * self.shaft1.get_rem_moment(),
         direction * self.shaft2.get_rem_moment()))


def main():
  print('hello')
  filename = "data/tmp.csv"

  # currently, inputs into the system are:
  #--the angular width of vanes (in degrees): 40
  #--the volume of combustion+expansion chambers (Vmax + Vmin) (in m^3): 1e-4
  #--indication of presence of electrical machines on either shaft1 or two or both: [1,1]
  #--the compression angle (in degrees): 10
  #--the moment of inertia of electrical machines, derived from some datasheet (kg m^2): [0.0134, 0.0134]

  #--Current model assumes a square wave torque applied by the electrical machines (not possible in reality)
  #--the magnitude of the torque is calculated as the torque necessary to consume all work output of the gases:
  #-- (Work of gases) / (angular distance travelled by leading shaft - angular distance travelled by trailing shaft)
  engine = Engine(40, 10e-4, rems=[1, 1], init_compression_angle=10,
                  rems_j=[0.6234e-3 * 0, 0.6234e-3 * 0])

  print("U1 [degrees]: %f" % hp.rad2deg(engine.min_angular_width)) #--minimal volume of chamber at start of combustion
  print("U2 [degrees]: %f" % hp.rad2deg(engine.max_angular_width)) #--maximal volume of chamber at end of combustion
  print("R1 [m]: %f" % engine.shaft1.shaft_radius()) #--derived shaft radius, the assumptions built into the model are
  #--that the vane area is square, hence, the outer radius of the vanes R2 = 3*R1 (where R1 is the shaft radius) and that
  #--the depth of      the chamber d = R1. See figure 3 here:https://galin-engine.com/method-and-device-specification
  print("R2 [m]: %f" % engine.shaft1.vane_radius())
  print("J1 [kgm^2]: %f" % engine.shaft1.get_moment_of_inertia()) #calculated from the geometry of the vanes in fig3 referenced above
  print("J2 [kgm^2]: %f" % engine.shaft2.get_moment_of_inertia())

  print("Compression ratio: %f" % engine.compression_ratio)

  engine.init_shafts(0) #--initialise the location and speed of shafts

  (t1s, _) = engine.execute_stroke(vpk=1) #--execute a single combustion cycle of the engine, to determine how long it takes (t1s)
  print("stroke time [ms]: %f" % (t1s * 1e3))
  print("rem moment [Nm]: %f" % engine.shaft2.rem.get_moment())

  engine.shaft1.print_log()
  engine.shaft2.print_log()
  #--print some other relevant output parameters that are inferred from the time of one combustion stroke:
  print("axt [degrees]: %f" % hp.rad2deg(engine.axt)) #--how far the combustion chamber opened
  print("loc bisector [degrees]: %f" % hp.rad2deg(engine.theta))
  print("stroke time [ms]: %f" % (engine.stroke_time * 1e3))
  print("work done by gases, Ag [J]: %f" % (engine.perform_ignition() + engine.perform_compression()))
  print("speed w0: [rad/s] ", ((pi / 2 - engine.theta) / engine.stroke_time))
  print("speed w0: [RPM] ", ((pi / 2 - engine.theta) / engine.stroke_time) / (2 * pi) * 60)
  print("ave power [W]: ", (engine.perform_ignition() + engine.perform_compression()) / engine.stroke_time)
  # now calculating motion:
  print("Im here")
  logger = lg.Logger(filename)
  engine.run(4, ((pi / 2 - engine.theta) / engine.stroke_time), leading_vane=1, log=logger)

  if logger is not None:
    logger.write()


if __name__ == "__main__":
  main()
