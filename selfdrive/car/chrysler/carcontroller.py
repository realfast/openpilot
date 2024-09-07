from opendbc.can.packer import CANPacker
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_meas_steer_torque_limits
from openpilot.selfdrive.car.chrysler import chryslercan
from openpilot.selfdrive.car.chrysler.values import RAM_CARS, STEER_TO_ZERO, CarControllerParams, ChryslerFlags
from openpilot.selfdrive.car.interfaces import CarControllerBase
from common.conversions import Conversions as CV  # Import Conversions
from common.op_params import opParams, STEER_DELTA_UP, STEER_DELTA_DOWN, STEER_ERROR_MAX


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM, OP = None):
    if OP is None:
      OP = opParams()
    self.op_params = OP

    self.CP = CP
    self.apply_steer_last = 0
    self.frame = 0

    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.lkas_control_bit_prev = False
    self.last_button_frame = 0
    self.spoof_speed = 0
    self.actual_min_speed = 18 * CV.MS_TO_KPH
    self.spoof_speed_increment = 0.1
    self.spoof_speed_threshold = 15 * CV.MPH_TO_KPH

    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams(CP)

  def increment_spoof_speed(self):
    if self.spoof_speed < self.spoof_speed_threshold:
      self.spoof_speed += self.spoof_speed_increment
    else:
      self.spoof_speed = self.actual_min_speed

  def update(self, CC, CS, now_nanos):
    can_sends = []

    self.params.STEER_DELTA_UP = self.op_params.get(STEER_DELTA_UP)
    self.params.STEER_DELTA_DOWN = self.op_params.get(STEER_DELTA_DOWN)
    self.params.STEER_ERROR_MAX = self.op_params.get(STEER_ERROR_MAX)

    lkas_active = CC.latActive and self.lkas_control_bit_prev

    # cruise buttons
    if (self.frame - self.last_button_frame)*DT_CTRL > 0.05:
      das_bus = 2 if self.CP.carFingerprint in RAM_CARS else 0

      # ACC cancellation
      if CC.cruiseControl.cancel:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, cancel=True))

      # ACC resume from standstill
      elif CC.cruiseControl.resume:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, resume=True))

    # HUD alerts
    if self.frame % 25 == 0:
      if CS.lkas_car_model != -1:
        can_sends.extend(chryslercan.create_lkas_hud(self.packer, self.CP, lkas_active, CC.hudControl.visualAlert,
                                                     self.hud_count, CS.lkas_car_model, CS.auto_high_beam))
        self.hud_count += 1

    # steering
    if self.frame % self.params.STEER_STEP == 0:

      # TODO: can we make this more sane? why is it different for all the cars?
      lkas_control_bit = self.lkas_control_bit_prev
      if CS.out.vEgo > self.CP.minSteerSpeed:
        lkas_control_bit = True
      elif self.CP.flags & ChryslerFlags.HIGHER_MIN_STEERING_SPEED:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 3.0):
          lkas_control_bit = False
      elif self.CP.carFingerprint in RAM_CARS:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 0.5):
          lkas_control_bit = False

      if self.CP.carFingerprint in STEER_TO_ZERO:        
        if lkas_control_bit and self.spoof_speed >= self.actual_min_speed:
          lkas_control_bit = True
        else:
          lkas_control_bit = False
      
      # EPS faults if LKAS re-enables too quickly
      lkas_control_bit = lkas_control_bit and (self.frame - self.last_lkas_falling_edge > 200)

      if not lkas_control_bit and self.lkas_control_bit_prev:
        self.last_lkas_falling_edge = self.frame
      self.lkas_control_bit_prev = lkas_control_bit

      # steer torque
      new_steer = int(round(CC.actuators.steer * self.params.STEER_MAX))
      apply_steer = apply_meas_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, self.params)
      if not lkas_active or not lkas_control_bit:
        apply_steer = 0
      self.apply_steer_last = apply_steer

      can_sends.extend(chryslercan.create_lkas_command(self.packer, self.CP, int(apply_steer), lkas_control_bit, self.frame/self.params.STEER_STEP))

    if self.CP.carFingerprint in STEER_TO_ZERO and self.frame % 2 == 0:
      if CC.enabled and CS.out.vEgoRaw * CV.MS_TO_KPH < self.actual_min_speed:
        self.increment_spoof_speed()
      else:
        self.spoof_speed = CS.out.vEgoRaw * CV.MS_TO_KPH
      can_sends.append(chryslercan.create_speed_spoof(self.packer, self.spoof_speed))

    self.frame += 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    return new_actuators, can_sends