from cereal import car
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.chrysler.chryslercan import create_lkas_hud, create_lkas_command, create_wheel_buttons
from selfdrive.car.chrysler.values import CAR, CarControllerParams, STEER_MAX_LOOKUP, STEER_DELTA_UP, STEER_DELTA_DOWN
from common.op_params import opParams, STOCK_DELTA_UP_DOWN, STOCK_STEER_MAX #STOCK_DELTA_DOWN, STOCK_DELTA_UP, 


class CarController:
  def __init__(self, dbc_name, CP, VM, OP=None):
    self.CP = CP
    self.apply_steer_last = 0
    self.frame = 0
    self.prev_lkas_frame = -1
    self.hud_count = 0
    self.car_fingerprint = CP.carFingerprint
    self.gone_fast_yet = False
    self.steer_rate_limited = False

    if not OP:
      OP = opParams()
    self.op_params = OP

    self.packer = CANPacker(dbc_name)

  def update(self, CC, CS):
    CarControllerParams.STEER_DELTA_UP = self.op_params.get(STOCK_DELTA_UP_DOWN)
    CarControllerParams.STEER_DELTA_DOWN = self.op_params.get(STOCK_DELTA_UP_DOWN)
    CarControllerParams.STEER_MAX= self.op_params.get(STOCK_STEER_MAX)
    # this seems needed to avoid steering faults and to force the sync with the EPS counter
    if self.prev_lkas_frame == CS.lkas_counter:
      return car.CarControl.Actuators.new_message(), []

    actuators = CC.actuators

    # steer torque
    new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.apply_steer_last,
                                                   CS.out.steeringTorqueEps, CarControllerParams)
    self.steer_rate_limited = new_steer != apply_steer

    moving_fast = CS.out.vEgo > self.CP.minSteerSpeed  # for status message

    if self.car_fingerprint not in (CAR.RAM_1500, CAR.RAM_2500):
      if CS.out.vEgo > (self.CP.minSteerSpeed - 0.5):  # for command high bit
        self.gone_fast_yet = True
      elif self.car_fingerprint in (CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019):
        if CS.out.vEgo < (self.CP.minSteerSpeed - 3.0):
          self.gone_fast_yet = False  # < 14.5m/s stock turns off this bit, but fine down to 13.5
    
    elif self.car_fingerprint in (CAR.RAM_1500, CAR.RAM_2500):
      if CS.out.vEgo > (self.CP.minSteerSpeed):  # for command high bit
        self.gone_fast_yet = True
      if CS.out.vEgo < (self.CP.minSteerSpeed - 1):
          self.gone_fast_yet = False  

    lkas_active = moving_fast and CC.enabled

    if not lkas_active:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    can_sends = []

    # *** control msgs ***

    if CC.cruiseControl.cancel:
      can_sends.append(create_wheel_buttons(self.packer, CS.button_counter + 1, self.car_fingerprint, cancel=True))

    # LKAS_HEARTBIT is forwarded by Panda so no need to send it here.
    # frame is 100Hz (0.01s period)
    if self.frame % 25 == 0:  # 0.25s period
      if CS.lkas_car_model != -1:
        can_sends.append(create_lkas_hud(self.packer, lkas_active,
                                         CC.hudControl.visualAlert, self.hud_count, CS, self.car_fingerprint))
        self.hud_count += 1

    can_sends.append(create_lkas_command(self.packer, int(apply_steer), self.gone_fast_yet, CS.lkas_counter))

    self.frame += 1
    self.prev_lkas_frame = CS.lkas_counter

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / CarControllerParams.STEER_MAX

    return new_actuators, can_sends