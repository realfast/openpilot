from cereal import car
from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.chrysler.chryslercan import create_lkas_hud, create_lkas_command, create_cruise_buttons
from selfdrive.car.chrysler.values import CAR, CarControllerParams
from opendbc.can.packer import CANPacker

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.frame = 0
    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.lkas_control_bit_prev = False
    self.last_button_frame = 0
    self.car_fingerprint = CP.carFingerprint
    self.steer_rate_limited = False
    self.lkaslast_frame = 0.
    self.gone_fast_yet_previous = False

    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams(CP)

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, hud_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart):

    can_sends = []
    # this seems needed to avoid steering faults and to force the sync with the EPS counter

    # *** compute control surfaces ***

    #moving_fast = CS.out.vEgo > CS.CP.minSteerSpeed  # for status message
    #lkas_active = moving_fast and enabled
    lkas_control_bit = self.lkas_control_bit_prev

    if self.car_fingerprint not in (CAR.RAM_1500):
      if CS.out.vEgo > (CS.CP.minSteerSpeed - 0.5):  # for command high bit
        lkas_control_bit = True
      elif self.car_fingerprint in (CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019):
        if CS.out.vEgo < (CS.CP.minSteerSpeed - 3.0):
          lkas_control_bit = False  # < 14.5m/s stock turns off this bit, but fine down to 13.5
          
    elif self.car_fingerprint in (CAR.RAM_1500):
      if CS.out.vEgo > (CS.CP.minSteerSpeed):  # for command high bit
        lkas_control_bit = True
      elif CS.CP.minSteerSpeed == 0.5:
        lkas_control_bit = True
      elif CS.out.vEgo < (CS.CP.minSteerSpeed - 0.5):
        lkas_control_bit = False  

    if self.gone_fast_yet_previous == True and lkas_control_bit == False:
        self.lkaslast_frame = self.frame


    if (CS.out.steerFaultPermanent is True) or (self.frame-self.lkaslast_frame<400):#If the LKAS Control bit is toggled too fast it can create and LKAS error
      lkas_control_bit = False

    lkas_active = lkas_control_bit and enabled

    
    #*** control msgs ***
    if self.frame % 2 == 0:
      if pcm_cancel_cmd: 
       # can_sends.append(create_cruise_buttons(self.packer, CS.button_counter + 1, 0, cancel=True, resume = False))
        can_sends.append(create_cruise_buttons(self.packer, CS.button_counter + 1, 2, cancel=True, resume = False))
      elif CS.out.cruiseState.standstill:
       # can_sends.append(create_cruise_buttons(self.packer, CS.button_counter + 1, 0, cancel=False, resume = True))
        can_sends.append(create_cruise_buttons(self.packer, CS.button_counter + 1, 2, cancel=False, resume = True))

    # LKAS_HEARTBIT is forwarded by Panda so no need to send it here.
    # frame is 100Hz (0.01s period)
    if self.frame % 25 == 0:  # 0.25s period
      if (CS.lkas_car_model != -1):
        can_sends.append(create_lkas_hud(self.packer, self.car_fingerprint, lkas_active, hud_alert, self.hud_count, CS.lkas_car_model, CS.auto_high_beam))
        self.hud_count += 1
    # steering
    if self.frame % 2 == 0:
      # steer torque
      new_steer = int(round(actuators.steer * self.params.STEER_MAX))
      apply_steer = apply_toyota_steer_torque_limits(new_steer, self.apply_steer_last,
                                                    CS.out.steeringTorqueEps, self.params)
      self.steer_rate_limited = new_steer != apply_steer
      if not lkas_active:
        apply_steer = 0
      self.apply_steer_last = apply_steer
      self.gone_fast_yet_previous = lkas_control_bit
      idx = self.frame // 2
      can_sends.append(create_lkas_command(self.packer, self.car_fingerprint, int(apply_steer), lkas_control_bit, idx))

    self.frame += 1
    if not lkas_control_bit and self.lkas_control_bit_prev:
      self.last_lkas_falling_edge = self.frame
    self.lkas_control_bit_prev = lkas_control_bit


    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last  / self.params.STEER_MAX

    return new_actuators, can_sends
