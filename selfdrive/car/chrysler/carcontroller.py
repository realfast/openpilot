from cereal import car
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.chrysler.chryslercan import create_lkas_hud, create_lkas_command, create_wheel_buttons, create_speed_spoof
from selfdrive.car.chrysler.values import CAR, CarControllerParams, STEER_MAX_LOOKUP, STEER_DELTA_UP, STEER_DELTA_DOWN
from common.conversions import Conversions as CV

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.frame = 0
    self.prev_lkas_frame = -1
    self.hud_count = 0
    self.car_fingerprint = CP.carFingerprint
    self.steer_rate_limited = False
    self.lkasdisabled = 0
    self.spoofspeed = 0
    self.lkasactivevalue = 0
    self.lkascativevalueprev = 0
    self.button_counter_prev = -1
    CarControllerParams.STEER_MAX = STEER_MAX_LOOKUP.get(CP.carFingerprint, 1.)
    CarControllerParams.STEER_DELTA_UP = STEER_DELTA_UP.get(CP.carFingerprint, 1.) 
    CarControllerParams.STEER_DELTA_DOWN = STEER_DELTA_DOWN.get(CP.carFingerprint, 1.) 

    self.packer = CANPacker(dbc_name)

  def update(self, CC, CS, low_speed_alert):
    # this seems needed to avoid steering faults and to force the sync with the EPS counter

    actuators = CC.actuators

    #moving_fast = CS.out.vEgo > self.CP.minSteerSpeed  # for status message

    lkas_active = CC.enabled and not low_speed_alert

    can_sends = []

    #*** control msgs ***

    # LKAS_HEARTBIT is forwarded by Panda so no need to send it here.
    # frame is 50Hz (0.02s period) #Becuase we skip every other frame
    if self.frame % 12 == 0:  # 0.25s period
      if CS.lkas_car_model != -1:
        can_sends.append(create_lkas_hud(self.packer, lkas_active, CC.hudControl.visualAlert, self.hud_count, CS, self.car_fingerprint, 0))
        can_sends.append(create_lkas_hud(self.packer, lkas_active, CC.hudControl.visualAlert, self.hud_count, CS, self.car_fingerprint, 1))
        self.hud_count += 1

    if CS.button_counter != self.button_counter_prev:
      if CC.cruiseControl.cancel:
        can_sends.append(create_wheel_buttons(self.packer, CS.button_counter + 1, 2, cancel=True, acc_resume = False))
      elif CS.out.cruiseState.standstill:
        can_sends.append(create_wheel_buttons(self.packer, CS.button_counter + 1, 0, cancel=False, acc_resume = True))
        can_sends.append(create_wheel_buttons(self.packer, CS.button_counter + 1, 2, cancel=False, acc_resume = True))
      
    if self.prev_lkas_frame != CS.lkas_counter:

      # steer torque
      new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
      apply_steer = apply_toyota_steer_torque_limits(new_steer, self.apply_steer_last,
                                                    CS.out.steeringTorqueEps, CarControllerParams)
      self.steer_rate_limited = new_steer != apply_steer

      if lkas_active and self.spoofspeed >= 36 * CV.MPH_TO_KPH:
        self.lkasactivevalue = 2
        if self.lkasactivevalue == self.lkascativevalueprev: #Allow lkas to wait atleast 1 frame after enabling bit before sending first steer
          apply_steer = apply_steer
        else:
          apply_steer = 0
      else:
        apply_steer = 0
        self.lkasactivevalue = 0

      if lkas_active and CS.out.vEgoRaw * CV.MS_TO_MPH < 36: #if lkas is active and below threshold spoof speed 
        self.spoofspeed = 36 * CV.MPH_TO_KPH #mph
      else:
        self.spoofspeed = CS.out.vEgoRaw * CV.MS_TO_KPH
    
      self.apply_steer_last = apply_steer
      self.lkascativevalueprev = self.lkasactivevalue
        
      can_sends.append(create_speed_spoof(self.packer, CS.esp8_counter, self.spoofspeed))
      can_sends.append(create_lkas_command(self.packer, int(apply_steer), self.lkasactivevalue, CS.lkas_counter,1))
      can_sends.append(create_lkas_command(self.packer, int(apply_steer), self.lkasactivevalue, CS.lkas_counter,0))

    self.frame += 1
    self.prev_lkas_frame = CS.lkas_counter
    self.button_counter_prev = CS.button_counter
    
    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last  / CarControllerParams.STEER_MAX

    return new_actuators, can_sends
