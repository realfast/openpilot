from opendbc.can.packer import CANPacker
from common.realtime import DT_CTRL
from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.chrysler.chryslercan import create_lkas_hud, create_lkas_command, create_cruise_buttons, das_3_message, acc_log, das_4_message, das_5_message, create_chime_message
from selfdrive.car.chrysler.values import CAR, RAM_CARS, RAM_DT, RAM_HD, CarControllerParams
from cereal import car

from common.numpy_fast import clip
from common.conversions import Conversions as CV
from common.params import Params, put_nonblocking
from cereal import car
import math

from common.op_params import opParams

LongCtrlState = car.CarControl.Actuators.LongControlState
# braking
BRAKE_CHANGE = 0.06

GearShifter = car.CarState.GearShifter

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.frame = 0

    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.lkas_control_bit_prev = False
    self.last_button_frame = 0

    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams(CP)
    self.accel_sent = False

    # long
    self.max_gear = None
    self.op_params = opParams()
    self.desired_velocity = 0
    self.calc_velocity = 0
    self.speed = 0
    self.long_active = False
    self.last_acc = False

  def update(self, CC, CS):
    can_sends = []

    lkas_active = CC.latActive and not CS.lkasdisabled

    # cruise buttons
    if (CS.button_counter != self.last_button_frame):
      das_bus = 2 if self.CP.carFingerprint in RAM_CARS else 0
      self.last_button_frame = CS.button_counter
      if self.CP.carFingerprint in RAM_CARS:
        if self.CP.carFingerprint in RAM_HD and CS.out.cruiseState.standstill and CS.button_counter % 14 == 0 and self.accel_sent == False:
          self.accel_sent = True
          can_sends.append(create_cruise_buttons(self.packer, CS.button_counter, das_bus, CS.cruise_buttons, accel = True))
        elif self.CP.carFingerprint in RAM_HD and CS.out.cruiseState.standstill and self.accel_sent == True:
          self.accel_sent = False
          can_sends.append(create_cruise_buttons(self.packer, CS.button_counter, das_bus, CS.cruise_buttons, decel = True))
        # elif CS.cruise_cancel:
        #   can_sends.append(create_cruise_buttons(self.packer, CS.button_counter, das_bus, CS.cruise_buttons, cancel=True))
        else:
          self.accel_sent = False
          can_sends.append(create_cruise_buttons(self.packer, CS.button_counter, das_bus, CS.cruise_buttons, cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

       # ACC cancellation
      elif CC.cruiseControl.cancel:
        can_sends.append(create_cruise_buttons(self.packer, CS.button_counter+1, das_bus, CS.cruise_buttons, cancel=True))

      # ACC resume from standstill
      elif CC.cruiseControl.resume:
        can_sends.append(create_cruise_buttons(self.packer, CS.button_counter+1, das_bus, CS.cruise_buttons, resume=True))

    if self.frame % 2 == 0:

    #LONG
      das_3_counter = self.frame / 2
      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      starting = CC.actuators.longControlState == LongCtrlState.starting
      self.accel = clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      self.long_active = CC.enabled
      self.speed = CC.hudControl.setSpeed

      brake_threshold = -self.op_params.get('brake_threshold') if CS.out.vEgo > 2.25 else 0
      time_for_sample = self.op_params.get('long_time_constant')
      torque_limits = self.op_params.get('torque_limits')
      drivetrain_efficiency = self.op_params.get('drivetrain_efficiency')
          

      accel_go = False
      decel_req = False
      accel_req = False
      standstill = False
      decel = 4
      torque = 0
      max_gear = 8
        
      if self.last_acc != CC.enabled:
        self.long_active = True

      elif CC.enabled:
        if CC.actuators.accel < brake_threshold:
          decel_req = True
          # if CC.actuators.speed < 0.1 and CS.out.vEgo < 0.1:
          if stopping and CS.out.vEgo < 0.01:
            standstill = True
          decel = CC.actuators.accel

        else:
          accel_req = True
          # if CS.out.vEgo < 0.1 and CC.actuators.accel > 0:
          if starting:
            accel_go = True

          self.calc_velocity = ((self.accel-CS.out.aEgo) * time_for_sample) + CS.out.vEgo
          if self.op_params.get('comma_speed'):
            self.desired_velocity = min(CC.actuators.speed, self.speed)
          else:
            self.desired_velocity = min(self.calc_velocity, self.speed)

          # kinetic energy (J) = 1/2 * mass (kg) * velocity (m/s)^2
          # use the kinetic energy from the desired velocity - the kinetic energy from the current velocity to get the change in velocity
          kinetic_energy = ((self.CP.mass * self.desired_velocity **2)/2) - ((self.CP.mass * CS.out.vEgo**2)/2)
          # convert kinetic energy to torque
          # torque(NM) = (kinetic energy (J) * 9.55414 (Nm/J) * time(s))/RPM
          torque = (kinetic_energy * 9.55414 * time_for_sample)/(drivetrain_efficiency * CS.engineRpm + 0.001)
          if not CS.tcLocked and CS.tcSlipPct > 0:
            torque = torque/CS.tcSlipPct
          torque = clip(torque, -torque_limits, torque_limits) # clip torque to -6 to 6 Nm for sanity

        if CS.engineTorque < 0 and torque > 0:
          #If the engine is producing negative torque, we need to return to a reasonable torque value quickly.
          # rough estimate of external forces in N
          # total_forces = 650
          # #torque required to maintain speed
          # torque = (total_forces * CS.out.vEgo * 9.55414)/(CS.engineRpm * drivetrain_efficiency + 0.001)
          torque = 75

        #If torque is positive, add the engine torque to the torque we calculated. This is because the engine torque is the torque the engine is producing.
        else:
          torque += CS.engineTorque

        torque = max(torque, (0 - self.op_params.get('min_torque')))
      
      self.last_acc = CC.enabled

      can_sends.append(das_3_message(self.packer, 0, das_3_counter, self.long_active,
                                    CS.out.cruiseState.available,
                                    accel_req, 
                                    decel_req,
                                    accel_go,
                                    torque,
                                    max_gear,
                                    standstill,
                                    decel))

      can_sends.append(das_5_message(self.packer, 0, self.CP, self.speed, self.frame / 2))

      can_sends.append(acc_log(self.packer, CC.actuators.accel, CC.actuators.speed, self.calc_velocity, CS.out.aEgo, CS.out.vEgo))

      if self.CP.carFingerprint not in RAM_CARS:
        can_sends.append(das_3_message(self.packer, 2, das_3_counter, self.long_active,
                                      CS.out.cruiseState.available,
                                      accel_req, 
                                      decel_req,
                                      accel_go,
                                      torque,
                                      max_gear,
                                      standstill,
                                      decel))

        can_sends.append(das_5_message(self.packer, 2, self.CP, self.speed, self.frame / 2))
      
      # steering
      lkas_control_bit = self.lkas_control_bit_prev
      # TODO: can we make this more sane? why is it different for all the cars?
      if self.CP.carFingerprint in RAM_DT:
        if CS.out.vEgo >= self.CP.minEnableSpeed and CS.out.vEgo <= self.CP.minEnableSpeed + 0.5:
          lkas_control_bit = True
        if (self.CP.minEnableSpeed >= 14.5)  and (CS.out.gearShifter != GearShifter.drive) :
          lkas_control_bit = False
      elif CS.out.vEgo > self.CP.minSteerSpeed:
        lkas_control_bit = True
      elif self.CP.carFingerprint in (CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019):
        if CS.out.vEgo < (self.CP.minSteerSpeed - 3.0):
          lkas_control_bit = False
      elif self.CP.carFingerprint in RAM_HD:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 0.5):
          lkas_control_bit = False

      # EPS faults if LKAS re-enables too quickly
      lkas_control_bit = lkas_control_bit and (self.frame - self.last_lkas_falling_edge > 200) and not CS.out.steerFaultTemporary and not CS.out.steerFaultPermanent

      if not lkas_control_bit and self.lkas_control_bit_prev:
        self.last_lkas_falling_edge = self.frame

      # steer torque
      self.params.STEER_DELTA_DOWN = self.op_params.get('steerDeltaDown')
      new_steer = int(round(CC.actuators.steer * self.params.STEER_MAX))
      apply_steer = apply_toyota_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, self.params)
      if not lkas_active or not lkas_control_bit or not self.lkas_control_bit_prev:
        apply_steer = 0
      self.apply_steer_last = apply_steer
      self.lkas_control_bit_prev = lkas_control_bit

      can_sends.append(create_lkas_command(self.packer, self.CP, int(apply_steer), lkas_control_bit))

    if self.frame % 6 == 0:
      state = 0
      if CS.out.cruiseState.available:
        state = 2 if CS.out.cruiseState.enabled else 1 #1/2 for regular cc, 3/4 for ACC
      can_sends.append(das_4_message(self.packer, 0, state, self.speed))
      if self.CP.carFingerprint not in RAM_CARS:
        can_sends.append(das_4_message(self.packer, 2, state, self.speed))

    # HUD alerts
    if self.frame % 25 == 0:
      if CS.lkas_car_model != -1:
        can_sends.append(create_lkas_hud(self.packer, self.CP, lkas_active, CC.hudControl.visualAlert, self.hud_count, CS.lkas_car_model, CS))
        self.hud_count += 1

    if self.CP.carFingerprint not in RAM_CARS:
      if self.frame % 50 == 0:
        # tester present - w/ no response (keeps radar disabled)
        can_sends.append((0x753, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 0))

      if self.frame % 100 == 0:
        can_sends.append(create_chime_message(self.packer, 0))
        can_sends.append(create_chime_message(self.packer, 2))  

    self.frame += 1

    new_actuators = CC.actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX

    return new_actuators, can_sends
