#!/usr/bin/env python3
from cereal import car
from panda import Panda
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.chrysler.values import CAR, DBC, RAM_CARS, RAM_HD, RAM_DT, CarControllerParams
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.disable_ecu import disable_ecu

ButtonType = car.CarState.ButtonEvent.Type
GearShifter = car.CarState.GearShifter

class CarInterface(CarInterfaceBase):

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX
    
  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None, experimental_long=False):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "chrysler"

    ret.steerActuatorDelay = 0.4
    ret.steerLimitTimer = 0.4
    stiffnessFactor = 1.0

    # safety config
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.chrysler)]
    if candidate in RAM_HD:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_CHRYSLER_RAM_HD
    elif candidate in RAM_DT:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_CHRYSLER_RAM_DT

    ret.minSteerSpeed = -0.1  # m/s
    if candidate in (CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019):
      # TODO: allow 2019 cars to steer down to 13 m/s if already engaged.
      ret.minSteerSpeed = 17.5  # m/s 17 on the way up, 13 on the way down once engaged.

    # Chrysler
    if candidate in (CAR.PACIFICA_2017_HYBRID, CAR.PACIFICA_2018, CAR.PACIFICA_2018_HYBRID, CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020):
      ret.mass = 2242. + STD_CARGO_KG
      ret.wheelbase = 3.089
      ret.steerRatio = 16.2  # Pacifica Hybrid 2017
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15, 0.30], [0.03, 0.05]]
      ret.lateralTuning.pid.kf = 0.00006
      ret.openpilotLongitudinalControl = True
      tune = ret.longitudinalTuning
      tune.deadzoneBP = [0., 9.]
      tune.deadzoneV = [.0, .15]
      tune.kpV = [0.25]
      tune.kiV = [0.05]
      ret.stopAccel = -2.0
      ret.startingState = True
      ret.vEgoStarting = 0.1
      ret.startAccel = 1.0
      ret.vEgoStopping = 0.05

    # Jeep
    elif candidate in (CAR.JEEP_CHEROKEE, CAR.JEEP_CHEROKEE_2019):
      ret.mass = 1778 + STD_CARGO_KG
      ret.wheelbase = 2.71
      ret.steerRatio = 16.7
      ret.steerActuatorDelay = 0.2
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15, 0.30], [0.03, 0.05]]
      ret.lateralTuning.pid.kf = 0.00006

    # Ram
    elif candidate == CAR.RAM_1500:
      ret.steerActuatorDelay = 0.2
      ret.wheelbase = 3.88
      ret.steerRatio = 16.3
      ret.mass = 2493. + STD_CARGO_KG
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.minSteerSpeed = 0.5
      ret.minEnableSpeed = 14.6
      if car_fw is not None:
        for fw in car_fw:
          if fw.ecu == 'eps' and fw.fwVersion in (b"68273275AF", b"68273275AG", b"68312176AE", b"68312176AG", ):
            ret.minEnableSpeed = 0.
      ret.openpilotLongitudinalControl = True
      tune = ret.longitudinalTuning
      tune.deadzoneBP = [0., 9.]
      tune.deadzoneV = [.0, .15]
      tune.kpV = [0.25]
      tune.kiV = [0.05]
      ret.stopAccel = -2.0
      ret.startingState = True
      ret.vEgoStarting = 0.1
      ret.startAccel = 1.0


    elif candidate == CAR.RAM_HD:
      stiffnessFactor = 0.35
      ret.steerActuatorDelay = 0.25
      ret.wheelbase = 3.785
      ret.steerRatio = 15.61
      ret.mass = 3405. + STD_CARGO_KG
      ret.minSteerSpeed = 16
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, 1.0, False)
      ret.openpilotLongitudinalControl = True
      tune = ret.longitudinalTuning
      tune.deadzoneBP = [0., 9.]
      tune.deadzoneV = [.0, .15]
      tune.kpV = [0.25]
      tune.kiV = [0.05]
      ret.longitudinalActuatorDelayUpperBound = 0.5 # s
      ret.stoppingDecelRate = 0.3  # reach stopping target smoothly
      ret.stopAccel = -2.0
      ret.startingState = True
      ret.vEgoStarting = 0.1
      ret.startAccel = 1.0

    else:
      raise ValueError(f"Unsupported car: {candidate}")

    ret.centerToFront = ret.wheelbase * 0.44

    # starting with reasonable value for civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, stiffnessFactor)

    ret.enableBsm = 720 in fingerprint[0]

    if ret.openpilotLongitudinalControl:
      ret.pcmCruise = False
      ret.radarOffCan = True

    return ret
  @staticmethod
  def init(CP, logcan, sendcan):
    if CP.carFingerprint not in RAM_CARS:
      disable_ecu(logcan, sendcan, bus=0, addr=0x753, com_cont_req=b'\x28\x81\x01', response_offset = -0x280)

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    # events
    events = self.create_common_events(ret, extra_gears=[car.CarState.GearShifter.low],
                                       pcm_enable=not self.CS.CP.openpilotLongitudinalControl,
                                       enable_buttons=(ButtonType.decelCruise, ButtonType.resumeCruise, ButtonType.accelCruise))

    if self.CP.carFingerprint in RAM_DT:
      if self.CS.out.vEgo >= self.CP.minEnableSpeed:
        self.low_speed_alert = False
      if (self.CP.minEnableSpeed >= 14.5)  and (self.CS.out.gearShifter != GearShifter.drive) :
        self.low_speed_alert = True

    else:# Low speed steer alert hysteresis logic
      if self.CP.minSteerSpeed > 0. and ret.vEgo < (self.CP.minSteerSpeed + 0.5):
        self.low_speed_alert = True
      elif ret.vEgo > (self.CP.minSteerSpeed + 1.):
        self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    return ret

  def apply(self, c):
    return self.CC.update(c, self.CS)