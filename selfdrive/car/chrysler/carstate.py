from cereal import car
from common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.chrysler.values import DBC, STEER_THRESHOLD, RAM_CARS, CarControllerParams

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CCP = CarControllerParams(CP)
    self.button_states = {button.event_type: False for button in self.CCP.BUTTONS}
    self.CP = CP
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    self.auto_high_beam = 0
    self.button_counter = 0
    self.lkas_car_model = -1
    self.lkasdisabled = 0
    self.lkasbuttonprev = 0

    self.engineRpm = None
    self.torqMin = None
    self.torqMax = None
    self.longEnabled = False
    self.cruisespeed = 0

    if CP.carFingerprint in RAM_CARS:
      self.shifter_values = can_define.dv["Transmission_Status"]["Gear_State"]
    else:
      self.shifter_values = can_define.dv["GEAR"]["PRNDL"]

  def create_button_events(self, cp, buttons):
    button_events = []

    for button in buttons:
      state = cp.vl[button.can_addr][button.can_msg] in button.values
      if self.button_states[button.event_type] != state:
        event = car.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state

    return button_events

  def update(self, cp, cp_cam):

    ret = car.CarState.new_message()

    # lock info
    ret.doorOpen = any([cp.vl["BCM_1"]["DOOR_OPEN_FL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_FR"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["ORC_1"]["SEATBELT_DRIVER_UNLATCHED"] == 1

    # brake pedal
    ret.brake = 0
    ret.brakePressed = cp.vl["ESP_1"]['Brake_Pedal_State'] == 1  # Physical brake pedal switch

    # gas pedal
    ret.gas = cp.vl["ECM_5"]["Accelerator_Position"]
    ret.gasPressed = ret.gas > 1e-5

    # car speed
    if self.CP.carFingerprint in RAM_CARS:
      ret.vEgoRaw = cp.vl["ESP_8"]["Vehicle_Speed"] * CV.KPH_TO_MS
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["Transmission_Status"]["Gear_State"], None))
    else:
      ret.vEgoRaw = (cp.vl["SPEED_1"]["SPEED_LEFT"] + cp.vl["SPEED_1"]["SPEED_RIGHT"]) / 2.
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["GEAR"]["PRNDL"], None))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = False #not ret.vEgoRaw > 0.001
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["ESP_6"]["WHEEL_SPEED_FL"],
      cp.vl["ESP_6"]["WHEEL_SPEED_FR"],
      cp.vl["ESP_6"]["WHEEL_SPEED_RL"],
      cp.vl["ESP_6"]["WHEEL_SPEED_RR"],
      unit=1,
    )

    # button presses
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(200, cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 1,
                                                                       cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 2)
    ret.genericToggle = cp.vl["STEERING_LEVERS"]["HIGH_BEAM_PRESSED"] == 1
    
    # if accelcruise pressed set self.longEnabled to True
    ret.cruiseState.speed = self.cruisespeed
    if cp.vl["CRUISE_BUTTONS"]["ACC_Accel"] == 1 or cp.vl["CRUISE_BUTTONS"]["ACC_Decel"] == 1 or cp.vl["CRUISE_BUTTONS"]["ACC_Resume"] == 1:
      self.longEnabled = True
    elif cp.vl["CRUISE_BUTTONS"]["ACC_Cancel"] == 1 or ret.brakePressed == 1:
      self.longEnabled = False

    # steering wheel
    ret.steeringAngleDeg = cp.vl["STEERING"]["STEERING_ANGLE"] + cp.vl["STEERING"]["STEERING_ANGLE_HP"]
    ret.steeringRateDeg = cp.vl["STEERING"]["STEERING_RATE"]
    ret.steeringTorque = cp.vl["EPS_2"]["COLUMN_TORQUE"]
    ret.steeringTorqueEps = cp.vl["EPS_2"]["EPS_TORQUE_MOTOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # cruise state
    cp_cruise = cp_cam if self.CP.carFingerprint in RAM_CARS else cp

    ret.cruiseState.enabled = self.longEnabled
    ret.cruiseState.available = True
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.standstill = ret.standstill
    # ret.accFaulted = cp_cruise.vl["DAS_3"]["ACC_FAULTED"] != 0


    
    self.torqMax = cp.vl["ECM_TRQ"]["ENGINE_TORQ_MAX"]
    self.engineRpm = cp.vl["ECM_1"]["ENGINE_RPM"]
    self.engineTorque = cp.vl["ECM_1"]["ENGINE_TORQUE"]
    self.inputSpeed = cp.vl["TRANS_SPEED"]["INPUT_SPEED"]
    self.tcLocked = cp.vl["TRANS_SPEED"]["TC_LOCKED"]
    self.tcSlipPct = (self.inputSpeed/(self.engineRpm + 0.001)) + 0.001

    if self.CP.carFingerprint in RAM_CARS:
      self.auto_high_beam = cp_cam.vl["DAS_6"]['AUTO_HIGH_BEAM_ON']  # Auto High Beam isn't Located in this message on chrysler or jeep currently located in 729 message
      ret.steerFaultTemporary  = cp.vl["EPS_3"]["DASM_FAULT"] == 1
      self.lkasbutton = (cp.vl["Center_Stack_2"]["LKAS_Button"] == 1) or (cp.vl["Center_Stack_1"]["LKAS_Button"] == 1)
      if self.lkasbutton ==1 and self.lkasdisabled== 0 and self.lkasbuttonprev == 0:
        self.lkasdisabled = 1
      elif self.lkasbutton ==1 and self.lkasdisabled == 1 and self.lkasbuttonprev == 0:
        self.lkasdisabled = 0
      self.lkasbuttonprev = self.lkasbutton
    else:
      ret.steerFaultPermanent = cp.vl["EPS_2"]["LKAS_STATE"] == 4
      self.lkasbutton = (cp.vl["TRACTION_BUTTON"]["TOGGLE_LKAS"] == 1)
      if self.lkasbutton ==1 and self.lkasdisabled== 0 and self.lkasbuttonprev == 0:
        self.lkasdisabled = 1
      elif self.lkasbutton ==1 and self.lkasdisabled == 1 and self.lkasbuttonprev == 0:
        self.lkasdisabled = 0
      self.lkasbuttonprev = self.lkasbutton

    # blindspot sensors
    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["BSM_1"]["LEFT_STATUS"] == 1
      ret.rightBlindspot = cp.vl["BSM_1"]["RIGHT_STATUS"] == 1

    self.lkas_car_model = cp_cam.vl["DAS_6"]["CAR_MODEL"]
    # self.cruise_cancel = cp.vl["CRUISE_BUTTONS"]["ACC_Cancel"]
    self.button_counter = cp.vl["CRUISE_BUTTONS"]["COUNTER"]
    self.cruise_buttons = cp.vl["CRUISE_BUTTONS"]

    ret.buttonEvents = self.create_button_events(cp, self.CCP.BUTTONS)
    self.cruisespeed = ret.cruiseState.speed

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("DOOR_OPEN_FL", "BCM_1"),
      ("DOOR_OPEN_FR", "BCM_1"),
      ("DOOR_OPEN_RL", "BCM_1"),
      ("DOOR_OPEN_RR", "BCM_1"),
      ("Brake_Pedal_State", "ESP_1"),
      ("Vehicle_Speed", "ESP_1"),
      ("Accelerator_Position", "ECM_5"),
      ("WHEEL_SPEED_FL", "ESP_6"),
      ("WHEEL_SPEED_RR", "ESP_6"),
      ("WHEEL_SPEED_RL", "ESP_6"),
      ("WHEEL_SPEED_FR", "ESP_6"),
      ("STEERING_ANGLE", "STEERING"),
      ("STEERING_ANGLE_HP", "STEERING"),
      ("STEERING_RATE", "STEERING"),
      ("TURN_SIGNALS", "STEERING_LEVERS"),
      ("HIGH_BEAM_PRESSED", "STEERING_LEVERS"),
      ("SEATBELT_DRIVER_UNLATCHED", "ORC_1"),
      ("COUNTER", "EPS_2",),
      ("COLUMN_TORQUE", "EPS_2"),
      ("EPS_TORQUE_MOTOR", "EPS_2"),
      ("LKAS_STATE", "EPS_2"),
      ("ACC_Cancel", "CRUISE_BUTTONS"),
      ("ACC_Distance_Dec", "CRUISE_BUTTONS"),
      ("ACC_Accel", "CRUISE_BUTTONS"),
      ("ACC_Decel", "CRUISE_BUTTONS"),
      ("ACC_Resume", "CRUISE_BUTTONS"),
      ("Cruise_OnOff", "CRUISE_BUTTONS"),
      ("ACC_OnOff", "CRUISE_BUTTONS"),
      ("ACC_Distance_Inc", "CRUISE_BUTTONS"),
      ("COUNTER", "CRUISE_BUTTONS"),

      ("ENGINE_RPM", "ECM_1", 0),
      ("ENGINE_TORQUE", "ECM_1", 0),
      ("ENGINE_TORQ_MIN", "ECM_TRQ", 0),
      ("ENGINE_TORQ_MAX", "ECM_TRQ", 0),
      ("INPUT_SPEED", "TRANS_SPEED"),
      ("TC_LOCKED", "TRANS_SPEED"),
    ]

    checks = [
      # sig_address, frequency
      ("ESP_1", 50),
      ("EPS_2", 100),
      ("ESP_6", 50),
      ("STEERING", 100),
      ("ECM_5", 50),
      ("CRUISE_BUTTONS", 50),
      ("STEERING_LEVERS", 10),
      ("ORC_1", 2),
      ("BCM_1", 1),
      ("ECM_1", 50),
      ("ECM_TRQ", 50),
      ("TRANS_SPEED", 50),
    ]

    if CP.enableBsm:
      signals += [
        ("RIGHT_STATUS", "BSM_1"),
        ("LEFT_STATUS", "BSM_1"),
      ]
      checks.append(("BSM_1", 2))

    if CP.carFingerprint in RAM_CARS:
      signals += [
        ("DASM_FAULT", "EPS_3"),
        ("Vehicle_Speed", "ESP_8"),
        ("Gear_State", "Transmission_Status"),
        ("LKAS_Button", "Center_Stack_1"),
        ("LKAS_Button", "Center_Stack_2"),
      ]
      checks += [
        ("ESP_8", 50),
        ("EPS_3", 50),
        ("Transmission_Status", 50),
        ("Center_Stack_1", 1),
        ("Center_Stack_2", 1),
      ]
    else:
      signals += [
        ("PRNDL", "GEAR"),
        ("SPEED_LEFT", "SPEED_1"),
        ("SPEED_RIGHT", "SPEED_1"),
        ("TOGGLE_LKAS", "TRACTION_BUTTON"),
      ]
      checks += [
        ("GEAR", 50),
        ("SPEED_1", 100),
        ("TRACTION_BUTTON", 1),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("CAR_MODEL", "DAS_6"),
    ]
    checks = [
      ("DAS_6", 4),
    ]

    if CP.carFingerprint in RAM_CARS:
      signals += [
        ("AUTO_HIGH_BEAM_ON", "DAS_6"),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)