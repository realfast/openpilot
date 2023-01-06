from cereal import car
from common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.chrysler.values import DBC, STEER_THRESHOLD, RAM_CARS

from common.op_params import opParams
ButtonType = car.CarState.ButtonEvent.Type

CHECK_BUTTONS = {ButtonType.accOnOff: ["CRUISE_BUTTONS", 'ACC_OnOff'],
                 ButtonType.cancel: ["CRUISE_BUTTONS", 'ACC_Cancel'],
                 ButtonType.resumeCruise: ["CRUISE_BUTTONS", 'ACC_Resume'],
                 ButtonType.accelCruise: ["CRUISE_BUTTONS", 'ACC_Accel'],
                 ButtonType.decelCruise: ["CRUISE_BUTTONS", 'ACC_Decel'],
                 ButtonType.followInc: ["CRUISE_BUTTONS", 'ACC_Distance_Inc'],
                 ButtonType.followDec: ["CRUISE_BUTTONS", 'ACC_Distance_Dec'],
                 ButtonType.lkasToggle: ["TRACTION_BUTTON", 'TOGGLE_LKAS']}

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
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
    self.op_params = opParams()

    if CP.carFingerprint in RAM_CARS:
      self.shifter_values = can_define.dv["Transmission_Status"]["Gear_State"]
    else: #Pacifica
      self.shifter_values = can_define.dv["GEAR"]["PRNDL"]
      self.lkasHeartbit = None
      self.longAvailable = False
      self.longEnabled = False
      self.allowLong = True

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
    if self.op_params.get('use_smoothed_accel'):
        ret.vEgoRaw = cp.vl["ESP_1"]["Vehicle_Speed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not ret.vEgoRaw > 0.001
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

    # steering wheel
    ret.steeringAngleDeg = cp.vl["STEERING"]["STEERING_ANGLE"] + cp.vl["STEERING"]["STEERING_ANGLE_HP"]
    ret.steeringRateDeg = cp.vl["STEERING"]["STEERING_RATE"]
    ret.steeringTorque = cp.vl["EPS_2"]["COLUMN_TORQUE"]
    ret.steeringTorqueEps = cp.vl["EPS_2"]["EPS_TORQUE_MOTOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # cruise state
    cp_cruise = cp_cam if self.CP.carFingerprint in RAM_CARS else cp
    if self.CP.carFingerprint in RAM_CARS:
      ret.cruiseState.available = cp_cruise.vl["DAS_3"]["ACC_AVAILABLE"] == 1
      ret.cruiseState.enabled = cp_cruise.vl["DAS_3"]["ACC_ACTIVE"] == 1
      ret.cruiseState.speed = cp_cruise.vl["DAS_4"]["ACC_SET_SPEED_KPH"] * CV.KPH_TO_MS
      ret.cruiseState.nonAdaptive = cp_cruise.vl["DAS_4"]["ACC_STATE"] in (1, 2)  # 1 NormalCCOn and 2 NormalCCSet
      ret.cruiseState.standstill = cp_cruise.vl["DAS_3"]["ACC_STANDSTILL"] == 1
      ret.accFaulted = cp_cruise.vl["DAS_3"]["ACC_FAULTED"] != 0
      self.das_3 = cp_cruise.vl['DAS_3']
      self.torqMin = cp_cruise.vl["DAS_3"]["ENGINE_TORQUE_REQUEST"]
      self.maxgear = cp_cruise.vl["DAS_3"]["GR_MAX_REQ"]


    
      self.torqMax = cp.vl["ECM_TRQ"]["ENGINE_TORQ_MAX"]
      self.engineRpm = cp.vl["ECM_1"]["ENGINE_RPM"]
      self.engineTorque = cp.vl["ECM_1"]["ENGINE_TORQUE"]

    else:
      ret.cruiseState.nonAdaptive = False
      ret.cruiseState.enabled = self.longEnabled
      ret.cruiseState.available = self.longAvailable
      ret.cruiseState.standstill = ret.standstill
      ret.accFaulted = False
      self.torqMin = cp.vl["ECM_TRQ"]["ENGINE_TORQ_MIN"]
      self.torqMax = cp.vl["ECM_TRQ"]["ENGINE_TORQ_MAX"]
      self.currentGear = cp.vl['TCM_A7']["CurrentGear"]
      self.engineRpm = cp.vl["ECM_1"]["ENGINE_RPM"]
      self.engineTorque = cp.vl["ECM_1"]["ENGINE_TORQUE"]
      self.maxgear = 9
      self.lkasHeartbit = cp_cam.vl["LKAS_HEARTBIT"]


    if self.CP.carFingerprint in RAM_CARS:
      self.auto_high_beam = cp_cam.vl["DAS_6"]['AUTO_HIGH_BEAM_ON']  # Auto High Beam isn't Located in this message on chrysler or jeep currently located in 729 message
      ret.steerFaultTemporary  = cp.vl["EPS_3"]["DASM_FAULT"] == 1
      self.lkasbutton = (cp.vl["Center_Stack_2"]["LKAS_Button"] == 1) or (cp.vl["Center_Stack_1"]["LKAS_Button"] == 1)
      if self.lkasbutton ==1 and self.lkasdisabled== 0 and self.lkasbuttonprev == 0:
        self.lkasdisabled = 1
      elif self.lkasbutton ==1 and self.lkasdisabled == 1 and self.lkasbuttonprev == 0:
        self.lkasdisabled = 0
      self.lkasbuttonprev = self.lkasbutton
    else: #Pacifica
      ret.steerFaultPermanent = cp.vl["EPS_2"]["LKAS_STATE"] == 4
      button_events = []
      for buttonType in CHECK_BUTTONS:
        self.check_button(button_events, buttonType, bool(cp.vl[CHECK_BUTTONS[buttonType][0]][CHECK_BUTTONS[buttonType][1]]))
      ret.buttonEvents = button_events

    # blindspot sensors
    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["BSM_1"]["LEFT_STATUS"] == 1
      ret.rightBlindspot = cp.vl["BSM_1"]["RIGHT_STATUS"] == 1

    self.lkas_car_model = cp_cam.vl["DAS_6"]["CAR_MODEL"]
    self.cruise_cancel = cp.vl["CRUISE_BUTTONS"]["ACC_Cancel"]
    self.button_counter = cp.vl["CRUISE_BUTTONS"]["COUNTER"]
    self.cruise_buttons = cp.vl["CRUISE_BUTTONS"]
    return ret
    
  def check_button(self, button_events, button_type, pressed):
    pressed_frames = 0
    pressed_changed = False
    for ob in self.out.buttonEvents:
      if ob.type == button_type:
        pressed_frames = ob.pressedFrames
        pressed_changed = ob.pressed != pressed
        break

    if pressed or pressed_changed:
      be = car.CarState.ButtonEvent.new_message()
      be.type = button_type
      be.pressed = pressed
      be.pressedFrames = pressed_frames

      if not pressed_changed:
        be.pressedFrames += 1

      button_events.append(be)

  def button_pressed(self, button_type, pressed=True):
    for b in self.out.buttonEvents:
      if b.type == button_type:
        if b.pressed == pressed:
          return b
        break

  @staticmethod
  def get_cruise_signals():
    signals = []
    checks = []
    if self.CP.carFingerprint in RAM_CARS:
      signals += [
        ("ACC_AVAILABLE", "DAS_3"),
        ("ACC_ACTIVE", "DAS_3"),
        ("ACC_FAULTED", "DAS_3"),
        ("ACC_STANDSTILL", "DAS_3"),
        ("COUNTER", "DAS_3"),
        ("ACC_SET_SPEED_KPH", "DAS_4"),
        ("ACC_STATE", "DAS_4"),

        ("ACC_GO", "DAS_3", 0),
        ("ENGINE_TORQUE_REQUEST", "DAS_3", 0),
        ("ENGINE_TORQUE_REQUEST_MAX", "DAS_3", 0),
        ("ACC_DECEL", "DAS_3", 0),
        ("ACC_DECEL_REQ", "DAS_3", 0),
        ("ACC_AVAILABLE", "DAS_3", 0),
        ("DISABLE_FUEL_SHUTOFF", "DAS_3", 0),
        ("GR_MAX_REQ", "DAS_3", 0),
        ("STS", "DAS_3", 0),
        ("COLLISION_BRK_PREP", "DAS_3", 0),
        ("ACC_BRK_PREP", "DAS_3", 0),
        ("DISPLAY_REQ", "DAS_3", 0),
        ("COUNTER", "DAS_3", 0),
        ("CHECKSUM", "DAS_3", 0),
      ]
      checks += [
        ("DAS_3", 50),
        ("DAS_4", 50),
      ]
    return signals, checks

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
      ("CurrentGear", "TCM_A7", 0),
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
      ("TCM_A7", 50),
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
        # ("SPEED_LEFT", "SPEED_1"),
        # ("SPEED_RIGHT", "SPEED_1"),
      ]
      checks += [
        ("GEAR", 50),
        # ("SPEED_1", 100),
      ]
      signals += CarState.get_cruise_signals()[0]
      checks += CarState.get_cruise_signals()[1]

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
      signals += CarState.get_cruise_signals()[0]
      checks += CarState.get_cruise_signals()[1]
    else:
      # LKAS_HEARTBIT data needs to be forwarded!
      forward_lkas_heartbit_signals = [
        ("AUTO_HIGH_BEAM", "LKAS_HEARTBIT"),
        ("FORWARD_1", "LKAS_HEARTBIT"),
        ("FORWARD_2", "LKAS_HEARTBIT"),
        ("FORWARD_3", "LKAS_HEARTBIT"),
      ]

      signals += forward_lkas_heartbit_signals
      checks.append(("LKAS_HEARTBIT", 10))
    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)