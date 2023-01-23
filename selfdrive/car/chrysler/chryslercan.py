from cereal import car
from selfdrive.car.chrysler.values import RAM_CARS
from common.conversions import Conversions as CV

GearShifter = car.CarState.GearShifter
VisualAlert = car.CarControl.HUDControl.VisualAlert

def create_lkas_hud(packer, CP, lkas_active, hud_alert, hud_count, car_model, CS):
  # LKAS_HUD - Controls what lane-keeping icon is displayed

  # == Color ==
  # 0 hidden?
  # 1 white
  # 2 green
  # 3 ldw

  # == Lines ==
  # 03 white Lines
  # 04 grey lines
  # 09 left lane close
  # 0A right lane close
  # 0B left Lane very close
  # 0C right Lane very close
  # 0D left cross cross
  # 0E right lane cross

  # == Alerts ==
  # 7 Normal
  # 6 lane departure place hands on wheel

  color = 2 if lkas_active and not CS.lkasdisabled else 1
  lines = 3 if lkas_active else 0
  alerts = 7 if lkas_active else 0

  if hud_count < (1 * 4):  # first 3 seconds, 4Hz
    alerts = 1

  if hud_alert in (VisualAlert.ldw, VisualAlert.steerRequired):
    color = 4
    lines = 0
    alerts = 6

  values = {
    "LKAS_ICON_COLOR": color,
    "CAR_MODEL": car_model,
    "LKAS_LANE_LINES": lines,
    "LKAS_ALERTS": alerts,
  }

  if CP.carFingerprint in RAM_CARS:
    values = {
      "AUTO_HIGH_BEAM_ON": CS.auto_high_beam,
      "LKAS_Disabled":CS.lkasdisabled,
    }

  return packer.make_can_msg("DAS_6", 0, values)


def create_lkas_command(packer, CP, apply_steer, lkas_control_bit):
  # LKAS_COMMAND Lane-keeping signal to turn the wheel
  enabled_val = 2 if CP.carFingerprint in RAM_CARS else 1
  values = {
    "STEERING_TORQUE": apply_steer,
    "LKAS_CONTROL_BIT": enabled_val if lkas_control_bit else 0,
  }
  return packer.make_can_msg("LKAS_COMMAND", 0, values)


def create_cruise_buttons(packer, frame, bus, cruise_buttons, cancel=False, resume=False, accel = False, decel = False):
  
  if (cancel == True) or (resume == True) or (accel == True) or (decel == True):
    values = {
      "ACC_Cancel": cancel,
      "ACC_Resume": resume,
      "ACC_Accel": accel,
      "ACC_Decel": decel,
      "COUNTER": frame % 0x10,
    }
  else:
    values = cruise_buttons.copy()
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)

def das_3_message(packer, counter, enabled, accel_req, decel_req, accel_go, torque, max_gear, standstill, decel):
  values = {
    'ACC_AVAILABLE': 1,
    'ACC_ACTIVE': enabled,
    'COUNTER': counter % 0x10,
    'ACC_DECEL_REQ': decel_req,
    'ACC_DECEL': decel,
    'ENGINE_TORQUE_REQUEST_MAX': accel_req,
    'ENGINE_TORQUE_REQUEST': torque,
    'GR_MAX_REQ': 9 if max_gear is None else max_gear,
    'ACC_STANDSTILL': standstill,#  stand_still,
    'ACC_GO': accel_go,
  }

  return packer.make_can_msg("DAS_3", 0, values)

def das_4_message(packer, bus, state, speed):
  values = {
    "ACC_DISTANCE_CONFIG_1": 0x1,
    "ACC_DISTANCE_CONFIG_2": 0x1,
    "SPEED_DIGITAL": 0xFE,
    "FCW_BRAKE_ENABLED": 0x1,
    "ACC_STATE": state,
    "ACC_SET_SPEED_KPH": round(speed * CV.MS_TO_KPH),
    "ACC_SET_SPEED_MPH": round(speed * CV.MS_TO_MPH),
  }

  return packer.make_can_msg("DAS_4", bus, values) 

def das_5_message(packer, bus, speed):
  values = {
    "FCW_STATE": 0x1,
    "FCW_DISTANCE": 0x2,
    "SET_SPEED_KPH": round(speed * CV.MS_TO_KPH),
    "COUNTER1": 0x0,
    "CHECKSUM1": 0xFF,
  }

  return packer.make_can_msg("DAS_5", bus, values)

def acc_log(packer, aTarget, vTarget, calcvTarget, aActual, vActual):
  values = {
    'OP_A_TARGET': aTarget,
    'OP_V_TARGET': vTarget,
    'CALC_V_TARGET': calcvTarget,
    'OP_A_ACTUAL': aActual,
    'OP_V_ACTUAL': vActual,
  }
  return packer.make_can_msg("ACC_LOG", 0, values)