from cereal import car
from selfdrive.car.chrysler.values import RAM_CARS

GearShifter = car.CarState.GearShifter
VisualAlert = car.CarControl.HUDControl.VisualAlert
from common.conversions import Conversions as CV

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

# def create_lkas_heartbit(packer, value, lkasHeartbit):
#   # LKAS_HEARTBIT (697) LKAS heartbeat
#   values = lkasHeartbit.copy()  # forward what we parsed
#   values["LKAS_DISABLED"] = value
#   return packer.make_can_msg("LKAS_HEARTBIT", 0, values)
  
def create_cruise_buttons(packer, frame, bus, cruise_buttons, cancel=False, resume=False):
  
  if (cancel == True) or (resume == True):
    values = {
      "ACC_Cancel": cancel,
      "ACC_Resume": resume,
      "COUNTER": frame % 0x10,
    }
  else:
    values = cruise_buttons.copy()
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)

def acc_command(packer, counter, bus, available, enabled, accel_req=0, torque=0, max_gear=8, decel_req=0, decel=0, das_3=0, not_RAM = 0, stand_still = 0):
  if not_RAM == 1:
    values = {
    'ACC_AVAILABLE': available,
    'ACC_ACTIVE': enabled,
    'COUNTER': counter % 0x10,
    'ACC_DECEL_REQ': decel_req,
    'ACC_DECEL': decel,
    'ENGINE_TORQUE_REQUEST_MAX': 0 if torque < 30 else 1,
    'ENGINE_TORQUE_REQUEST': torque if torque > 0 else 0,
    'GR_MAX_REQ': 9 if max_gear is None else max_gear,
    'ACC_STANDSTILL': 0, #stand_still,
    'ACC_GO': accel_req,
  }
    
  else: 
    values = das_3.copy()  # forward what we parsed
    values['ACC_DECEL_REQ'] = enabled and decel is not None
    values['ACC_GO'] = accel_req
    values['ACC_STANDSTILL'] = decel_req
    if decel is not None:
      values['ACC_DECEL'] = decel
      values['ENGINE_TORQUE_REQUEST_MAX'] = enabled and torque is not None
    if torque is not None:
      values['ENGINE_TORQUE_REQUEST'] = torque
    values['ACC_ACTIVE'] = enabled
    values['COUNTER'] = counter % 0x10
    values['GR_MAX_REQ'] = max_gear

  return packer.make_can_msg("DAS_3", bus, values)
def create_acc_1_message(packer, bus, frame):
  values = {
    "ACCEL_PERHAPS": 32767,
    "COUNTER": frame % 0x10,
  }

  return packer.make_can_msg("ACC_1", bus, values)

def create_das_4_message(packer, bus, state, speed):
  values = {
    "ACC_DISTANCE_CONFIG_1": 0x1,
    "ACC_DISTANCE_CONFIG_2": 0x1,
    "SPEED_DIGITAL": 0xFE,
    "ALWAYS_ON": 0x1,
    "ACC_STATE": state,
    "ACC_SET_SPEED_KPH": round(speed * CV.MS_TO_KPH),
    "ACC_SET_SPEED_MPH": round(speed * CV.MS_TO_MPH),
  }

  return packer.make_can_msg("DAS_4", bus, values) 

def create_chime_message(packer, bus):
  values = { # 1000ms
    # "CHIME": chime if (chime_timer > 0 and (gap_timer == 0 or gap_timer == chimegap_time)) else 14,
    # "CHIME_REQ_L": 1 if (chime_timer > 0 and (gap_timer == 0 or gap_timer == chimegap_time)) else 0,
    # "CHIME_REQ_R": 1 if (chime_timer > 0 and (gap_timer == 0 or gap_timer == chimegap_time)) else 0
  }
  return packer.make_can_msg("CHIME", bus, values)
