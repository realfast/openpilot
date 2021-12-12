from cereal import car
from selfdrive.car import make_can_msg


GearShifter = car.CarState.GearShifter
VisualAlert = car.CarControl.HUDControl.VisualAlert



def create_lkas_command(packer, apply_steer, moving_fast, frame):
  # LKAS_COMMAND Lane-keeping signal to turn the wheel.
  values = {
    "LKAS_STEERING_TORQUE": apply_steer,
    "LKAS_CONTROL_BIT": int(moving_fast),
    "COUNTER": frame % 0x10,
  }
  return packer.make_can_msg("LKAS_COMMAND", 0, values)


def create_wheel_buttons(packer, frame, cancel=False):
  # Cruise_Control_Buttons Message sent to cancel ACC.
  values = {
    "ACC_Cancel": cancel,
    "COUNTER": frame % 10
  }
  return packer.make_can_msg("Cruise_Control_Buttons", 0, values)
