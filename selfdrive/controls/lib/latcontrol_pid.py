import math

from cereal import log
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.pid import PIDController



#from common.op_params import opParams, LAT_KP_BP, LAT_KP_V, LAT_KI_BP, LAT_KI_V, LAT_KD_BP, LAT_KD_V, LAT_KF

class LatControlPID(LatControl):
  def __init__(self, CP, CI, OP=None):
    # if OP is None:
    #   OP = opParams()
    # self.op_params = OP
    # super().__init__(CP,CI)
    kp = 0 #(LAT_KP_BP, LAT_KP_V)
    ki = 0 #(LAT_KI_BP, LAT_KI_V)
    kd = 0 #(LAT_KD_BP, LAT_KD_V)
    kf = 0 #LAT_KF
    self.pid = PIDController(kp, ki, kf, kd, pos_limit=1.0, neg_limit=-1.0, isLateral=True)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()

  def reset(self):
    super().reset()
    self.pid.reset()

  def update(self, active, CS, VM, params, last_actuators, desired_curvature, desired_curvature_rate, llk):
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg
    error = angle_steers_des - CS.steeringAngleDeg

    pid_log.steeringAngleDesiredDeg = angle_steers_des
    pid_log.angleError = error
    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      self.pid.pos_limit = self.steer_max
      self.pid.neg_limit = -self.steer_max

      # offset does not contribute to resistive torque
      steer_feedforward = self.get_steer_feedforward(angle_steers_des_no_offset, CS.vEgo)

      output_steer = self.pid.update(error, override=CS.steeringPressed,
                                     feedforward=steer_feedforward, speed=CS.vEgo)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_steer) < 1e-3, CS)

    return output_steer, angle_steers_des, pid_log
