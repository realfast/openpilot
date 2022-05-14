import math
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.discrete import DiscreteController
from common.realtime import DT_CTRL
from common.numpy_fast import clip
from cereal import log
from common.op_params import opParams, MAX_LAT_ACCEL, LAT_KP_V, LAT_KI_V, LAT_KD_V

class LatControlTorque(LatControl):
  def __init__(self, CP, CI, OP=None):
    if OP is None:
      OP = opParams()
    self.op_params = OP
    super().__init__(CP, CI)
    
    p = self.op_params.get(LAT_KP_V) / self.op_params.get(MAX_LAT_ACCEL)
    i = self.op_params.get(LAT_KI_V) / self.op_params.get(MAX_LAT_ACCEL)
    d = self.op_params.get(LAT_KD_V) / self.op_params.get(MAX_LAT_ACCEL)
    gains = [i, p, d]
    N = 10 # Filter coefficient. corner frequency in rad/s. 20 = ~3.18hz
    Z = [[[1, 1], [2, -2]], [[1], [1]], [[2, -2], [1-2j, 1+2j]]] # Trapezoidal IPD
    T = [[[1, 0], [1    ]], [[1], [1]], [[1    ], [1   , (1/N)*1j]]] # Trapezoidal IPD
    self.pid = DiscreteController(gains, Z, T, rate=(1 / DT_CTRL))
    
    self.steer_max = 1.0
    self.use_steering_angle = CP.lateralTuning.torque.useSteeringAngle

  def reset(self):
    super().reset()
    self.pid.reset()

  def update(self, active, CS, VM, params, last_actuators, desired_curvature, desired_curvature_rate, llk):

    p = self.op_params.get(LAT_KP_V) / self.op_params.get(MAX_LAT_ACCEL)
    i = self.op_params.get(LAT_KI_V) / self.op_params.get(MAX_LAT_ACCEL)
    d = self.op_params.get(LAT_KD_V) / self.op_params.get(MAX_LAT_ACCEL)
    gains = [i, p, d]
    self.pid.update_gains()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_torque = 0.0
      self.reset()
    else:
      error = -(desired_curvature_rate) * (CS.vEgo**2)
      output_torque = self.pid.update(error, last_actuators.steer)
      output_torque = clip(output_torque, -self.steer_max, self.steer_max)
      
      pid_log.error = error
      pid_log.active = True
      pid_log.i = float(self.pid.gains[0]*self.pid.d[0][1])
      pid_log.p = float(self.pid.gains[1]*self.pid.d[1][1])
      pid_log.d = float(self.pid.gains[2]*self.pid.d[2][1])
      pid_log.output = -output_torque
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS)
      pid_log.actualLateralAccel = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll) * (CS.vEgo**2)
      pid_log.desiredLateralAccel = desired_curvature * (CS.vEgo**2)

    return output_torque, 0.0, pid_log
