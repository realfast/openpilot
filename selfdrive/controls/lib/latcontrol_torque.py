import math
from selfdrive.controls.lib.discrete import DiscreteController
from common.numpy_fast import clip
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from common.realtime import DT_CTRL, DT_MDL
from cereal import log

LOW_SPEED_FACTOR = 200

class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    
    p = 120.0 / CP.lateralTuning.torque.maxLatAccel
    i = 180.0 / CP.lateralTuning.torque.maxLatAccel
    d = 1.5 / CP.lateralTuning.torque.maxLatAccel
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
    desired_curv =  desired_curvature + desired_curvature_rate * DT_MDL 
    
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    
    # logging desired angle
    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curv, CS.vEgo, params.roll))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringAngleDesiredDeg = angle_steers_des

    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_torque = 0.0
      pid_log.active = False
      self.reset()
    else:
      if self.use_steering_angle:
        actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      else:
        actual_curvature = llk.angularVelocityCalibrated.value[2] / CS.vEgo
      # desired curvature is actual curvature from MPC step. 
      # desired - actual curvature is error since last MPC step. 
      # desired + desired_rate*MDL is the real target for the current time step
      # desired_rate*MDL is the error at the last MPC step
      error = -(desired_curv - actual_curvature) *(CS.vEgo**2 + LOW_SPEED_FACTOR)
      output_torque = self.pid.update(error, last_actuators.steer)
      
      output_torque = clip(output_torque, -self.steer_max, self.steer_max)
      
      pid_log.error = error
      pid_log.active = True
      pid_log.i = float(self.pid.gains[0]*self.pid.d[0][1])
      pid_log.p = float(self.pid.gains[1]*self.pid.d[1][1])
      pid_log.d = float(self.pid.gains[2]*self.pid.d[2][1])
      pid_log.f = 0
      pid_log.output = -output_torque
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS)
      pid_log.actualLateralAccel = actual_curvature * (CS.vEgo**2)
      pid_log.desiredLateralAccel = desired_curv * (CS.vEgo**2)


    return output_torque, 0.0, pid_log
