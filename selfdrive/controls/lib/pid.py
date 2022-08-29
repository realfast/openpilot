import numpy as np
from numbers import Number

from common.numpy_fast import clip, interp
from common.op_params import opParams, MAX_LAT_ACCEL


class PIDController():
  def __init__(self, k_p, k_i, k_f=0., k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100, is_lateral = False, OP=None):
    self.is_lateral = is_lateral
    if OP is None:
      OP = opParams()
    self.op_params = OP
    self._k_p = k_p
    self._k_i = k_i
    self._k_d = k_d
    self.k_f = k_f   # feedforward gain
    if is_lateral:
      self.pidList = [k_p,k_i,k_d,k_f]
    if isinstance(self._k_p, Number):
      self._k_p = [[0], [self._k_p]]
    if isinstance(self._k_i, Number):
      self._k_i = [[0], [self._k_i]]
    if isinstance(self._k_d, Number):
      self._k_d = [[0], [self._k_d]]

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.speed = 0.0

    self.reset()

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return interp(self.speed, self._k_d[0], self._k_d[1])

  @property
  def error_integral(self):
    return self.i/self.k_i

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0

  def _update_params(self):
    self._k_p = 1.0 / self.op_params.get(MAX_LAT_ACCEL)
    self._k_i = 0.1 / self.op_params.get(MAX_LAT_ACCEL)
    self.k_f = 1.0 / self.op_params.get(MAX_LAT_ACCEL)
    self._k_d = 0

  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
    self.speed = speed

    if(self.is_lateral):
      self._update_params()
      self.p = float(error) * self._k_p
      self.f = feedforward * self.k_f
      self.d = error_rate * self._k_d
    
    else:
      self.p = float(error) * self.k_p
      self.f = feedforward * self.k_f
      self.d = error_rate * self.k_d

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      if (self.is_lateral):
        i = self.i + error * self._k_i * self.i_rate
      else:
        i = self.i + error * self.k_i * self.i_rate
      control = self.p + i + self.d + self.f

      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.i = i

    control = self.p + self.i + self.d + self.f

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control
