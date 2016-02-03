# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.doppler_linear` module contains classes and functions
related to generation of signals with linearily changing doppler.

"""

import math
import scipy.constants

class Doppler(object):
  '''
  Doppler control for an object that has constant acceleration. Such signal has
  constant doppler value with a possible sign invert.
  '''
  def __init__(self, distance0_m, speed0_mps, acceleration_mps2):
    '''
    Constructs doppler control object for linear acceleration.

    Parameters
    ----------
    distance0_m : float
      Initial distance to satellite at time 0 in meters for computing signal
      phase and delay.
    speed0_mps : float
      Speed of satellite at time 0 in meters per second.
    acceleration_mps2 : float
      Acceleration of satellite
    '''
    super(Doppler, self).__init__()
    self.distance0_m = distance0_m
    self.speed0_mps = speed0_mps
    self.acceleation_mps2 = acceleration_mps2
    self.tau0_s = self.distance0_m / scipy.constants.c

  def computeDistanceM(self, svTime_s):
    '''
    Computes distance to satellite in meters.

    Parameters
    ----------
    svTime_s : float
      Time in seconds at which distance is computed. Please note that  is not
      a time of the observer.

    Returns
    -------
    float
      Distance to satellite in meters.
    '''
    return self.distance0_m + self.speed0_mps * svTime_s + 0.5 * self.acceleation_mps2 * svTime_s * svTime_s

  def computeSpeedMps(self, svTime_s):
    '''
    Computes speed of satellite in meters per second.

    Parameters
    ----------
    svTime_s : float
      Time in seconds at which speed is computed. Please note that  is not
      a time of the observer.

    Returns
    -------
    float
      Speed of satellite in meters per second.
    '''
    return self.speed0_mps + self.acceleation_mps2 * svTime_s

  def computeSvTimeS(self, userTime_s):
    '''
    Computes SV time from a user time.

    The computation is a solution of the formulae:
    T_user = T_sv + D(T_sv)/c, for D(t) = D_0 + V*T_sv

    Parameters
    ----------
    userTime_s : float
      Observer's time in seconds

    Returns
    -------
    float
      Satellite vehicle's time at which signal has been generated.
    '''

    _ax2 = self.acceleation_mps2
    _b = scipy.constants.c + self.speed0_mps
    _c = self.distance0_m - userTime_s * scipy.constants.c

    _d = _b * _b - 2 * _ax2 * _c
    _sd = math.sqrt(_d)
    _r0 = (-_b + _sd) / _ax2
    _r1 = (-_b - _sd) / _ax2
    if (_r0 < 0):
      _res = _r1
    elif (_r1 < 0):
      _res = _r0
    else:
      _res = min(_r0, _r1)

    return userTime_s - _res  # _tau1

  def computeBatch(self,
                   userTime0_s,
                   n_samples,
                   amplitude,
                   planFrequency_hz,
                   ifFrequency_hz,
                   message,
                   code):
    '''
    Computes signal samples for the doppler object.

    Parameters
    ----------
    userTime0_s : float
      Observer's time in seconds of the beginning of the interval.
    n_samples : int
      Number of samples to generate
    amplitude : float
      Signal amplitude.
    planFrequency_hz : float
      Central carrier frequency in hertz
    ifFrequency_hz: float
      Intermediate frequency in hertz
    message : object
      Message object for providing access to symbols
    code : object
      PRN code object for providing access to chips

    Returns
    -------
    signal : numpy.ndarray(n_samples, dtype=float)
      Generated samples
    userTimeX_s : float
      End of interval time in seconds
    chipAll_idx : numpy.ndarray(n_samples, dtype=float)
      Code chip phases for the samples
    chips : numpy.ndarray(n_samples, dtype=int)
      Code combined with data
    '''

    raise Exception

def linearDoppler(distance0m, dopplerShift0Hz, frequencyHz, dopplerShiftSpeedHzps):
  '''
  Makes an object that corresponds to linear doppler change.

  @param[in] distance0_m             Initial distance to object.
  @param[in] dopplerShift0Hz        Initial doppler shift in hz.
  @param[in] frequencyHz            Carrier frequency in Hz.
  @param[in] dopplerShiftChangeHzps Doppler shift rate in Hz per second.

  @return object that implments constant acceleration logic.
  '''
  _speed0 = -scipy.constants.c / frequencyHz * dopplerShift0Hz
  _accel = -scipy.constants.c / frequencyHz * dopplerShiftSpeedHzps
  if (distance0m < 0):
    _speed0 = -_speed0
    _accel = -_accel
  return Doppler(distance0m, _speed0, _accel)
