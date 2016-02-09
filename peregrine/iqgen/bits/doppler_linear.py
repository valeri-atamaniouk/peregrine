# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
from peregrine.iqgen import if_iface
from scipy.stats.vonmises_cython import numpy

"""
The :mod:`peregrine.iqgen.doppler_linear` module contains classes and functions
related to generation of signals with linearily changing doppler.

"""

import math
import scipy.constants

class Doppler(object):

  NAME = "LinearDoppler"

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
    self.acceleration_mps2 = acceleration_mps2
    self.tau0_s = self.distance0_m / scipy.constants.c
    self.phaseShift = 0. # For debugging
    self.chipShift = 0.

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
    return self.distance0_m + self.speed0_mps * svTime_s + 0.5 * self.acceleration_mps2 * svTime_s * svTime_s

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
    return self.speed0_mps + self.acceleration_mps2 * svTime_s

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

    _ax2 = self.acceleration_mps2

    if _ax2 == 0:
      return userTime_s - self.speed0_mps * userTime_s / scipy.constants.c

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
                   carrierSignal,
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
    carrierSignal : object
      Carrier frequency object
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

    TYPE=numpy.float128

    deltaUserTime_s = float(n_samples) / if_iface.Chip.SAMPLE_RATE_HZ
    userTimeX_s = userTime0_s + deltaUserTime_s
    userTimeAll_s = scipy.linspace(userTime0_s,
                                   userTimeX_s,
                                   n_samples,
                                   dtype=TYPE,
                                   endpoint=False)

    # Computing doppler coefficients
    freqRatio = -carrierSignal.CENTER_FREQUENCY_HZ / scipy.constants.c 
    D_0 = freqRatio * self.acceleration_mps2
    D_1 = freqRatio * self.speed0_mps
    D_2 = freqRatio * self.distance0_m

    # phaseAll[i] = 0.5 * D_0 * T[i]^2 + (D_1 + IF) * T[i] + D_2) * 2 * pi

    algMode = 1
    if algMode == 1:
      phaseAll = scipy.ndarray(n_samples, dtype=TYPE)
      phaseAll.fill(D_0 * 0.5)
      phaseAll *= userTimeAll_s
      phaseAll += D_1
      phaseAll *= userTimeAll_s
      phaseAll += ifFrequency_hz * userTimeAll_s + D_2
      phaseAll *= 2 * scipy.constants.pi
    elif algMode == 2:
      K1 = scipy.constants.pi * D_0
      K2 = 2 * scipy.constants.pi * (D_1 + ifFrequency_hz)
      phasePoint0 = K1 * userTime0_s + K2 
      phasePointX = K1 * userTimeX_s + K2
      phaseAll = scipy.linspace(phasePoint0,
                                phasePointX,
                                n_samples,
                                dtype=TYPE,
                                endpoint=False)
      phaseAll *= userTimeAll_s
      phaseAll += D_2
    elif algMode == 3:
      phi_E = 2 * scipy.constants.pi * D_0 / if_iface.Chip.SAMPLE_RATE_HZ
      phaseAll = scipy.ndarray(n_samples, dtype=TYPE)
      phaseAll.fill(phi_E + self.phaseShift)
      numpy.cumsum(phaseAll, out=phaseAll)
      # numpy.cumsum(phaseAll, out=phaseAll)
      self.phaseShift = phaseAll[n_samples - 1]
      phaseAll += userTimeAll_s * 2 * scipy.constants.pi * ifFrequency_hz

    # Convert phase to signal value and multiply by amplitude
    signal = scipy.sin(phaseAll)
    signal *= amplitude

    # PRN and data index computation
    # Computing doppler coefficients
    # Doppler for chips: chip[i] = 0.5 * D_C0 * T[i]^2 + (D_C1 + F_chip) * T[i] + D_C2
    
    chipFreqRatio = carrierSignal.CODE_CHIP_RATE_HZ / carrierSignal.CENTER_FREQUENCY_HZ
    D_C0 = D_0 * chipFreqRatio
    D_C1 = D_1 * chipFreqRatio
    D_C2 = D_2 * chipFreqRatio

    if algMode == 1:
      chipAll_idx = scipy.ndarray((n_samples), dtype=TYPE)
      chipAll_idx.fill(D_C0 * 0.5)
      chipAll_idx *= userTimeAll_s
      chipAll_idx += D_C1
      chipAll_idx *= userTimeAll_s
      chipAll_idx += carrierSignal.CODE_CHIP_RATE_HZ * userTimeAll_s
      chipAll_idx += D_C2
    elif algMode == 2:
      K3 = 0.5 * D_C0
      K4 = D_C1 + carrierSignal.CODE_CHIP_RATE_HZ
      chipAll_idx = scipy.linspace(K3 * userTime0_s + K4,
                                   K3 * userTimeX_s + K4,
                                   n_samples,
                                   dtype=TYPE,
                                   endpoint=False)
      chipAll_idx *= userTimeAll_s
      chipAll_idx += D_C2
    elif algMode == 3:
      phi_E = 2 * scipy.constants.pi * D_C0 / if_iface.Chip.SAMPLE_RATE_HZ
      chipAll_idx = scipy.ndarray(n_samples, dtype=TYPE)
      chipAll_idx.fill(phi_E + self.chipShift)
      numpy.cumsum(chipAll_idx, out=chipAll_idx)
      # numpy.cumsum(chipAll_idx, out=chipAll_idx)
      self.chipShift = chipAll_idx[n_samples - 1]
      chipAll_idx += userTimeAll_s * 2 * scipy.constants.pi * carrierSignal.CODE_CHIP_RATE_HZ
      print "phase shift ", chipAll_idx[n_samples - 1] - chipAll_idx[0], (chipAll_idx[n_samples - 1] - chipAll_idx[0]) / n_samples 

    # print chipAll_idx

    def dataChip(idx):
      chipIdx = long(idx)
      dataIdx = chipIdx / carrierSignal.CHIP_TO_SYMBOL_DIVIDER
      x = message.getBit(dataIdx) * code.getCodeBit(chipIdx)
      return x

    vdata = scipy.vectorize(dataChip)
    chips = vdata(chipAll_idx)
    # print "Chips 0..last=", chipAll_idx[0], chipAll_idx[n_samples-1]

    scipy.multiply(signal, chips, signal)
    return (signal, userTimeX_s, chipAll_idx, chips)

def linearDoppler(distance0_m, doppler0_hz, frequency_hz, dopplerChange_hzps):
  '''
  Makes an object that corresponds to linear doppler change.

  Parameters
  ----------
  distance0_m : float
    Initial distance to object.
  doppler0_hz : float
    Initial doppler shift in hz.
  frequency_hz
    Carrier frequency in Hz.
  dopplerChange_hzps : float
    Doppler shift rate in Hz per second.

  Returns
  -------
  Doppler
    object that implments constant acceleration logic.
  '''
  _speed0 = -scipy.constants.c / frequency_hz * doppler0_hz
  _accel = -scipy.constants.c / frequency_hz * dopplerChange_hzps

  return Doppler(distance0_m, _speed0, _accel)
