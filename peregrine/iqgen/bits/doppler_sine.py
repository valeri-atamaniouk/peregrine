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
The :mod:`peregrine.iqgen.doppler_sine` module contains classes and functions
related to generation of signals with circular changing doppler.

"""

import scipy.constants

class Doppler(object):

  NAME = "SineDoppler"

  '''
  Doppler control for an object that has peridic acceleration.
  '''
  def __init__(self, distance0_m, speed0_mps, amplutude_mps, period_s):
    '''
    Constructs doppler control object for linear acceleration.

    Parameters
    ----------
    distance0_m : float
      Initial distance to satellite at time 0 in meters for computing signal
      phase and delay.
    speed0_mps : float
      Speed of satellite at time 0 in meters per second.
    amplutude_mps : float
      Amplitude of change
    period_s : float
      Period of change
    '''
    super(Doppler, self).__init__()
    self.distance0_m = distance0_m
    self.speed0_mps = speed0_mps
    self.amplutude_mps = amplutude_mps
    self.period_s = self.period_s
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
    T_user = T_sv + D(T_sv)/c, for D(t) = D_0 + A*sin(T_sv/P)

    Parameters
    ----------
    userTime_s : float
      Observer's time in seconds

    Returns
    -------
    float
      Satellite vehicle's time at which signal has been generated.
    '''

    if userTime_s != 0:
      raise ValueError()

    return -self.distance0_m / scipy.constants.c

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
    # D(t) = D_0 + A * sin(2 * pi / P * t)
    # I(D(t)dt)=D_0 * t + A * (1 - cos(2 * pi / P * t)) * P / (2 * pi)
    # I= (D_0 + F_i) * t + D_1 * (1 - cos( D_2 * t)) + D_3
    
    freqRatio = -carrierSignal.CENTER_FREQUENCY_HZ / scipy.constants.c 
    D_0 = self.speed0_mps * freqRatio 
    D_1 = 2. * scipy.constants.pi / self.period_s * freqRatio
    D_2 = freqRatio * self.amplutude_mps * self.period_s / (2. * scipy.constants.pi)
    D_3 = self.distance0_m * freqRatio
    
    algMode = 1
    if algMode == 1:
      phaseAll = scipy.copy(userTimeAll_s)
      phaseAll *= D_2
      numpy.cos(phaseAll, out=phaseAll)
      numpy.negative(phaseAll, out=phaseAll)
      phaseAll += 1.
      phaseAll *= D_1
      phaseAll += (D_0 + ifFrequency_hz) * userTimeAll_s
      phaseAll += D_3
      phaseAll *= 2 * scipy.constants.pi
    elif algMode == 2:
      pass
    elif algMode == 3:
      pass

    # Convert phase to signal value and multiply by amplitude
    signal = scipy.sin(phaseAll)
    signal *= amplitude

    # PRN and data index computation
    # Computing doppler coefficients
    
    chipFreqRatio = carrierSignal.CODE_CHIP_RATE_HZ / carrierSignal.CENTER_FREQUENCY_HZ
    D_C0 = D_0 * chipFreqRatio
    D_C1 = D_1 * chipFreqRatio
    D_C2 = D_2 * chipFreqRatio
    D_C3 = D_3 * chipFreqRatio

    if algMode == 1:
      chipAll_idx = userTimeAll_s * D_C2
      numpy.cos(chipAll_idx, out=chipAll_idx)
      numpy.negative(chipAll_idx, out=chipAll_idx)
      chipAll_idx += 1.
      chipAll_idx *= D_C1
      chipAll_idx += (D_C0 + ifFrequency_hz) * userTimeAll_s
      chipAll_idx += D_C3
    elif algMode == 2:
      pass
    elif algMode == 3:
      pass

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

def sineDoppler(distance0_m, doppler0_hz, frequency_hz, dopplerAmplitude_hz, dopplerPeriod_s):
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
  dopplerAmplitude_hz : float
    Doppler change amplitude in Hz
  dopplerPeriod_s : float
    Doppler change period in seconds

  Returns
  -------
  Doppler
    object that implments constant acceleration logic.
  '''
  speed0_mps = -scipy.constants.c / frequency_hz * doppler0_hz
  amplitude_mps = -scipy.constants.c / frequency_hz * dopplerAmplitude_hz

  return Doppler(distance0_m, speed0_mps, amplitude_mps, dopplerPeriod_s)
