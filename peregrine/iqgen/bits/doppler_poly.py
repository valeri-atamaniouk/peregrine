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
related to generation of signals with polynomial-based movement.

"""

import numpy
import scipy.constants
from peregrine.iqgen.bits.doppler_base import DopplerBase

class Doppler(DopplerBase):
  '''
  Doppler control for an object that has constant acceleration. Such signal has
  constant doppler value with a possible sign invert.
  '''

  TWO_PI = scipy.constants.pi * 2

  def __init__(self, coeffs):
    '''
    Constructs doppler control object for linear acceleration.

    Parameters
    ----------
    coeffs : tuple
      Phase shift coefficients. Phase chift will be computed as:
      C_n*t^n + C_(n-1)^(n-1) + ... + C_2*t^2 + C_1*t + C_0
      C_n..C_0 - values for speed of light
    '''
    super(Doppler, self).__init__()
    self.coeffs = tuple([x for x in coeffs])
    self.n_coeffs = len(coeffs)
    self.speedPoly = None
    self.distancePoly = None
    if self.n_coeffs > 0:
      self.distancePoly = numpy.poly1d(coeffs)
      if self.n_coeffs > 1:
        self.speedPoly = numpy.poly1d(coeffs[:-1])

  def __str__(self):
    '''
    Constructs literal presentation of object.

    Returns
    -------
    string
      Literal presentation of object
    '''
    return "DopplerPoly(coeffs={})".format(self.coeffs)

  def computeDistanceM(self, svTime_s):
    '''
    Computes doppler shift in meters.

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
    poly = self.distancePoly
    if poly is not None:
      return poly(svTime_s)  # self.coeffs[cnt - 1]
    else:
      return 0.

  def computeSpeedMps(self, svTime_s):
    '''
    Computes speed along the vector to satellite in meters per second.

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
    poly = self.speedPoly
    if poly is not None:
      return poly(svTime_s)
    else:
      return 0.

  def computeBatch(self,
                   userTimeAll_s,
                   amplitude,
                   carrierSignal,
                   ifFrequency_hz,
                   message,
                   code,
                   outputConfig):
    '''
    Computes signal samples for the doppler object.

    Parameters
    ----------
    userTimeAll_s : numpy.ndarray(dtype=numpy.float)
      Sample timestamps in seconds
    amplitude : object
      Signal amplitude.
    carrierSignal : object
      Carrier frequency object
    ifFrequency_hz: float
      Intermediate frequency in hertz
    message : object
      Message object for providing access to symbols
    code : object
      PRN code object for providing access to chips
    outputConfig : object
      Output configuration object, containing sampling rate etc.

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

    n_samples = len(userTimeAll_s)

    # Computing doppler coefficients
    freqRatio = -carrierSignal.CENTER_FREQUENCY_HZ / scipy.constants.c

    algMode = 1
    if algMode == 1:
      doppler = numpy.ndarray(n_samples, dtype=self.dtype)
      coeffs = self.coeffs
      n_coeffs = self.n_coeffs
      if n_coeffs > 2:
        # Linear doppler acceleration and further higher-order changes
        # Compute phase accumulator for all orders except 1.
        # This approach saves one vector multiplication.
        d_1 = coeffs[n_coeffs - 2]
        d_0 = coeffs[n_coeffs - 1]
        doppler.fill(coeffs[0])
        for i in range(1, n_coeffs - 2):
          doppler *= userTimeAll_s
          doppler += coeffs[i]
        doppler *= userTimeAll_s
      elif n_coeffs == 2:
        # Constant doppler
        d_1 = coeffs[0]
        d_0 = coeffs[1]
        doppler.fill(0.)
      elif n_coeffs == 1:
        # Constant shift
        d_1 = 0.
        d_0 = coeffs[0]
        doppler.fill(0.)
      else:
        # No phase shift
        d_1 = 0.
        d_0 = 0.
        doppler.fill(0)

      # Phase vector now contains everything except the frequency and d_1
      phaseAll = doppler * freqRatio
      phaseAll += (ifFrequency_hz + d_1 * freqRatio)
      phaseAll *= userTimeAll_s
      phaseAll += d_0 * freqRatio
      phaseAll *= Doppler.TWO_PI
    elif algMode == 2:
      # Simple computation using polynomial function.
      distancePoly = self.distancePoly
      if distancePoly is not None:
        doppler = distancePoly(userTimeAll_s)
      else:
        # No phase shift
        doppler.fill(0)

      # Phase vector now contains everything except the frequency and d_1
      phaseAll = doppler * (freqRatio * Doppler.TWO_PI)
      phaseAll += (Doppler.TWO_PI * ifFrequency_hz) * userTimeAll_s

    # Convert phase to signal value and multiply by amplitude
    signal = scipy.cos(phaseAll)
    amplitude.applyAmplitude(signal, userTimeAll_s)

    # PRN and data index computation
    # Computing doppler coefficients
    chipFreqRatio = -carrierSignal.CODE_CHIP_RATE_HZ / scipy.constants.c

    if algMode == 1:
      chipAll_idx = doppler * chipFreqRatio
      chipAll_idx += (carrierSignal.CODE_CHIP_RATE_HZ + d_1 * chipFreqRatio)
      chipAll_idx *= userTimeAll_s
      chipAll_idx += d_0 * chipFreqRatio
    elif algMode == 2:
      chipAll_idx = doppler * chipFreqRatio
      chipAll_idx += carrierSignal.CODE_CHIP_RATE_HZ * userTimeAll_s

    chips = self.computeDataNChipVector(chipAll_idx,
                                        carrierSignal,
                                        message,
                                        code)
    scipy.multiply(signal, chips, signal)
    return (signal, doppler, chipAll_idx, chips)

def linearDoppler(distance0_m, frequency_hz, doppler0_hz, dopplerChange_hzps):
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
  speed0_mps = -scipy.constants.c / frequency_hz * doppler0_hz
  accel_mps2 = -scipy.constants.c / frequency_hz * dopplerChange_hzps

  return Doppler((accel_mps2, speed0_mps, distance0_m))

def constDoppler(distance0_m, frequency_hz, doppler_hz):
  '''
  Makes an object that corresponds to a constant doppler value.

  Parameters
  ----------
  distance0_m : float
    Initial distance to object.
  frequency_hz : float
    Carrier frequency in Hz.
  doppler_hz : float
    Doppler shift in Hz.

  Returns
  -------
  Doppler
    Object that implements constant speed logic.
  '''
  speed_mps = -scipy.constants.c / frequency_hz * doppler_hz
  return Doppler((speed_mps, distance0_m))

def zeroDoppler(distance_m, frequency_hz):
  '''
  Makes an object that corresponds to zero doppler change.

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
  if distance_m is not None and distance_m != 0.:
    return Doppler((distance_m,))
  else:
    return Doppler(())
