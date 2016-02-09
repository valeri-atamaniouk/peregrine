# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.doppler_const` module contains classes and functions
related to generation of zero-doppler signals.

"""

import scipy.constants
from peregrine.iqgen import if_iface

class Doppler(object):

  NAME = "ConstDoppler"

  '''
  Doppler control for a signal source that moves with a constant speed.
  '''
  def __init__(self, distance0_m, speed_mps):
    '''
    Constructs doppler control object for linear movement.

    Parameters
    ----------
    distance0_m : float
      Initial distance to satellite at time 0 in meters for computing signal
      phase and delay.
    speed_mps : float
      Speed of satellite in meters per second.
    '''
    super(Doppler, self).__init__()
    self.distance0_m = distance0_m
    self.speed_mps = speed_mps
    self.c1 = self.distance0_m / (scipy.constants.c + self.speed_mps)
    self.c2 = self.speed_mps / (scipy.constants.c + self.speed_mps)

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
    return self.distance0_m + self.speed_mps * svTime_s

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
    return self.speed_mps

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

    tau_s = self.c1 + self.c2 * userTime_s
    svTime_s = userTime_s - tau_s

    return svTime_s

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
    deltaUserTime_s = float(n_samples) / if_iface.Chip.SAMPLE_RATE_HZ
    userTimeX_s = userTime0_s + deltaUserTime_s
    tau0_s = self.c1 + self.c2 * userTime0_s  # Initial time difference
    tauX_s = self.c1 + self.c2 * userTimeX_s  # Final time difference

    svTime0_s = userTime0_s - tau0_s  # Start time
    svTimeX_s = userTimeX_s - tauX_s  # End time

    # Compute initial and final signal phases and then phase space
    doppler_hz = -carrierSignal.CENTER_FREQUENCY_HZ / scipy.constants.c * self.speed_mps
    phase0_s = scipy.constants.pi * 2. * (ifFrequency_hz + doppler_hz) * userTime0_s  # svTime0_s
    phaseX_s = scipy.constants.pi * 2. * (ifFrequency_hz + doppler_hz) * userTimeX_s  # svTimeX_s
    signal = scipy.linspace(phase0_s, phaseX_s, n_samples, endpoint=False)

    # Convert phase to signal value and multiply by amplitude
    scipy.sin(signal, signal)
    scipy.multiply(signal, amplitude, signal)

    # PRN and data index computation
    chip0_idx = svTime0_s * carrierSignal.CODE_CHIP_RATE_HZ
    chipX_idx = svTimeX_s * carrierSignal.CODE_CHIP_RATE_HZ
    chipAll_idx = scipy.linspace(chip0_idx, chipX_idx, n_samples, endpoint=False)

    def dataChip(idx):
      chipIdx = long(idx)
      dataIdx = chipIdx / carrierSignal.CHIP_TO_SYMBOL_DIVIDER
      x = message.getBit(dataIdx) * code.getCodeBit(chipIdx)
      return x

    vdata = scipy.vectorize(dataChip)
    chips = vdata(chipAll_idx)

    scipy.multiply(signal, chips, signal)
    return (signal, userTimeX_s, chipAll_idx, chips)

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
  if (distance0_m < 0):
    speed_mps = -speed_mps
  return Doppler(distance0_m, speed_mps)
