# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.doppler_zero2` module contains classes and functions
related to generation of signals with zero doppler using sample step approach.

"""


import scipy.constants
import peregrine.iqgen.if_iface
import numpy

class Doppler(object):
  '''
  Doppler control for non-moving signal source.

  This object uses time and phase delta increments instead of numpy.linspace
  sampling.
  '''
  def __init__(self, distance0m):
    super(Doppler, self).__init__()
    self.distance0_m = distance0m

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
    return self.distance0_m

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
    return 0.

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
    return  userTime_s - abs(self.distance0_m) / scipy.constants.c

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

    # Phase step for 1 sample
    phaseStep = 2. * scipy.constants.pi * ifFrequency_hz / peregrine.iqgen.if_iface.Chip.SAMPLE_RATE_HZ

    signal = scipy.ndarray(n_samples, dtype=numpy.float64)
    signal.fill(phaseStep)
    phaseOffset = 2. * scipy.constants.pi * ifFrequency_hz * userTime0_s
    signal[0] = phaseOffset
    numpy.cumsum(signal, dtype=numpy.float64, out=signal)

    numpy.sin(signal, dtype=numpy.float64, out=signal)
    numpy.multiply(signal, amplitude, dtype=numpy.float64, out=signal)

    chip1_s = 1023000. / peregrine.iqgen.if_iface.Chip.SAMPLE_RATE_HZ
    chipAll_idx = numpy.ndarray(n_samples, dtype=numpy.float64);
    chipAll_idx.fill(chip1_s)
    chipOffset = 1023000. * userTime0_s
    chipAll_idx[0] = chipOffset
    numpy.cumsum(chipAll_idx, dtype=numpy.float64, out=chipAll_idx)
    self.chipOffset = chipAll_idx[len(chipAll_idx) - 1]

    def dataChip(idx):
      chipIdx = long(idx)
      dataIdx = chipIdx / (1023 * 20)
      x = message.getBit(dataIdx) * code.getCodeBit(chipIdx)
      return x

    vdata = scipy.vectorize(dataChip)
    chips = vdata(chipAll_idx)

    numpy.multiply(signal, chips, out=signal)

    return (signal, userTime0_s, chipAll_idx, chips)
