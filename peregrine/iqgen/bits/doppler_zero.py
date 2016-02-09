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
related to generation of signals with zero doppler using `linspace` approach.

"""


import scipy.constants
import peregrine.iqgen.if_iface

class Doppler(object):

  NAME = "ZeroDoppler"

  '''
  Doppler control for non-moving signal source.
  '''
  def __init__(self, distance0_m=0.):
    super(Doppler, self).__init__()
    self.distance0_m = distance0_m
    self.phaseShift = 0.
    self.n_samples = 0l

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

    deltaUserTime_s = float(n_samples) / peregrine.iqgen.if_iface.Chip.SAMPLE_RATE_HZ
    userTimeX_s = userTime0_s + deltaUserTime_s
    self.n_samples += n_samples

    tau0_s = abs(self.distance0_m) / scipy.constants.c  # Initial time difference

    # SV Time Vector
    svTime0_s = userTime0_s - tau0_s
    svTimeX_s = userTimeX_s - tau0_s
    # svTimeAll_s = scipy.linspace(svTime0_s, svTimeX_s, n_samples)

    # valueType = numpy.float32

    phase0_s = scipy.constants.pi * 2. * ifFrequency_hz * userTime0_s
    phaseX_s = scipy.constants.pi * 2. * ifFrequency_hz * userTimeX_s
    signal = scipy.linspace(phase0_s, phaseX_s, n_samples, endpoint=False)

    # carrierAll = amplitude * scipy.sin(signal)
    scipy.sin(signal, signal)
    scipy.multiply(signal, amplitude, signal)

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
