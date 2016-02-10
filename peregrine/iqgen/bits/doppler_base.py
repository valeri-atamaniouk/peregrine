# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
import numpy

"""
The :mod:`peregrine.iqgen.doppler_base` module contains classes and functions
related to base implementation of doppler class.

"""

import scipy.constants

class DopplerBase(object):

  NAME = "Doppler"

  '''
  Doppler control for a signal source that moves with a constant speed.
  '''
  def __init__(self, dtype=numpy.float128):
    '''
    Constructs doppler base object for movement control.

    Parameters
    dtype : object, optional
      Numpy type for sample computations.
    '''
    super(DopplerBase, self).__init__()
    self.dtype = dtype

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
    raise NotImplementedError()

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
    raise NotImplementedError()

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
    raise NotImplementedError()

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
    raise NotImplementedError()

  @staticmethod
  def computeDeltaUserTimeS(userTime0_s, n_samples, outputConfig):
    deltaUserTime_s = float(n_samples) / outputConfig.SAMPLE_RATE_HZ
    return deltaUserTime_s

  @staticmethod
  def computeDopplerHz(frequency_hz, speed_mps):
    doppler_hz = -frequency_hz / scipy.constants.c * speed_mps
    return doppler_hz

  def computeDataNChipVector(self, chipAll_idx, carrierSignal, message, code):
    '''
    Helper for computing vector that combines data and code chips.

    Parameters
    ----------
    chipAll_idx : ndarray
      vector of chip phases
    carrierSignal : object
      Signal description object
    messge : object
      Data bits source
    code : objects
      Code chips source

    Returns
    -------
    ndarray
      Array of code chips multiplied with data bits
    '''

    def dataChip(idx):
      chipIdx = long(idx)
      dataIdx = chipIdx / carrierSignal.CHIP_TO_SYMBOL_DIVIDER
      x = message.getBit(dataIdx) * code.getCodeBit(chipIdx)
      return x

    vdata = scipy.vectorize(dataChip)
    return vdata(chipAll_idx)

