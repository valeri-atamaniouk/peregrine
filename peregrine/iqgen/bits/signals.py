# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.signals` module contains classes and functions
related to common satellite signal definitions and operations.

"""

from peregrine.iqgen.if_iface import Chip

import scipy.constants

class GPS:
  '''
  GPS signal parameters and utilities.
  '''

  @staticmethod
  def _calcDopplerShiftHz(frequency_hz, distance_m, velocity_mps):
    '''
    Utility to compute doppler shift from ditance and velocity for a band
    frequency.

    Parameters
    ----------
    frequency_hz : float
      Band frequency in hertz
    distance_m : float
      Distance to satellite in meters
    velocity_m : float
      Satellite velocity in meters per second.

    Return
    ------
    float
      Doppler shift in hertz
    '''
    if (distance_m > 0.):
      doppler_hz = -velocity_mps * frequency_hz / scipy.constants.c
    elif (distance_m < 0.):
      doppler_hz = velocity_mps * frequency_hz / scipy.constants.c
    else:
      doppler_hz = 0.
    return doppler_hz

  class L1CA:
    '''
    GPS L1 C/A parameters and utilities.
    '''
    INDEX = Chip.GPS.L1.INDEX
    SYMBOL_RATE_HZ = 50
    CENTER_FREQUENCY_HZ = 1575.42e6
    CODE_CHIP_RATE_HZ = 1023000

    @staticmethod
    def calcDopplerShiftHz(distance_m, velocity_mps):
      '''
      Converts relative speed into doppler value for GPS L1 C/A band.

      Parameters
      ----------
      distance_m : float
        Distance in meters
      velocity_mps : float
        Relative speed in meters per second.

      Returns
      -------
      float
        Doppler shift in Hz.
      '''
      return GPS._calcDopplerShiftHz(GPS.L1CA.CENTER_FREQUENCY_HZ, distance_m, velocity_mps)

    @staticmethod
    def getSymbolIndex(svTime_s):
      '''
      Computes symbol index.

      Parameters
      ----------
      svTime_s : float
        SV time in seconds

      Returns
      -------
      long
        Symbol index
      '''
      return long(svTime_s * GPS.L1CA.SYMBOL_RATE_HZ)

    @staticmethod
    def getCodeChipIndex(svTime_s):
      '''
      Computes code chip index.

      Parameters
      ----------
      svTime_s : float
        SV time in seconds

      Returns
      -------
      long
        Code chip index
      '''
      return long(svTime_s * GPS.L1CA.CODE_CHIP_RATE_HZ)

  class L2C:
    '''
    GPS L2 C parameters and utilities.
    '''

    INDEX = Chip.GPS.L2.INDEX
    SYMBOL_RATE_HZ = 50
    CENTER_FREQUENCY_HZ = 1227.60e6
    CODE_CHIP_RATE_HZ = 1023000

    @staticmethod
    def calcDopplerShiftHz(distance_m, velocity_mps):
      '''
      Converts relative speed into doppler value for GPS L2 C band.

      Parameters
      ----------
      distance_m : float
        Distance in meters
      velocity_mps : float
        Relative speed in meters per second.

      Returns
      -------
      float
        Doppler shift in Hz.
      '''
      return GPS._calcDopplerShiftHz(GPS.L2C.CENTER_FREQUENCY_HZ, distance_m, velocity_mps)

    @staticmethod
    def getSymbolIndex(svTime_s):
      '''
      Computes symbol index.

      Parameters
      ----------
      svTime_s : float
        SV time in seconds

      Returns
      -------
      long
        Symbol index
      '''
      return long(svTime_s * GPS.L2C.SYMBOL_RATE_HZ)

    @staticmethod
    def getCodeChipIndex(svTime_s):
      '''
      Computes code chip index.

      Parameters
      ----------
      svTime_s : float
        SV time in seconds

      Returns
      -------
      long
        Code chip index
      '''
      return long(svTime_s * GPS.L2C.CODE_CHIP_RATE_HZ)
