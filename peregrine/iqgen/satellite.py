# coding=utf-8
# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
from peregrine.iqgen.bits.amplitude_poly import AmplitudePoly
"""
The :mod:`peregrine.iqgen.satellite` module contains classes and functions
related to satellite configuration.

"""
import peregrine.iqgen.bits.signals as signals

from peregrine.iqgen.bits.doppler_poly import Doppler
from peregrine.iqgen.bits.message_const import Message
from peregrine.iqgen.bits.prn_gps_l1ca import PrnCode as GPS_L1CA_Code
from peregrine.iqgen.bits.prn_gps_l2c import PrnCode as GPS_L2C_Code

import numpy

class SV(object):
  '''
  Satellite object.

  Satellite object combines speed/position computation and data generation for
  all supported bands.
  '''

  def __init__(self, svName):
    '''
    Constructor.

    Parameters
    ----------
    svNo : string
      Satellite name
    '''
    super(SV, self).__init__()
    self.svName = svName
    self.doppler = Doppler(())

  def getSvName(self):
    '''
    Returns satellite name.

    Returns
    -------
    string
      Satellite name
    '''
    return self.svName

DEFAULT_MESSAGE = Message(1)

class GPS_SV(SV):
  '''
  GPS satellite object.
  '''

  def __init__(self, prnNo):
    '''
    Constructs satellite object

    Parameters
    ----------
    prnNo : int
      GPS satellite number for selecting PRN.
    '''
    super(GPS_SV, self).__init__("GPS {}".format(prnNo))
    self.prn = prnNo
    self.l1caCode = GPS_L1CA_Code(prnNo)
    self.l2cCode = GPS_L2C_Code(prnNo)
    self.l1caEnabled = False
    self.l2cEnabled = False
    self.l1caMessage = DEFAULT_MESSAGE
    self.l2cMessage = DEFAULT_MESSAGE
    self.time0S = 0.
    self.pr0M = 0.
    self.amplitude = AmplitudePoly(())
    self.phaseShift = 0.

  def setAmplitude(self, amplitude):
    '''
    Changes amplitude

    Parameters
    ----------
    amplitude : float
      amplitude value for signal generation
    '''
    self.amplitude = amplitude

  def setL1CAEnabled(self, enableFlag):
    '''
    Enables or disable GPS L1 C/A sample generation

    Parameters
    ----------
    enableFlag : boolean
      Flag to enable (True) or disable (False) GPS L1 C/A samples
    '''
    self.l1caEnabled = enableFlag

  def setL2CEnabled(self, enableFlag):
    '''
    Enables or disable GPS L2 C sample generation

    Parameters
    ----------
    enableFlag : boolean
      Flag to enable (True) or disable (False) GPS L2 C samples
    '''
    self.l2cEnabled = enableFlag

  def setL1CAMessage(self, message):
    '''
    Configures data source for L1 C/A signal.

    Parameters
    ----------
    message : object
      Message object that will provide symbols for L1 C/A signal.
    '''
    self.l1caMessage = message

  def setL2CMessage(self, message):
    '''
    Configures data source for L2 C signal.

    Parameters
    ----------
    message : object
      Message object that will provide symbols for L2 C signal.
    '''
    self.l2cMessage = message

  def getBatchSignals(self, userTimeAll_s, samples, outputConfig):
    '''
    Generates signal samples.

    Parameters
    ----------
    userTimeAll_s : numpy.ndarray(n_samples, dtype=numpy.float64)
      Vector of observer's timestamps in seconds for the interval start.
    samples : numpy.ndarray((4, n_samples))
      Array to which samples are added.
    outputConfig : object
      Output configuration object.

    Returns
    -------
    list
      Debug information
    '''
    result = [[], [], [], []]
    if (self.l1caEnabled):
      intermediateFrequency_hz = outputConfig.GPS.L1.INTERMEDIATE_FREQUENCY_HZ
      frequencyIndex = outputConfig.GPS.L1.INDEX
      values = self.doppler.computeBatch(userTimeAll_s,
                                         self.amplitude,
                                         signals.GPS.L1CA,
                                         intermediateFrequency_hz,
                                         self.l1caMessage,
                                         self.l1caCode,
                                         outputConfig)
      numpy.add(samples[frequencyIndex], values[0], out=samples[frequencyIndex])
      result[0].append(values[0])
      result[1].append(values[1])
      result[2].append(values[2])
      result[3].append(values[3])
    if (self.l2cEnabled):
      intermediateFrequency_hz = outputConfig.GPS.L2.INTERMEDIATE_FREQUENCY_HZ
      frequencyIndex = outputConfig.GPS.L2.INDEX
      values = self.doppler.computeBatch(userTimeAll_s,
                                         self.amplitude,
                                         signals.GPS.L2C,
                                         intermediateFrequency_hz,
                                         self.l2cMessage,
                                         self.l2cCode,
                                         outputConfig)
      numpy.add(samples[frequencyIndex], values[0], out=samples[frequencyIndex])
      result[0].append(values[0])
      result[1].append(values[1])
      result[2].append(values[2])
      result[3].append(values[3])
    return result

  def isBandEnabled(self, bandIndex, outputConfig):
    '''
    Checks if particular band is supported and enabled.

    Parameters
    ----------
    bandIndex : int
      Signal band index
    outputConfig : object
      Output configuration

    Returns:
    bool
      True, if the band is supported and enabled; False otherwise.
    '''
    result = None
    if bandIndex == outputConfig.GPS.L1.INDEX:
      result = self.l1caEnabled
    elif bandIndex == outputConfig.GPS.L2.INDEX:
      result = self.l2cEnabled
    else:
      result = False
    return result
