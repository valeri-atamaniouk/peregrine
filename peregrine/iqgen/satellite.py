# coding=utf-8
# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.satellite` module contains classes and functions
related to satellite configuration.

"""
import peregrine.iqgen.bits.signals as signals

from peregrine.iqgen.if_iface import Chip

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
    self.amplitude = 1.
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

  def getBatchSignals(self, time0_s, n_samples, samples):
    '''
    Generates signal samples.

    Parameters
    ----------
    time0_s : float
      Observer's time in seconds for the interval start.
    n_samples : int
      Number of samples to generate.
    samples : numpy.ndarray((4, n_samples))
      Array to which samples are added.
    '''
    if (self.l1caEnabled):
      values = self.doppler.computeBatch(time0_s,
                                         n_samples,
                                         self.amplitude,
                                         signals.GPS.L1CA.CENTER_FREQUENCY_HZ,
                                         Chip.GPS.L1.INTERMEDIATE_FREQUENCY_HZ,
                                         self.l1caMessage,
                                         self.l1caCode)
      numpy.add(samples[Chip.GPS.L1.INDEX], values[0], out=samples[Chip.GPS.L1.INDEX])
    if (self.l2cEnabled):
      values = self.doppler.computeBatch(time0_s,
                                         n_samples,
                                         self.amplitude,
                                         signals.GPS.L2C.CENTER_FREQUENCY_HZ,
                                         Chip.GPS.L2.INTERMEDIATE_FREQUENCY_HZ,
                                         self.l2cMessage,
                                         self.l2cCode)
      numpy.add(samples[Chip.GPS.L2.INDEX], values[0], out=samples[Chip.GPS.L2.INDEX])
    return values
