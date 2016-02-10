# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.if_iface` module contains classes and functions
related to radio interface parameters

"""

LOW_RATE = True

class LowRateConfig(object):
  '''
  Output control configuration for quick tests.

  Attributes
  ----------
  NAME : string
    Configuration name
  SAMPLE_RATE_HZ : float
    Sample rate in hertz for data generation.
  SAMPLE_BATCH_SIZE : int
    Size of the sample batch in samples.
  GPS : object
    GPS band information
  '''
  NAME = "Low rate configuration for fast tests"
  SAMPLE_RATE_HZ = 24.84375e5
  SAMPLE_BATCH_SIZE = 100000

  class GPS(object):
    '''
    Parameters for GPS bands data generation.

    Attributes
    ----------
    L1 : object
      GPS L1 band information
    L2 : object
      GPS L2 band information
    '''
    class L1(object):
      '''
      Parameters for GPS L1 C/A band data generation.

      Attributes
      ----------
      INTERMEDIATE_FREQUENCY_HZ : float
        Intermediate frequency in hertz for sample construction.
      INDEX : int
        Band index
      '''
      INTERMEDIATE_FREQUENCY_HZ = 14.5e5  # 2.048e+6  # 2.048 MHz
      INDEX = 0

    class L2(object):
      '''
      Parameters for GPS L2 C band data generation.

      Attributes
      ----------
      INTERMEDIATE_FREQUENCY_HZ : float
        Intermediate frequency in hertz for sample construction.
      INDEX : int
        Band index
      '''
      INTERMEDIATE_FREQUENCY_HZ = 2.048e+6
      INDEX = 1

class NormalRateConfig(object):
  '''
  Output control configuration for normal tests.

  Attributes
  ----------
  NAME : string
    Configuration name
  SAMPLE_RATE_HZ : float
    Sample rate in hertz for data generation.
  SAMPLE_BATCH_SIZE : int
    Size of the sample batch in samples.
  GPS : object
    GPS band information
  '''
  NAME = "Normal rate configuration equivalent to decimated data output"
  SAMPLE_RATE_HZ = 24.84375e6
  SAMPLE_BATCH_SIZE = 100000

  class GPS(object):
    '''
    Parameters for GPS bands data generation.

    Attributes
    ----------
    L1 : object
      GPS L1 band information
    L2 : object
      GPS L2 band information
    '''
    class L1(object):
      '''
      Parameters for GPS L1 C/A band data generation.

      Attributes
      ----------
      INTERMEDIATE_FREQUENCY_HZ : float
        Intermediate frequency in hertz for sample construction.
      INDEX : int
        Band index
      '''
      INTERMEDIATE_FREQUENCY_HZ = 14.5e+6
      INDEX = 0

    class L2(object):
      '''
      Parameters for GPS L2 C band data generation.

      Attributes
      ----------
      INTERMEDIATE_FREQUENCY_HZ : float
        Intermediate frequency in hertz for sample construction.
      INDEX : int
        Band index
      '''
      INTERMEDIATE_FREQUENCY_HZ = 7.5e+6
      INDEX = 1

class HighRateConfig(object):
  '''
  Output control configuration for high data rate tests.

  Attributes
  ----------
  NAME : string
    Configuration name
  SAMPLE_RATE_HZ : float
    Sample rate in hertz for data generation.
  SAMPLE_BATCH_SIZE : int
    Size of the sample batch in samples.
  GPS : object
    GPS band information
  '''
  NAME = "High rate configuration equivalent to full rate data output"
  SAMPLE_RATE_HZ = 99.375e6
  SAMPLE_BATCH_SIZE = 100000

  GPS = NormalRateConfig.GPS

