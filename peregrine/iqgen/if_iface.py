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

class Chip(object):
  '''
  Parameters for band data generation.

  Attributes
  ----------
  SAMPLE_RATE_HZ : float
    Sample rate in hertz for data generation.
  SAMPLE_BATCH_SIZE : int
    Size of the sample batch in samples.
  '''
  if (LOW_RATE):
    SAMPLE_RATE_HZ = 25e5  # 4.096e6  # Sample rate in Hz for use with generator
  else:
    SAMPLE_RATE_HZ = 25e6

  SAMPLE_BATCH_SIZE = 100000

  class GPS(object):
    '''
    Parameters for GPS bands data generation.
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
      if (LOW_RATE):
        INTERMEDIATE_FREQUENCY_HZ = 14.5e5  # 2.048e+6  # 2.048 MHz
      else:
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
      if (LOW_RATE):
        INTERMEDIATE_FREQUENCY_HZ = 2.048e+6
      else:
        INTERMEDIATE_FREQUENCY_HZ = 7.5e+6
      INDEX = 1

  class GLONASS(object):
    '''
    Parameters for GLONASS bands data generation.
    '''
    pass

