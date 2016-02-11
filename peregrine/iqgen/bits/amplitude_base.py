# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.amplitude_base` module contains classes and functions
related to base implementation of amplitude class.

"""

import numpy

class AmplitudeBase(object):
  '''
  Amplitude control for a signal source.
  '''

  NAME = "Amptlitude"

  def __init__(self, dtype=numpy.float128):
    '''
    Constructs doppler base object for movement control.

    Parameters
    dtype : object, optional
      Numpy type for sample computations.
    '''
    super(AmplitudeBase, self).__init__()
    self.dtype = dtype

  def applyAmplitude(self, signal, userTimeAll_s):
    '''
    Applies amplutude modulation to signal

    Parameters
    ----------
    signal : numpy.ndarray
      Array with input samples. This array is modified in place.
    userTimeAll_s : numpy.ndarray
      Sample time array in seconds

    Returns
    -------
    numpy.ndarray
      Array with output samples
    '''
    raise NotImplementedError()

