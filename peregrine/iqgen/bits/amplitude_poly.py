# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.amplitude_poly` module contains classes and functions
related to implementation of polynomial-based amplitude class.

"""

from peregrine.iqgen.bits.amplitude_base import AmplitudeBase

import numpy

class AmplitudePoly(AmplitudeBase):
  '''
  Amplitude control with polynomial dependency over time.
  '''

  def __init__(self, coeffs, dtype=numpy.float128):
    '''
    Constructs polynomial amplitude control object.

    Parameters
    coeffs : tuple
      Polynomial coefficients
    dtype : object, optional
      Numpy type for sample computations.
    '''
    super(AmplitudePoly, self).__init__(dtype)
    self.coeffs = coeffs

  def __str__(self):
    '''
    Constructs literal presentation of object.

    Returns
    -------
    string
      Literal presentation of object
    '''
    return "AmplitudePoly(c={}, dtype={})".format(self.coeffs, self.dtype)

  def applyAmplitude(self, signal, userTimeAll_s):
    '''
    Applies amplutude modulation to signal

    Parameters
    ----------
    signal : numpy.ndarray
      Array with input samples
    userTimeAll_s : numpy.ndarray
      Array with input samples. This array is modified in place.

    Returns
    -------
    numpy.ndarray
      Array with output samples
    '''
    coeffs = self.coeffs
    n_coeffs = len(coeffs)

    if n_coeffs > 0:
      ampAll = numpy.ndarray(len(signal), dtype=self.dtype)
      ampAll.fill(coeffs[0])
      for i in range(1, n_coeffs):
        ampAll *= userTimeAll_s
        ampAll += coeffs[i]
      return numpy.multiply(signal, ampAll, out=signal)
    else:
      return signal
