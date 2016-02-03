# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.low_pass_filter` module contains classes and functions
related to generated signal attenuation.

"""

from scipy.signal.signaltools import lfiltic, lfilter

class LowPassFilter(object):
  '''
  Chebyshev type 2 low-pass filter.
  '''

  b = [0.082680, 0.245072 , 0.397168 , 0.397168  , 0.245072 , 0.082680];
  a = [1.0000000, -0.3474596 , 0.7770501 , -0.0737540 , 0.0922819 , 0.0017200];

  def __init__(self):
    '''
    Initializez filter object.
    '''
    super(LowPassFilter, self).__init__()
    self.zi = lfiltic(self.b, self.a, [])

  def filter(self, data):
    '''
    Performs noise reduction using Chebyshev type 2 IIR filter.

    Parameters
    ----------
    data : array-like
      Data samples before LPF processing

    Returns
    -------
    array-like
      Data samples after LPF processing
    '''
    data_out, zo = lfilter(self.b, self.a, data, zi=self.zi)
    self.zi = zo
    return data_out
