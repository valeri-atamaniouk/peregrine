# Copyright (C) 2016 Swift Navigation Inc.
# Contact: Valeri Atamaniouk <valeri@swiftnav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.bits.low_pass_filter` module contains classes and
functions related to generated signal attenuation.

"""

from scipy.signal.signaltools import lfiltic, lfilter
from scipy.signal import cheby2


class LowPassFilter(object):
  '''
  Chebyshev type 2 low-pass filter.

  The filter simulates receiver lowpass effect:
  - ~2MHz lowpass at 5*1.023MHz sampling pkg load signal
  - b, a = cheby2(5, 40, 3e6/sample_freq)

  For 5.115MHz the coefficents are as follows:
  - b = [0.082680, 0.245072, 0.397168, 0.397168 , 0.245072, 0.082680]
  - a = [1.0000000, -0.3474596, 0.7770501, -0.0737540, 0.0922819, 0.0017200]

  '''

  def __init__(self, outputConfig, frequency_hz=0.):
    '''
    Initialize filter object.

    Parameters
    ----------
    outputConfig : object
      Output configuration parameters object
    frequency_hz : float
      Intermediate frequency
    '''
    super(LowPassFilter, self).__init__()

    b, a = cheby2(5,  # Order of the filter
                  40,  # Minimum attenuation required in the stop band in dB
                  (frequency_hz + 3e6) / outputConfig.SAMPLE_RATE_HZ,
                  btype="lowpass",
                  analog=False,
                  output='ba')

    self.a = a
    self.b = b
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
