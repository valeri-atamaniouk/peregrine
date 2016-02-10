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
The :mod:`peregrine.iqgen.encoder_2bits` module contains classes and functions
related to generating two bits signal output.

"""

from peregrine.iqgen.bits.encoder_base import Encoder

class BandTwoBitsEncoder(Encoder):
  '''
  Base class for single bit encoding.
  '''

  def __init__(self, bandIndex):
    '''
    Initializes encoder object.

    Parameters
    ----------
    bandIndex : int
      Index of the band in the generated sample matrix.
    '''
    super(BandTwoBitsEncoder, self).__init__()
    self.bandIndex = bandIndex

  @staticmethod
  def convertBand(band_samples):
    # Signal power is a square of the amplitude
    power = numpy.square(band_samples)
    totalPower = numpy.sum(power)

    # Build histrogram to find 60% power
    hist, edges = numpy.histogram(power,
                                  bins=10,
                                  density=True)
    acc = 0.
    totalPowerLimit = totalPower * 0.6
    powerLimit = 0.
    for i in range(10):
      # Approximate power of samples in the bin
      entryPower = hist[i] * (edges[i] + edges[i + 1]) / 2.
      if acc + entryPower > totalPowerLimit:
        powerLimit = edges[i]
        break
      else:
        acc += entryPower

    # Signal sign
    signs = numpy.greater(numpy.sign(band_samples), 0)
    amps = numpy.greater_equal(power, powerLimit)

    return signs, amps

  def addSamples(self, sample_array):
    '''
    Extracts samples of the supported band and coverts them into bit stream.

    Parameters
    ----------
    sample_array : numpy.ndarray((4, N))
      Sample vectors ordered by band index.

    Returns
    -------
    ndarray
      Array of type uint8 containing the encoded data.
    '''
    band_samples = sample_array[self.bandIndex]
    n_samples = len(band_samples)

    # Signal signs and amplitude
    signs, amps = self.convertBand(band_samples)

    n_bits = self.n_bits
    bits = self.bits
    if len(bits) < n_bits + 2 * n_samples:
      bits.resize(n_bits + 2 * n_samples)

    for i in range(n_samples):
      if signs[i]:
        sign_bit = 1
      else:
        sign_bit = 0
      if amps[i]:
        amp_bit = 1
      else:
        amp_bit = 0
      bits[n_bits] = sign_bit
      bits[n_bits + 1] = amp_bit
      n_bits += 2

    self.n_bits = n_bits

    if (self.n_bits >= Encoder.BLOCK_SIZE):
      return self.encodeValues()
    else:
      return Encoder.EMPTY_RESULT
