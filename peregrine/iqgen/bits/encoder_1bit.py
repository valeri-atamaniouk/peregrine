# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.encoder_1bit` module contains classes and functions
related to generating single bit signal output.

"""

from peregrine.iqgen.bits.encoder_base import Encoder

class BandBitEncoder(Encoder):
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
    super(BandBitEncoder, self).__init__()
    self.bandIndex = bandIndex

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

    self.ensureExtraCapacity(n_samples)

    self.bits[self.n_bits:self.n_bits + n_samples] = [self.getBit(x) for x in band_samples]
    self.n_bits += n_samples

    if (self.n_bits >= Encoder.BLOCK_SIZE):
      return self.encodeValues()

    return Encoder.EMPTY_RESULT

  def getBit(self, value):
    '''
    Converts a signed sample value into a bit value.

    Parameters
    ----------
    value : float
      Sample value

    Returns
    -------
    int
      Bit value: 0 if the value is positive, 1 otherwise
    '''
    if (value > 0):
      return 0
    else:
      return 1
