# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.encoder_gps` module contains classes and functions
related to generating GPS signal output.

"""

from peregrine.iqgen.bits.encoder_base import Encoder
from peregrine.iqgen.bits.encoder_1bit import BandBitEncoder
from peregrine.iqgen.bits.encoder_2bits import BandTwoBitsEncoder
from peregrine.iqgen.if_iface import Chip

GPS_L1_INDEX = Chip.GPS.L1.INDEX
GPS_L2_INDEX = Chip.GPS.L2.INDEX

class GPSL1BitEncoder(BandBitEncoder):
  '''
  Generic single bit encoder for GPS L1 C/A signal
  '''
  def __init__(self):
    '''
    Constructs GPS L1 C/A band single bit encoder object.
    '''
    super(GPSL1BitEncoder, self).__init__(GPS_L1_INDEX)

class GPSL2BitEncoder(BandBitEncoder):
  '''
  Generic single bit encoder for GPS L2 Civil signal
  '''
  def __init__(self):
    '''
    Constructs GPS L2 C band single bit encoder object.
    '''
    super(GPSL2BitEncoder, self).__init__(GPS_L2_INDEX)

class GPSL1L2BitEncoder(Encoder):
  '''
  Generic single bit encoder for GPS L1 C/A and L2 Civil signals
  '''
  def __init__(self):
    '''
    Constructs GPS L1 C/A and L2 C dual band single bit encoder object.
    '''
    super(GPSL1L2BitEncoder, self).__init__()

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
    band1_samples = sample_array[GPS_L1_INDEX]
    band2_samples = sample_array[GPS_L2_INDEX]
    n_samples = len(band1_samples)
    def combined():
      try:
        it1 = iter(band1_samples)
        it2 = iter(band2_samples)
        yield next(it1)
        yield next(it2)
      except StopIteration:
        return

    self.bits[self.n_bits:self.n_bits + n_samples * 2 ] = [self.getBit(x) for x in combined()]
    self.n_bits += n_samples + n_samples

    if (self.n_bits >= Encoder.BLOCK_SIZE):
      return self.encodeValues()
    else:
      return Encoder.EMPTY_RESULT

class GPSL1TwoBitsEncoder(BandTwoBitsEncoder):
  '''
  Generic single bit encoder for GPS L1 C/A signal
  '''
  def __init__(self):
    '''
    Constructs GPS L1 C/A band single bit encoder object.
    '''
    super(GPSL1TwoBitsEncoder, self).__init__(GPS_L1_INDEX)

class GPSL2TwoBitsEncoder(BandTwoBitsEncoder):
  '''
  Generic single bit encoder for GPS L2 Civil signal
  '''
  def __init__(self):
    '''
    Constructs GPS L2 C band single bit encoder object.
    '''
    super(GPSL2TwoBitsEncoder, self).__init__(GPS_L2_INDEX)

class GPSL1L2TwoBitsEncoder(Encoder):
  '''
  Generic single bit encoder for GPS L1 C/A and L2 Civil signals
  '''
  def __init__(self):
    '''
    Constructs GPS L1 C/A and L2 C dual band single bit encoder object.
    '''
    super(GPSL1L2TwoBitsEncoder, self).__init__()

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
    band1_samples = sample_array[GPS_L1_INDEX]
    band2_samples = sample_array[GPS_L2_INDEX]
    n_samples = len(band1_samples)

    # Signal signs and amplitude
    signs1, amps1 = BandTwoBitsEncoder.convertBand(band1_samples)
    signs2, amps2 = BandTwoBitsEncoder.convertBand(band2_samples)

    self.ensureExtraCapacity(n_samples * 4)

    n_bits = self.n_bits
    bits = self.bits

    for i in range(n_samples):
      if signs1[i]:
        sign_bit1 = 1
      else:
        sign_bit1 = 0
      if amps1[i]:
        amp_bit1 = 1
      else:
        amp_bit1 = 0
      if signs2[i]:
        sign_bit2 = 1
      else:
        sign_bit2 = 0
      if amps2[i]:
        amp_bit2 = 1
      else:
        amp_bit2 = 0
      bits[n_bits + 0] = sign_bit1
      bits[n_bits + 1] = amp_bit1
      bits[n_bits + 2] = sign_bit2
      bits[n_bits + 3] = amp_bit2
      n_bits += 4

    self.n_bits = n_bits

    if (self.n_bits >= Encoder.BLOCK_SIZE):
      return self.encodeValues()
    else:
      return Encoder.EMPTY_RESULT
