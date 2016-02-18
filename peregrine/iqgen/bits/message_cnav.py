'''
Copyright (C) 2016 Swift Navigation Inc.

This source is subject to the license found in the file 'LICENSE' which must
be be distributed together with this source. All other rights reserved.

THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

'''

'''
'''

import numpy
from swiftnav.cnav_msg import CNavRawMsg

G1 = 0171  # generator polinomial for p1
G2 = 0133  # generator polinomial for p2


def generate27Vector(g1, g2):
  '''
  Helper method for convolution encoder lookup table generation.

  Parameters
  ----------
  g1 : int
    First polynomial coefficient
  g2 : int
    Second polynomial coefficient

  Results
  -------
  numpy.ndvector(shape=(128,2),dtype=numpy.uint8)
    Lookup matrix for convolution encoder
  '''

  def parity6(value):
    '''
    Helper for computing parity of 6-bit value.

    Parameters
    ----------
    value : int
      6-bit integer value

    Results
    -------
    int
      Parity bit: 0 or 1.
    '''
    return (0x6996 >> ((value ^ (value >> 4)) & 15)) & 1

  vectorG = numpy.ndarray((128, 2), dtype=numpy.uint8)
  for i in range(128):
    vectorG[i][0] = parity6(i & g1)
    vectorG[i][1] = parity6(i & g2)

  return vectorG


class ConvEncoder27(object):
  '''
  Convolution encoder class.

  Standard 2-7 convolution encoder implementation.
  '''

  DEFAULT_VECTOR_G = generate27Vector(G1, G2)

  def __init__(self, g1=G1, g2=G2, state=0):
    self.g1 = g1
    self.g2 = g2
    self.state = state
    vectorG = ConvEncoder27.DEFAULT_VECTOR_G if g1 == G1 and g2 == G2 \
        else generate27Vector(g1, g2)
    self.vectorG = vectorG

  def encode(self, bitArray):
    '''
    Encodes source bit array.

    This method updates the encoder state during processing.

    Parameters
    ----------
    bitArray : array-like
      Array of bit values. Can be integers or booleans.
    Returns
    -------
    numpy.ndarray(shape(len(bitArray)), dtype=numpy.uint8)
      Encoded output
    '''
    result = numpy.ndarray((len(bitArray) * 2), dtype=numpy.uint8)
    state = self.state
    dstIndex = 0
    vectorG = self.vectorG

    for srcBit in bitArray:
      state = (srcBit << 6) | (state >> 1)
      result[dstIndex:dstIndex + 2] = vectorG[state]
      dstIndex += 2

    self.state = state
    return result


class Message(object):
  '''
  Message that is a block of bits
  '''

  def __init__(self, n_msg=10, n_prefixBits=50):
    '''
    Constructs message object.

    Parameters
    ----------
    n_msg : int, optional
      Number of messages to generate for output
    n_prefixBits : int, optional
      Number of bits to prepend before the first message
    '''
    super(Message, self).__init__()

    self.encoder = ConvEncoder27()
    self.n_msg = n_msg
    self.n_prefixBits = n_prefixBits
    prefix_len = n_prefixBits * 2
    self.symbolData = numpy.ndarray(n_msg * 600 + prefix_len,
                                    dtype=numpy.uint8)
    self.symbolData.fill(0)

    prefixBits = numpy.ndarray(self.n_prefixBits, dtype=numpy.uint8)
    prefixBits.fill(0)
    prefixBits[0::2] = 1
    self.symbolData[0:prefix_len] = self.encoder.encode(prefixBits)

    for i in range(prefix_len, self.n_msg * 600 + prefix_len, 600):
      cnav_msg = CNavRawMsg.generate()
      # print "CNAV=", cnav_msg
      encoded = self.encoder.encode(cnav_msg)
      # print "ENC=", encoded
      self.symbolData[i:i + 600] = encoded
    self.messageLen = len(self.symbolData)

    print self.symbolData

  def getDataBits(self, dataAll_idx):
    '''
    Generates vector of data bits corresponding to input index

    Parameters
    ----------
    dataAll_idx : numpy.ndarray(dtype=numpy.int64)
      Vector of bit indexes

    Returns
    -------
    numpy.ndarray(dtype=numpy.uint8)
      Vector of data bits
    '''
    # numpy.take degrades performance a lot over time.
    # return numpy.take(self.symbolData, dataAll_idx , mode='wrap')
    return self.symbolData[dataAll_idx % self.messageLen]


def __main():
  #   src = numpy.ndarray(128 * 1024, dtype=numpy.uint8)
  #   src[1::2] = 0
  #   src[0::2] = 1
  #   t2 = time.time()
  #   enc3 = ConvEncoder27().encode(src)
  #   t3 = time.time()
  # cnav_msg = CNavRawMsg()
  print CNavRawMsg.generate()

  print "Done"

if __name__ == '__main__':
  __main()
