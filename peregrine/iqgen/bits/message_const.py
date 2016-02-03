# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.message_const` module contains classes and functions
related to non-changing symbol contents.

"""

class Message(object):
  '''
  Message consisting of same bits
  '''
  def __init__(self, bitValue):
    '''
    Initializes object.

    Parameters
    ----------
    bitValue : int
      Value for the bits. 1 for 0 bits, -1 for 1 bits.
    '''
    super(Message, self).__init__()
    self.value = bitValue

  def getBit(self, bitIndex):
    '''
    Provides bit at a given index

    Parameters
    ----------
    bitIndex : long
      Bit index

    Returns
    -------
    int
      Bit value: 1 for bit 0 and -1 for bit 1
    '''
    return self.value
