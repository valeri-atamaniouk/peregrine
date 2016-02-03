# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.message_zeroone` module contains classes and functions
related to symbol contents that flips the value every other bit.

"""

class Message(object):
  '''
  Message that contains zeros and ones
  '''
  def __init__(self):
    '''
    Constructs object.
    '''
    super(Message, self).__init__()

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
    if (bitIndex & 1 != 0):
      return -1
    else:
      return 1
