# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.message_block` module contains classes and functions
related to providing predefined symbol contents.

"""

class Message(object):
  '''
  Message that is a block of bits
  '''
  def __init__(self, messageData):
    '''
    Constructs message object.

    Parameters
    ----------
    messageData : array-like
      Array with message bits. Bit 0 is encoded with 1, bit 1 is encoded with -1
    '''
    super(Message, self).__init__()
    self.messageData = messageData[:]
    self.messageLen = len(self.messageData)

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

    return self.messageData[bitIndex % self.messageLen]
