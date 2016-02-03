# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.prn_gps_l2c` module contains classes and functions
related to GPS L2C PRN processing

"""

from peregrine.include.generateL2CMcode import L2CMCodes

class PrnCode(object):
  '''
  Combined GPS L2 CM and CL code object
  '''

  class CM_Code(object):
    '''
    GPS L2 Civil Medium code object
    '''
    CODE_LENGTH = 10230
    CODE_FREQUENCY_HZ = 511.5e3

    def __init__(self, prnNo):
      '''
      Initializes object.

      Parameters
      ----------
      prnNo : int
        SV identifier
      '''
      super(PrnCode.CM_Code, self).__init__()
      self.caCode = L2CMCodes[prnNo - 1][:]
      self.prnNo = prnNo

    def getCodeBit(self, codeBitIndex):
      '''
      Returns chip value by index.

      Parameters
      ----------
      chipIndex : long
        Chip index

      Returns
      -------
      int
        Chip value by index
      '''
      return self.caCode[codeBitIndex % self.CODE_LENGTH]

  class CL_Code(object):
    '''
    GPS L2 Civil Long code object
    '''
    CODE_LENGTH = 767250
    CODE_FREQUENCY_HZ = 511.5e3

    def __init__(self, prnNo):
      '''
      Initializes object.

      Parameters
      ----------
      prnNo : int
        SV identifier
      '''
      super(PrnCode.CL_Code, self).__init__()
      self.prnNo = prnNo

    def getCodeBit(self, codeBitIndex):
      '''
      Returns chip value by index.

      Currently GPS L2 CL code can be pseudo-random

      Parameters
      ----------
      chipIndex : long
        Chip index

      Returns
      -------
      int
        Chip value by index
      '''
      if (codeBitIndex & 1 != 0):
        return -1
      else:
        return 1

  CODE_LENGTH = CL_Code.CODE_LENGTH * 2
  CODE_FREQUENCY_HZ = 1023e3

  def __init__(self, prnNo):
    '''
    Initializes object.

    Parameters
    ----------
    prnNo : int
      SV identifier
    '''
    super(PrnCode, self).__init__()
    self.cl = PrnCode.CL_Code(prnNo)
    self.cm = PrnCode.CM_Code(prnNo)

  def getCodeIndex(self, svTime_s):
    '''
    Computes code chip index for a given SV time.

    Parameters
    ----------
    svTime_s : float
      SV time in seconds

    Returns
    -------
    long
      code chip index
    '''

    return long(svTime_s * self.CODE_FREQUENCY_HZ)

  def getCodeBit(self, codeBitIndex):
    '''
    For GPS L2C code bits are taken from CM and CL codes in turn.
    '''
    idx = long(codeBitIndex)
    if (idx & 1 != 0):
      return self.cl.getCodeBit(idx / 2)
    else:
      return self.cm.getCodeBit(idx / 2)
