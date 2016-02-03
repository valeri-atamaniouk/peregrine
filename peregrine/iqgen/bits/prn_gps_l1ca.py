# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.prn_gps_l1ca` module contains classes and functions
related to GPS L1 C/A PRN processing

"""

import peregrine.include.generateCAcode

caCodes = peregrine.include.generateCAcode.caCodes

class PrnCode(object):
  '''
  GPS L1 C/A code object
  '''
  CODE_LENGTH = 1023
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
    self.caCode = caCodes[prnNo - 1][:]
    self.prnNo = prnNo

  def getCodeBit(self, chipIndex):
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
    return self.caCode[chipIndex % self.CODE_LENGTH]

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
