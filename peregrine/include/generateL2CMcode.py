# Copyright (C) 2016 Swift Navigation Inc.
#
# Contact: Dmitry Tatarinov <dmitry.tatarinov@exafore.com>
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

import numpy as np

def generateL2CMcode(PRN):
  '''
  The function generates PRN sequence for a particular SV.
  In the sequence '0' is represented as '1', 1 as -1

  INPUT: SV number from 0 (SV1) to 31 (SV32)
  OUTPUT: 
    - PRN seequence array of 10230, 
    - end state of shift register for testing purpuses
  '''
  #--- Sanity sheck for PRN number ------------------------------------------
  if PRN < 0 or PRN > 31:
    print 'PRN number(',PRN,') is not in range [0..31]'
    return -1

  #--- Initial states for shift register for each PRN[1..32], ---------------
  # see IS-GPS-200H, Table 3-IIa
  initL2CM = [\
      0742417664, #PRN 1
      0756014035,
      0002747144,
      0066265724,
      0601403471,
      0703232733,
      0124510070,
      0617316361,
      0047541621,
      0733031046,
      0713512145,
      0024437606,
      0021264003,
      0230655351,
      0001314400,
      0222021506,
      0540264026,
      0205521705,
      0064022144,
      0120161274,
      0044023533,
      0724744327,
      0045743577,
      0741201660,
      0700274134,
      0010247261,
      0713433445,
      0737324162,
      0311627434,
      0710452007,
      0722462133,
      0050172213  #PRN 32
  ]

  #--- Init L2CM PRN and shift reg ------------------------------------------
  L2CM_PRN = [0 for i in range(10230)]

  #--- Load Shift register --------------------------------------------------
  shift_cm = initL2CM[PRN]

  #--- Generate L2CM PRN chips ----------------------------------------------
  for i in range(10230):

    out = shift_cm & 1
    if out == 1:
      L2CM_PRN[i] = -1 #-1 to represent '1'
    else:
      L2CM_PRN[i] = 1  # 1 to represent '0'

    if i == 10230-1:
      break #don't shift the reg if it's a last chip

    shift_cm ^= out << 3
    shift_cm ^= out << 4
    shift_cm ^= out << 5
    shift_cm ^= out << 6
    shift_cm ^= out << 9
    shift_cm ^= out << 11
    shift_cm ^= out << 13
    shift_cm ^= out << 16
    shift_cm ^= out << 19
    shift_cm ^= out << 21
    shift_cm ^= out << 24
    shift_cm = (shift_cm >> 1) | (out << 26)

  return (L2CM_PRN, shift_cm)
  
L2CMCodes = np.empty((32,10230), dtype=np.int8)
for PRN in range(32):
  L2CMCodes[PRN][:] = generateL2CMcode(PRN)[0]

if __name__ == "__main__":

#--- Unit test --------------------------------------------------------------

  end_shift_regs_test = [\
      0552566002,
      0034445034,
      0723443711,
      0511222013,
      0463055213,
      0667044524,
      0652322653,
      0505703344,
      0520302775,
      0244205506,
      0236174002,
      0654305531,
      0435070571,
      0630431251,
      0234043417,
      0535540745,
      0043056734,
      0731304103,
      0412120105,
      0365636111,
      0143324657,
      0110766462,
      0602405203,
      0177735650,
      0630177560,
      0653467107,
      0406576630,
      0221777100,
      0773266673,
      0100010710,
      0431037132,
      0624127475
  ]

  for prn in range(32):
    (L2CMcode,s_r) =  generateL2CMcode(prn)

    if s_r != end_shift_regs_test[prn]:
      print 'L2CM code for PRN',prn+1,'is bad', oct(s_r)
    else:
      print 'L2CM code for PRN',prn+1,'is good'

#--- End of unit test -------------------------------------------------------

