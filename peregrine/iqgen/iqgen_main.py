# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.iqgen_main` module contains classes and functions
related to parameter processing.

"""
from satellite import GPS_SV
import time
from peregrine.iqgen.bits.doppler_const import constDoppler
from peregrine.iqgen.bits.doppler_linear import linearDoppler
from peregrine.iqgen.bits.doppler_zero import Doppler as Stationary
from peregrine.iqgen.bits.doppler_zero2 import Doppler as Stationary2

# from signals import GPS, GPS_L2C_Signal, GPS_L1CA_Signal
import peregrine.iqgen.bits.signals as signals
import scipy.constants

from peregrine.iqgen.if_iface import Chip

# Message data
from peregrine.iqgen.bits.message_const import Message as ConstMessage
from peregrine.iqgen.bits.message_zeroone import Message as ZeroOneMessage
from  peregrine.iqgen.bits.message_block import Message as BlockMessage

# PRN code generators
from peregrine.iqgen.bits.prn_gps_l1ca import PrnCode as GPS_L1CA_Code
from peregrine.iqgen.bits.prn_gps_l2c import PrnCode as GPS_L2C_Code

# Bit stream encoders
from peregrine.iqgen.bits.encoder_gps import GPSL1BitEncoder
from peregrine.iqgen.bits.encoder_gps import GPSL2BitEncoder
from peregrine.iqgen.bits.encoder_gps import GPSL1L2BitEncoder

from peregrine.iqgen.generate import generateSamples

def computeTimeDelay(doppler, symbol_index, chip_index, signal, code):
  '''
  Helper function to compute signal delay to match given symbol and chip
  indexes.

  @param[in] sv           Satellite object
  @param[in] symbol_index Index of the symbol or pseudosymbol
  @param[in] chip_index   Chip index
  @param[in] signal       Signal object
  @param[in] code         Code object

  @return User's time in seconds when the user starts receiving the given symbol
          and code.
  '''
  symbolDelay_s = (1. / signal.SYMBOL_RATE_HZ) * symbol_index
  chipDelay_s = (1. / signal.CODE_CHIP_RATE_HZ) * chip_index
  distance_m = doppler.computeDistanceM(symbolDelay_s + chipDelay_s)
  return distance_m / scipy.constants.c

def main():
  svs = [ GPS_SV(22) ]  # SV list
  SNR = 100  # Power for some C/No control . When 100 - disabled
  useLpf = False
  n_seconds = 3  # Total number of seconds to generate samples
  initial_symbol_idx = 0  # Initial symbol index
  initial_chip_idx = 0  # Initial chip index
  distance0_m = 0.
  doppler0_hz = 10.
  amplitude = 1.
  dopplerSpeed_hzps = 1.
  enableDebugLog = False

  enableGPSL1 = True
  enableGPSL2 = False

  if enableGPSL1:
    frequency_hz = signals.GPS.L1CA.CENTER_FREQUENCY_HZ
    signal = signals.GPS.L1CA
    code = GPS_L1CA_Code
  elif enableGPSL2:
    frequency_hz = signals.GPS.L2C.CENTER_FREQUENCY_HZ
    signal = signals.GPS.L2C
    code = GPS_L2C_Code

  def zeroDoppler(): return Stationary(distance0_m)
  def zeroDoppler2(): return Stationary2(distance0_m)
  def constDoppler2(): return constDoppler(distance0_m, frequency_hz, doppler0_hz)
  def linearDoppler2(): return linearDoppler(distance0_m, doppler0_hz, frequency_hz, dopplerSpeed_hzps)

  messageVariant = 0
  encoderVariant = 0
  dopplerVariant = 0

  if enableGPSL1 and enableGPSL2:
    encoderVariant = 2
  elif enableGPSL1:
    encoderVariant = 0
  elif enableGPSL2:
    encoderVariant = 1
  else:
    raise BaseException

  encoders = [GPSL1BitEncoder(), GPSL2BitEncoder(), GPSL1L2BitEncoder()]
  messages = [ConstMessage(1), ConstMessage(1), ZeroOneMessage()]
  dopplers = [constDoppler2, linearDoppler2, zeroDoppler, zeroDoppler2]

  for sv in svs:
    # Choose symbol data
    sv.setL1CAMessage(messages[messageVariant])
    sv.setL2CMessage(messages[messageVariant])

    # Set some amplitude in range (0.8;1.0), mean 0.9
    sv.setAmplitude(amplitude)

    # Choose doppler function. Can be anything from doppler.py
    sv.doppler = dopplers[dopplerVariant]()

    # Enable bands
    sv.setL1CAEnabled(enableGPSL1)  # Enabled L1 C/A
    sv.setL2CEnabled(enableGPSL2)  # Enable L2 C

    # print "PRN=[",
    # for i in range(sv.l1caCode.CODE_LENGTH):
    #   print sv.l1caCode.getCodeBit(i),
    # print "]"

  # Compute time delay for the needed bit/chip number
  # This delay is computed for the first satellite
  time0_s = computeTimeDelay(svs[0].doppler,
                             initial_symbol_idx,
                             initial_chip_idx,
                             signal,
                             code)
  print "Computed symbol/chip delay={} seconds".format(time0_s)

  # Configure data encoder
  _encoder = encoders[encoderVariant]

  _t0 = time.clock()
  _n_samples = long(Chip.SAMPLE_RATE_HZ) * n_seconds  # 1 second

  print "Generating {} samples for {} seconds".format(_n_samples, n_seconds)

  generateSamples("build/out.bin", svs, _encoder, time0_s, _n_samples,
                  SNR=SNR,
                  lowPass=useLpf,
                  debugLog=enableDebugLog)

  _t1 = time.clock()
  _ratio = _n_samples / (_t1 - _t0)
  print "Total time = {} sec. Ratio={} samples per second".format(_t1 - _t0, _ratio)

if __name__ == '__main__':
  main()

