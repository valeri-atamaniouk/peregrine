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
import time
import argparse
import scipy.constants
import numpy

from peregrine.iqgen.satellite import GPS_SV

# Doppler objects
from peregrine.iqgen.bits.doppler_poly import zeroDoppler
from peregrine.iqgen.bits.doppler_poly import constDoppler
from peregrine.iqgen.bits.doppler_poly import linearDoppler
from peregrine.iqgen.bits.doppler_sine import sineDoppler

# Amplitude objects
from peregrine.iqgen.bits.amplitude_poly import AmplitudePoly
from peregrine.iqgen.bits.amplitude_sine import AmplitudeSine

# from signals import GPS, GPS_L2C_Signal, GPS_L1CA_Signal
import peregrine.iqgen.bits.signals as signals

from peregrine.iqgen.if_iface import LowRateConfig
from peregrine.iqgen.if_iface import NormalRateConfig
from peregrine.iqgen.if_iface import HighRateConfig
from peregrine.iqgen.if_iface import CustomRateConfig

# Message data
from peregrine.iqgen.bits.message_const import Message as ConstMessage
from peregrine.iqgen.bits.message_zeroone import Message as ZeroOneMessage
from peregrine.iqgen.bits.message_block import Message as BlockMessage

# PRN code generators
from peregrine.iqgen.bits.prn_gps_l1ca import PrnCode as GPS_L1CA_Code
from peregrine.iqgen.bits.prn_gps_l2c import PrnCode as GPS_L2C_Code

# Bit stream encoders
from peregrine.iqgen.bits.encoder_gps import GPSL1BitEncoder
from peregrine.iqgen.bits.encoder_gps import GPSL2BitEncoder
from peregrine.iqgen.bits.encoder_gps import GPSL1L2BitEncoder
from peregrine.iqgen.bits.encoder_gps import GPSL1TwoBitsEncoder
from peregrine.iqgen.bits.encoder_gps import GPSL2TwoBitsEncoder
from peregrine.iqgen.bits.encoder_gps import GPSL1L2TwoBitsEncoder

from peregrine.iqgen.generate import generateSamples

def computeTimeDelay(doppler, symbol_index, chip_index, signal, code):
  '''
  Helper function to compute signal delay to match given symbol and chip
  indexes.

  Parameters
  ----------
  doppler : object
    Doppler object
  symbol_index : long
    Index of the symbol or pseudosymbol
  chip_index : long
    Chip index
  signal : object
    Signal object
  code : object
    Code object

  Returns
  -------
  float
     User's time in seconds when the user starts receiving the given symbol
     and code.
  '''
  if symbol_index == 0 and chip_index == 0:
    return 0.

  symbolDelay_s = (1. / signal.SYMBOL_RATE_HZ) * symbol_index
  chipDelay_s = (1. / signal.CODE_CHIP_RATE_HZ) * chip_index
  distance_m = doppler.computeDistanceM(symbolDelay_s + chipDelay_s)
  return distance_m / scipy.constants.c

def prepareArgsParser():
  '''
  Constructs command line argument parser.

  Returns
  -------
  object
    Command line argument parser object.
  '''
  class AddSv(argparse.Action):
    def __init__(self, option_strings, dest, nargs=None, **kwargs):
      super(AddSv, self).__init__(option_strings, dest, **kwargs)

    def __call__(self, parser, namespace, values, option_string=None):
      # Initialize SV list if not yet done
      if namespace.gps_sv is None:
        namespace.gps_sv = []

      # Add SV to the tail of the list.
      sv = GPS_SV(int(values))
      namespace.gps_sv.append(sv)

      # Reset all configuration parameters

      # Doppler
      namespace.doppler_type = "zero"
      namespace.doppler_value = 0.
      namespace.doppler_speed = 0.
      namespace.doppler_distance = 0.
      namespace.doppler_amplitude = 0.
      namespace.doppler_period = 1.

      # Source message data
      namespace.message_type = "zero"
      namespace.message_file = None

      # Amplitude parameters
      namespace.amplitude_type = "poly"
      namespace.amplitude_a0 = None
      namespace.amplitude_a1 = None
      namespace.amplitude_a2 = None
      namespace.amplitude_a3 = None
      namespace.amplitude_period = None

  class UpdateSv(argparse.Action):
    def __init__(self, option_strings, dest, nargs=None, **kwargs):
      super(UpdateSv, self).__init__(option_strings, dest, **kwargs)

    def __call__(self, parser, namespace, values, option_string=None):
      sv_list = getattr(namespace, "gps_sv")
      if sv_list is None:
        raise ValueError("No SV specified")
      setattr(namespace, self.dest, values)
      # super(UpdateSv, self).__call__(parser, namespace, values, option_string)
      self.doUpdate(sv_list[len(sv_list) - 1], parser, namespace, values,
                    option_string)

    def doUpdate(self, sv, parser, namespace, values, option_string):
      pass

  class UpdateBands(UpdateSv):
    def __init__(self, option_strings, dest, nargs=None, **kwargs):
      super(UpdateBands, self).__init__(option_strings, dest, **kwargs)

    def doUpdate(self, sv, parser, namespace, values, option_string):
      l1caEnabled = False
      l2cEnabled = False
      if namespace.bands == "l1ca":
        l1caEnabled = True
      elif namespace.bands == "l2c":
        l2cEnabled = True
      elif namespace.bands == "l1ca+l2c":
        l1caEnabled = True
        l2cEnabled = True
      else:
        raise ValueError()
      sv.setL1CAEnabled(l1caEnabled)
      sv.setL2CEnabled(l2cEnabled)

  class UpdateDopplerType(UpdateSv):
    def __init__(self, option_strings, dest, nargs=None, **kwargs):
      super(UpdateDopplerType, self).__init__(option_strings, dest, **kwargs)

    def doUpdate(self, sv, parser, namespace, values, option_string):
      if sv.l1caEnabled:
        frequency_hz = signals.GPS.L1CA.CENTER_FREQUENCY_HZ
      elif sv.l2Enabled:
        frequency_hz = signals.GPS.L2C.CENTER_FREQUENCY_HZ
      else:
        raise ValueError("Signal band must be specified before doppler")

      if namespace.doppler_type == "zero":
        doppler = zeroDoppler(namespace.doppler_distance, frequency_hz)
      elif namespace.doppler_type == "const":
        doppler = constDoppler(namespace.doppler_distance,
                               frequency_hz,
                               namespace.doppler_value)
      elif namespace.doppler_type == "linear":
        doppler = linearDoppler(namespace.doppler_distance,
                                frequency_hz,
                                namespace.doppler_value,
                                namespace.doppler_speed)
      elif namespace.doppler_type == "sine":
        doppler = sineDoppler(namespace.doppler_distance,
                              frequency_hz,
                              namespace.doppler_value,
                              namespace.doppler_amplitude,
                              namespace.doppler_period)
      else:
        raise ValueError("Unsupported doppler type")
      sv.doppler = doppler

  class UpdateAmplitudeType(UpdateSv):
    def __init__(self, option_strings, dest, nargs=None, **kwargs):
      super(UpdateAmplitudeType, self).__init__(option_strings, dest, **kwargs)

    def doUpdate(self, sv, parser, namespace, values, option_string):
      if namespace.amplitude_type == "poly":
        coeffs = []
        hasHighOrder = False

        srcA = [namespace.amplitude_a3, namespace.amplitude_a2,
                namespace.amplitude_a1, namespace.amplitude_a0]
        for a in srcA:
          if a is not None:
            coeffs.append(a)
            hasHighOrder = True
          elif hasHighOrder:
            coeffs.append(0.)
        amplitude = AmplitudePoly(tuple(coeffs))
      elif namespace.amplitude_type == "sine":
        initial = 1.
        ampl = 0.5
        period_s = 1.
        if namespace.amplitude_a0 is not None:
          initial = namespace.amplitude_a0
        if namespace.amplitude_a1 is not None:
          ampl = namespace.amplitude_a1
        if namespace.amplitude_period is not None:
          period_s = namespace.amplitude_period

        amplitude = AmplitudeSine(initial, ampl, period_s)
      else:
        raise ValueError("Unsupported amplitude type")
      sv.setAmplitude(amplitude)

  class UpdateMessageType(UpdateSv):
    def __init__(self, option_strings, dest, nargs=None, **kwargs):
      super(UpdateMessageType, self).__init__(option_strings, dest, **kwargs)

    def doUpdate(self, sv, parser, namespace, values, option_string):
      if namespace.message_type == "zero":
        message = ConstMessage(1)
      elif namespace.message_type == "one":
        message = ConstMessage(-1)
      elif namespace.message_type == "zero+one":
        message = ZeroOneMessage()
      else:
        raise ValueError("Unsupported message type")
      sv.setL1CAMessage(message)
      sv.setL2CMessage(message)

  class UpdateMessageFile(UpdateSv):
    def __init__(self, option_strings, dest, nargs=None, **kwargs):
      super(UpdateMessageFile, self).__init__(option_strings, dest, **kwargs)

    def doUpdate(self, sv, parser, namespace, values, option_string):
      data = numpy.fromfile(namespace.message_file, dtype=numpy.uint8)
      namespace.message_file.close()
      data = numpy.unpackbits(data)
      data = numpy.asarray(data, dtype=numpy.int8)
      data <<= 1
      data -= 1
      numpy.negative(data, out=data)
      message = BlockMessage(data)

      sv.setL1CAMessage(message)
      sv.setL2CMessage(message)

  parser = argparse.ArgumentParser(description="Signal generator", usage='%(prog)s [options]')
  parser.add_argument('--gps-sv',
                      default=[],
                      help='Enable GPS satellite',
                      action=AddSv)
  parser.add_argument('--bands',
                      default="l1ca",
                      choices=["l1ca", "l2c", "l1ca+l2c"],
                      help="Signal bands to enable",
                      action=UpdateBands)
  parser.add_argument('--doppler-type',
                      default="zero",
                      choices=["zero", "const", "linear", "sine"],
                      help="Configure doppler type",
                      action=UpdateDopplerType)
  parser.add_argument('--doppler-value',
                      type=float,
                      help="Doppler shift in hertz (initial)",
                      action=UpdateDopplerType)
  parser.add_argument('--doppler-speed',
                      type=float,
                      help="Doppler shift change in hertz/second",
                      action=UpdateDopplerType)
  parser.add_argument('--doppler-distance',
                      type=float,
                      help="Distance in meters for doppler delay (initial)",
                      action=UpdateDopplerType)
  parser.add_argument('--doppler-amplitude',
                      type=float,
                      help="Doppler change amplitude (hertz)",
                      action=UpdateDopplerType)
  parser.add_argument('--doppler-period',
                      type=float,
                      help="Doppler change period (seconds)",
                      action=UpdateDopplerType)
  parser.add_argument('--amplitude-type',
                      default="poly",
                      choices=["poly", "sine"],
                      help="Configure amplitude type: polynomial or sine.",
                      action=UpdateAmplitudeType)
  parser.add_argument('--amplitude-a0',
                      type=float,
                      help="Amplitude coefficient (a0 for polynomial; offset for sine)",
                      action=UpdateAmplitudeType)
  parser.add_argument('--amplitude-a1',
                      type=float,
                      help="Amplitude coefficient (a1 for polynomial, amplitude for size)",
                      action=UpdateAmplitudeType)
  parser.add_argument('--amplitude-a2',
                      type=float,
                      help="Amplitude coefficient (a2 for polynomial)",
                      action=UpdateAmplitudeType)
  parser.add_argument('--amplitude-a3',
                      type=float,
                      help="Amplitude coefficient (a3 for polynomial)",
                      action=UpdateAmplitudeType)
  parser.add_argument('--amplitude-period',
                      type=float,
                      help="Amplitude period in seconds for sine",
                      action=UpdateAmplitudeType)
  parser.add_argument('--message-type', default="zero",
                      choices=["zero", "one", "zero+one"],
                      help="Message type",
                      action=UpdateMessageType)
  parser.add_argument('--message-file',
                      type=argparse.FileType('rb'),
                      help="Source file for message contents.",
                      action=UpdateMessageFile)
  parser.add_argument('--symbol_delay',
                      type=int,
                      help="Initial symbol index")
  parser.add_argument('--chip_delay',
                      type=int,
                      help="Initial chip index")
  parser.add_argument('--lpf',
                      default=False,
                      help="Enable low pass filter",
                      action='store_true')
  parser.add_argument('--snr',
                      type=float,
                      help="SNR for noise generation")
  parser.add_argument('--debug',
                      default=False,
                      help="Enable debug output",
                      action='store_true')
  parser.add_argument('--generate',
                      type=float,
                      default=3.,
                      help="Amount of data to generate, in seconds")
  parser.add_argument('--encoder',
                      default="2bits",
                      choices=["1bit", "2bits"],
                      help="Output data format")
  parser.add_argument('--output',
                      type=argparse.FileType('wb'),
                      help="Output file name")
  parser.add_argument('--profile',
                      default="normal_rate",
                      choices=["low_rate", "normal_rate", "high_rate",
                               "custom_rate"],
                      help="Output profile configuration")
  parser.add_argument('-j', '--jobs',
                      type=int,
                      default=1,
                      help="Use parallel threads")

  return parser

def main():
  parser = prepareArgsParser()
  args = parser.parse_args()

  if args.output is None:
    parser.print_help()
    return 0

  if args.profile == "low_rate":
    outputConfig = LowRateConfig
  elif args.profile == "normal_rate":
    outputConfig = NormalRateConfig
  elif args.profile == "high_rate":
    outputConfig = HighRateConfig
  elif args.profile == "custom_rate":
    outputConfig = CustomRateConfig
  else:
    raise ValueError()

  print "Output configuration:"
  print "\tDescription:", outputConfig.NAME
  print "\tSampling rate:", outputConfig.SAMPLE_RATE_HZ
  print "\tBatch size:", outputConfig.SAMPLE_BATCH_SIZE
  print "\tGPS L1 IF:", outputConfig.GPS.L1.INTERMEDIATE_FREQUENCY_HZ
  print "\tGPS L2 IF:", outputConfig.GPS.L2.INTERMEDIATE_FREQUENCY_HZ

  print "Satellites: ", args.gps_sv

  # Check which signals are enabled on each of satellite to select proper
  # output encoder
  enabledGPSL1 = False
  enabledGPSL2 = False

  for sv in args.gps_sv:
    enabledGPSL1 |= sv.isBandEnabled(outputConfig.GPS.L1.INDEX, outputConfig)
    enabledGPSL2 |= sv.isBandEnabled(outputConfig.GPS.L2.INDEX, outputConfig)

  # Configure data encoder
  if args.encoder == "1bit":
    if enabledGPSL1 and enabledGPSL2:
      encoder = GPSL1L2BitEncoder(outputConfig)
    elif enabledGPSL2:
      encoder = GPSL2BitEncoder(outputConfig)
    else:
      encoder = GPSL1BitEncoder(outputConfig)
  elif args.encoder == "2bits":
    if enabledGPSL1 and enabledGPSL2:
      encoder = GPSL1L2TwoBitsEncoder(outputConfig)
    elif enabledGPSL2:
      encoder = GPSL2TwoBitsEncoder(outputConfig)
    else:
      encoder = GPSL1TwoBitsEncoder(outputConfig)
  else:
    raise ValueError("Encoder type is not supported")

  if enabledGPSL1:
    signal = signals.GPS.L1CA
    code = GPS_L1CA_Code
  elif enabledGPSL2:
    signal = signals.GPS.L2C
    code = GPS_L2C_Code
  else:
    signal = signals.GPS.L1CA
    code = GPS_L1CA_Code

  # Compute time delay for the needed bit/chip number
  # This delay is computed for the first satellite
  initial_symbol_idx = 0  # Initial symbol index
  initial_chip_idx = 0  # Initial chip index
  if args.chip_delay is not None:
    initial_chip_idx = args.chip_delay
  if args.symbol_delay is not None:
    initial_chip_idx = args.symbol_delay

  time0_s = computeTimeDelay(args.gps_sv[0].doppler,
                             initial_symbol_idx,
                             initial_chip_idx,
                             signal,
                             code)
  print "Computed symbol/chip delay={} seconds".format(time0_s)

  startTime_s = time.clock()
  n_samples = long(outputConfig.SAMPLE_RATE_HZ * args.generate)

  print "Generating {} samples for {} seconds".format(n_samples, args.generate)

  generateSamples(args.output,
                  args.gps_sv,
                  encoder,
                  time0_s,
                  n_samples,
                  outputConfig,
                  SNR=args.snr,
                  lowPass=args.lpf,
                  debugLog=args.debug,
                  threadCount=args.jobs)
  args.output.close()

  duration_s = time.clock() - startTime_s
  ratio = n_samples / duration_s
  print "Total time = {} sec. Ratio={} samples per second".format(duration_s, ratio)

if __name__ == '__main__':
  main()

