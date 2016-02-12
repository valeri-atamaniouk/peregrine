# Copyright (C) 2016 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""
The :mod:`peregrine.iqgen.generate` module contains classes and functions
related to main loop of samples generation.

"""
from peregrine.iqgen.bits.low_pass_filter import LowPassFilter

from peregrine.iqgen.bits import signals

import scipy
import time

def computeTimeIntervalS(outputConfig):
  '''
  Helper for computing generation interval duration in seconds.

  Parameters
  ----------
  outputConfig : object
    Output configuration.

  Returns
  -------
  float
    Generation interval duration in seconds
  '''
  deltaTime_s = float(outputConfig.SAMPLE_BATCH_SIZE) / \
                outputConfig.SAMPLE_RATE_HZ
  return deltaTime_s


def generateSamples(outputFile,
                    sv_list,
                    encoder,
                    time0S,
                    nSamples,
                    outputConfig,
                    SNR=None,
                    lowPass=False,
                    debugLog=False):
  '''
  Generates samples.

  Parameters
  ----------
  fileName : string
    Output file name.
  sv_list : list
    List of configured satellite objects.
  encoder : Encoder
    Output encoder object.
  time0S : float
    Time epoch for the first sample.
  nSamples : long
    Total number of samples to generate.
  outputConfig : object
    Output parameters
  SNR : float, optional
    When specified, adds random noise to the output.
  lowPass : bool, optional
    Controls LPF signal post-processing. Disabled by default.
  debugLog : bool, optional
    Control generation of additional debug output. Disabled by default.
  '''

  #
  # Print out parameters
  #
  print "Generating samples, sample rate={} Hz, interval={} seconds, SNR={}".format(
        outputConfig.SAMPLE_RATE_HZ, nSamples / outputConfig.SAMPLE_RATE_HZ, SNR)

  #
  # Print out SV parameters
  #
  for _sv in sv_list:
    _svNo = _sv.getSvName()
    _amp = _sv.amplitude
    _svTime0_s = 0
    _dist0_m = _sv.doppler.computeDistanceM(_svTime0_s)
    _speed_mps = _sv.doppler.computeSpeedMps(_svTime0_s)
    _bit = signals.GPS.L1CA.getSymbolIndex(_svTime0_s)
    _c1 = signals.GPS.L1CA.getCodeChipIndex(_svTime0_s)
    _c2 = signals.GPS.L2C.getCodeChipIndex(_svTime0_s)
    _d1 = signals.GPS.L1CA.calcDopplerShiftHz(_dist0_m, _speed_mps)
    _d2 = signals.GPS.L2C.calcDopplerShiftHz(_dist0_m, _speed_mps)

    print "{} = {{".format(_svNo)
    print "  .amplitude: {}".format(_amp)
    print "  .epoc:"
    print "    .userTime:   {}".format(time0S)
    print "    .svTime:     {}".format(_svTime0_s)
    print "    .distance:   {} m".format(_dist0_m)
    print "    .speed:      {} m/s".format(_speed_mps)
    print "    .symbol:     {}".format(_bit)
    print "    .l1_doppler: {} hz".format(_d1)
    print "    .l2_doppler: {} hz".format(_d2)
    print "    .l1_chip:    {}".format(_c1)
    print "    .l2_chip:    {}".format(_c2)
    print "  .doppler: {}".format(_sv.doppler)
    print "}"

  _t0 = time.clock()
  _count = 0l

  if SNR is not None:
    Nsigma = scipy.sqrt(1. / (4. * 10. ** (SNR / 10.)))
  else:
    Nsigma = None

  # Check which bands are enabled, configure band-specific parameters
  bands = [outputConfig.GPS.L1, outputConfig.GPS.L2]  # Supported bands
  lpf = [None] * len(bands)
  bandsEnabled = [False] * len(bands)

  for band in bands:
    for sv in sv_list:
      bandsEnabled[band.INDEX] |= sv.isBandEnabled(band.INDEX, outputConfig)
    if lowPass:
      lpf[band.INDEX] = LowPassFilter(outputConfig,
                                      band.INTERMEDIATE_FREQUENCY_HZ)

  if debugLog: _out_txt = open("out.txt", "wt");

  userTime_s = float(time0S)
  oldPerformanceCounter = 0

  sigs = scipy.ndarray((4, outputConfig.SAMPLE_BATCH_SIZE), dtype=float)

  deltaUserTime_s = computeTimeIntervalS(outputConfig)
  totalSampleCounter = 0l

  for _s in range(0l, nSamples, outputConfig.SAMPLE_BATCH_SIZE):

    # Print performance statistics
    deltaProcessingTime_s = time.clock() - _t0
    if (deltaProcessingTime_s >= 1.):
      newSamples = _s - oldPerformanceCounter
      oldPerformanceCounter = _s
      _t1 = time.clock()
      _rate = float(newSamples) / (_t1 - _t0)
      _t0 = _t1
      _nleft = nSamples - _s
      _timeLeftS = _nleft / _rate
      _timeLeftM = int(_timeLeftS / 60)
      _timeLeftH = _timeLeftM / 60
      _timeLeftD = _timeLeftH / 24
      _timeLeftH %= 24
      _timeLeftM %= 60
      _s1 = int(_timeLeftS % 60)
      _msec = int((_timeLeftS % 1.) * 1000)
      print "Generated {} samples; rate={:.02f} samples/sec; est={:02d}:{:02d}:{:02d}:{:02d}.{:03d}".format(
            _s, _rate,
            _timeLeftD, _timeLeftH, _timeLeftM, _s1,
            _msec)

    if Nsigma is not None:
      # Initialize signal array with noise
      sigs = Nsigma * scipy.randn(4, outputConfig.SAMPLE_BATCH_SIZE)
    else:
      sigs.fill(0.)

    userTimeX_s = userTime_s + deltaUserTime_s
    userTimeAll_s = scipy.linspace(userTime_s,
                                   userTimeX_s,
                                   outputConfig.SAMPLE_BATCH_SIZE,
                                   endpoint=False)

    if debugLog:
      debugData = []

    # Sum up signals for all SVs
    for svIdx in range(len(sv_list)):
      sv = sv_list[svIdx]
      # Add signal from satellite to signal accumulator
      t = sv.getBatchSignals(userTimeAll_s,
                             sigs,
                             outputConfig)
      # Debugging output
      if debugLog:
        debugData.append(t)
      t = None

    if debugLog:
      # Data from all satellites is collected. Now we can dump the debug matrix
      for smpl_idx in range(len(userTimeAll_s)):
        _out_txt.write("{},{},".format(totalSampleCounter,
                                       userTimeAll_s[smpl_idx]))
        for svIdx in range(len(sv_list)):
          sv = sv_list[svIdx]
          sv_bands = debugData[svIdx]
          for band in sv_bands:
            sv_sigs = band[0]
            doppler = band[1]
            idx = band[2]
            codes = band[3]
            _out_txt.write("{},{},{},{}".format(sv_sigs[smpl_idx],
                                                doppler[smpl_idx],
                                                idx[smpl_idx],
                                                codes[smpl_idx]))
        # End of line
        _out_txt.write("\n")
      _out_txt.flush()  # Flush the batch data into file

    if lowPass:
      # Filter signal values through LPF, BPF or another
      for i in range(len(lpf)):
        filterObject = lpf[i]
        if filterObject is not None:
          sigs[i][:] = filterObject.filter(sigs[i])

    # Feed data into encoder
    _bytes = encoder.addSamples(sigs)

    if (len(_bytes) > 0):
      _count += len(_bytes)
      _bytes.tofile(outputFile)

    userTime_s += deltaUserTime_s

  _bytes = encoder.flush()
  if (len(_bytes) > 0):
    _bytes.tofile(outputFile)

  if (debugLog): _out_txt.close()
