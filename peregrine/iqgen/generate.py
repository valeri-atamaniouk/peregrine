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
    _svTime0_s = _sv.doppler.computeSvTimeS(time0S)
    _dist0_m = _sv.doppler.computeDistanceM(_svTime0_s)
    _speed_mps = _sv.doppler.computeSpeedMps(_svTime0_s)
    _bit = signals.GPS.L1CA.getSymbolIndex(_svTime0_s)
    _c1 = signals.GPS.L1CA.getCodeChipIndex(_svTime0_s)
    _c2 = signals.GPS.L2C.getCodeChipIndex(_svTime0_s)
    _d1 = signals.GPS.L1CA.calcDopplerShiftHz(_dist0_m, _speed_mps)
    _d2 = signals.GPS.L2C.calcDopplerShiftHz(_dist0_m, _speed_mps)

    print "SV[{}]: distance={} m, speed={} m/s, amplitude={}, time0={} s, SV time={} s, symbol={}, l1 chip={}, l2 chip={}, l1 doppler={}, Hz l2 doppler={} name={}".format(
              _svNo,
              _dist0_m,
              _speed_mps,
              _amp,
              time0S, _svTime0_s,
              _bit,
              _c1, _c2,
              _d1,
              _d2,
              _sv.doppler.NAME
              )

  _t0 = time.clock()
  _count = 0l

  if SNR is not None:
    Nsigma = scipy.sqrt(1. / (4. * 10. ** (SNR / 10.)))
  else:
    Nsigma = None

  if (lowPass):
    lpf = [LowPassFilter() for _ in range(4)]

  if (debugLog): _out_txt = open("out.txt", "wt");

  userTime_s = float(time0S)
  deltaUserTime_s = float(outputConfig.SAMPLE_BATCH_SIZE) / outputConfig.SAMPLE_RATE_HZ
  oldPerformanceCounter = 0

  sigs = scipy.ndarray((4, outputConfig.SAMPLE_BATCH_SIZE), dtype=float)

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

    # Sum up signals for all SVs
    for svIdx in range(len(sv_list)):
      sv = sv_list[svIdx]

      # Add signal from satellite to signal accumulator
      t = sv.getBatchSignals(userTime_s,
                             outputConfig.SAMPLE_BATCH_SIZE,
                             sigs,
                             outputConfig)

      # Debugging output
      if debugLog:
        sv_sigs = t[0]
        idx = t[2]
        codes = t[3]
        for smpl in range(outputConfig.SAMPLE_BATCH_SIZE):
          _out_txt.write("{},{},{}\n".format(sv_sigs[smpl], idx[smpl], codes[smpl]))

    if lowPass:
      # Filter signal values through LPF (IIR Chebyshev type 2)
      for i in range(4):
        sigs[i][:] = lpf[i].filter(sigs[i])

    # for s in sv_sigs:
      # signal_array[Chip.GPS.L1.INDEX] = s

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
