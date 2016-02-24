# Copyright (C) 2016 Swift Navigation Inc.
# Contact: Valeri Atamaniouk <valeri@swiftnav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

import sys
import traceback

"""
The :mod:`peregrine.iqgen.generate` module contains classes and functions
related to main loop of samples generation.

"""
from peregrine.iqgen.bits.filter_lowpass import LowPassFilter
from peregrine.iqgen.bits.filter_bandpass import BandPassFilter

from peregrine.iqgen.bits import signals

import scipy
import numpy
import time

# import threading
import multiprocessing

import logging

logger = logging.getLogger(__name__)


class Task(object):
  '''
  Period computation task. This object performs a batch computation of signal
  in the specified range.
  '''

  def __init__(self,
               outputConfig,
               signalSources,
               noiseSigma=None,
               tcxo=None,
               signalFilters=None,
               generateDebug=False):
    '''
    Parameters
    ----------
    outputConfig : object
    userTime0_s : float
    nSamples : long
    signalSources : array-like
    noiseSigma : float
    tcxo : object, optional
      TCXO control object
    signalFilters : array-like
    generateDebug : bool
    firstSampleIndex : long
    '''

    self.outputConfig = outputConfig
    self.signalSources = signalSources
    self.signalFilters = signalFilters
    self.generateDebug = generateDebug
    self.noiseSigma = noiseSigma
    self.tcxo = tcxo
    self.signals = scipy.ndarray(shape=(4, outputConfig.SAMPLE_BATCH_SIZE),
                                 dtype=numpy.float)
    if noiseSigma is not None:
      # Initialize signal array with noise
      # self.noise = noiseSigma * scipy.randn(4, outputConfig.SAMPLE_BATCH_SIZE)
      self.noise = numpy.random.normal(loc=0.,
                                       scale=noiseSigma,
                                       size=(4, outputConfig.SAMPLE_BATCH_SIZE))
      # print self.noise
    else:
      self.noise = None
    self.nSamples = outputConfig.SAMPLE_BATCH_SIZE

  def update(self, userTime0_s, nSamples, firstSampleIndex):
    self.userTime0_s = userTime0_s
    self.firstSampleIndex = firstSampleIndex

    if (self.nSamples != nSamples):
      self.signals = scipy.ndarray((4, nSamples), dtype=float)
      # self.signals.resize((4, nSamples))
      if self.noiseSigma is not None:
        # Initialize signal array with noise
        self.noise = self.noiseSigma * scipy.randn(4, nSamples)
      self.nSamples = nSamples

  def perform(self):
    outputConfig = self.outputConfig
    signalSources = self.signalSources
    signalFilters = self.signalFilters
    tcxo = self.tcxo
    firstSampleIndex = self.firstSampleIndex
    finalSampleIndex = firstSampleIndex + self.nSamples

    generateDebug = self.generateDebug

    userTime0_s = self.userTime0_s
    userTimeX_s = userTime0_s + float(self.nSamples) / \
        float(outputConfig.SAMPLE_RATE_HZ)
    userTimeAll_s = scipy.linspace(userTime0_s,
                                   userTimeX_s,
                                   self.nSamples,
                                   endpoint=False)

    if tcxo:
      tcxoTimeDrift_s = tcxo.computeTcxoTime(firstSampleIndex,
                                             finalSampleIndex,
                                             outputConfig)
      if tcxoTimeDrift_s:
        userTimeAll_s += tcxoTimeDrift_s

    noise = self.noise
    sigs = self.signals
    sigs.fill(0.)
    if noise is not None:
      # Initialize signal array with noise
      sigs += noise

    if generateDebug:
      debugData = []
    else:
      debugData = None

    # Sum up signals for all SVs
    for signalSource in signalSources:
      # Add signal from source (satellite) to signal accumulator
      t = signalSource.getBatchSignals(userTimeAll_s,
                                       sigs,
                                       outputConfig)
      # Debugging output
      if generateDebug:
        debugData.append(t)
      t = None

    if signalFilters is list:
      # Filter signal values through LPF, BPF or another
      for i in range(len(self.filters)):
        filterObject = signalFilters[i]
        if filterObject is not None:
          sigs[i][:] = filterObject.filter(sigs[i])

    inputParams = (self.userTime0_s, self.nSamples, self.firstSampleIndex)
    return (inputParams, sigs, debugData)


class Worker(multiprocessing.Process):

  def __init__(self,
               outputConfig,
               signalSources,
               noiseSigma,
               tcxo,
               signalFilters,
               generateDebug):
    super(Worker, self).__init__()
    self.queueIn = multiprocessing.Queue()
    self.queueOut = multiprocessing.Queue()
    self.totalWaitTime_s = 0.
    self.totalExecTime_s = 0.
    self.outputConfig = outputConfig
    self.signalSources = signalSources
    self.noiseSigma = noiseSigma
    self.tcxo = tcxo
    self.signalFilters = signalFilters
    self.generateDebug = generateDebug

  def run(self):
    task = Task(self.outputConfig,
                self.signalSources,
                noiseSigma=self.noiseSigma,
                tcxo=self.tcxo,
                signalFilters=self.signalFilters,
                generateDebug=self.generateDebug)

    while True:
      opStartTime_s = time.clock()
      inputRequest = self.queueIn.get()
      if inputRequest is None:
        # EOF reached
        break
      (userTime0_s, nSamples, firstSampleIndex) = inputRequest
      # print "Received params", userTime0_s, nSamples, firstSampleIndex
      opDuration_s = time.clock() - opStartTime_s
      self.totalWaitTime_s += opDuration_s
      startTime_s = time.clock()
      try:
        task.update(userTime0_s, nSamples, firstSampleIndex)
        result = task.perform()
        import copy
        result = copy.deepcopy(result)
        self.queueOut.put(result)
      except:
        exType, exValue, exTraceback = sys.exc_info()
        traceback.print_exception(
            exType, exValue, exTraceback, file=sys.stderr)
        self.queueOut.put(None)
        self.queueIn.close()
        self.queueOut.close()
        sys.exit(1)
      duration_s = time.clock() - startTime_s
      self.totalExecTime_s += duration_s
    statistics = (self.totalWaitTime_s, self.totalExecTime_s)
    self.queueOut.put(statistics)
    self.queueIn.close()
    self.queueOut.close()
    sys.exit(0)


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
                    tcxo=None,
                    filterType="none",
                    logFile=None,
                    threadCount=1):
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
  tcxo : object, optional
    When specified, controls TCXO drift
  filterType : string, optional
    Controls IIR/FIR signal post-processing. Disabled by default.
  debugLog : bool, optional
    Control generation of additional debug output. Disabled by default.
  '''

  #
  # Print out parameters
  #
  print "Generating samples, sample rate={} Hz, interval={} seconds, SNR={}".format(
        outputConfig.SAMPLE_RATE_HZ, nSamples / outputConfig.SAMPLE_RATE_HZ, SNR)
  print "Jobs: ", threadCount

  _t0 = time.clock()
  _count = 0l

  # Check which bands are enabled, configure band-specific parameters
  bands = [outputConfig.GPS.L1, outputConfig.GPS.L2]  # Supported bands
  lpf = [None] * len(bands)
  bandsEnabled = [False] * len(bands)

  bandPass = False
  lowPass = False
  if filterType == 'lowpass':
    lowPass = True
  elif filterType == 'bandpass':
    bandPass = True
  elif filterType == 'none':
    pass
  else:
    raise ValueError("Invalid filter type %s" % repr(filter))

  for band in bands:
    for sv in sv_list:
      bandsEnabled[band.INDEX] |= sv.isBandEnabled(band.INDEX, outputConfig)

    filterObject = None
    if lowPass:
      filterObject = LowPassFilter(outputConfig,
                                   band.INTERMEDIATE_FREQUENCY_HZ)
    elif bandPass:
      filterObject = BandPassFilter(outputConfig,
                                    band.INTERMEDIATE_FREQUENCY_HZ)
    if filterObject:
      lpf[band.INDEX] = filterObject
      logger.debug("Band %d filter NBW is %f" %
                   (band.INDEX, filterObject.getNBW()))

  if SNR is not None:
    sourcePower = 0.
    for sv in sv_list:
      svMeanPower = sv.getAmplitude().computeMeanPower()
      sourcePower += svMeanPower
      logger.debug("[%s] Estimated mean power is %f" %
                   (sv.getSvName(), svMeanPower))
    meanPower = sourcePower / len(sv_list)
    meanAmplitude = scipy.sqrt(meanPower)
    logger.debug("Estimated total signal power is %f, mean %f, mean amplitude %f" %
                 (sourcePower, meanPower, meanAmplitude))

    # Nsigma and while noise amplitude computation: check if the Nsigma is
    # actually a correct value for white noise with normal distribution.
    Nsigma = scipy.sqrt(meanPower / (2. * 10. ** (float(SNR) / 10.)))
    logger.info("Selected noise sigma %f for SNR %f" % (Nsigma, float(SNR)))
    noisePower = scipy.square(Nsigma)

    for sv in sv_list:
      svMeanPower = sv.getAmplitude().computeMeanPower()
      svSNR = 10. * numpy.log10(svMeanPower / (sourcePower + noisePower) / 2.)
      logger.info("[%s] Estimated SNR is %f" % (sv.getSvName(), svSNR))
      if lpf[0]:
        nbwL1 = lpf[0].getNBW()
        svCNoL1 = svSNR + 10. * numpy.log10(2. * nbwL1)
        logger.info("[%s] Estimated CN0 for L1 is %f" %
                    (sv.getSvName(), svCNoL1))
      if lpf[1]:
        nbwL2 = lpf[1].getNBW()
        svCNoL2 = svSNR + 10. * numpy.log10(2. * nbwL2) - 3
        logger.info("[%s] Estimated CN0 for L2 is %f" %
                    (sv.getSvName(), svCNoL2))

  else:
    Nsigma = None
    logger.info("SNR is not provided, noise is not generated.")

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
    print "    .l1_message: {}".format(_sv.getL1CAMessage())
    print "    .l2_message: {}".format(_sv.getL2CMessage())
    print "    .l2_cl:      {}".format(_sv.getL2CLCodeType())
    print "  .doppler: {}".format(_sv.doppler)
    svMeanPower = _sv.getAmplitude().computeMeanPower()
    svSNR = 10. * numpy.log10(svMeanPower / (sourcePower + noisePower) / 2.)
    print "  .SNR: %f" % svSNR
    if lpf[0]:
      nbwL1 = lpf[0].getNBW()
      svCNoL1 = svSNR + 10. * numpy.log10(2. * nbwL1)
      print "  .L1 CNo: %f" % (svCNoL1)
    if lpf[1]:
      nbwL2 = lpf[1].getNBW()
      # CNo for L2: 20ms integration (+13dB), half power used (-3dB)
      svCNoL2 = svSNR + 10. * numpy.log10(2. * nbwL2) - 3 + 13
      print "  .L2 CNo: %f" % (svCNoL2)
    print "}"

  userTime_s = float(time0S)

  deltaUserTime_s = computeTimeIntervalS(outputConfig)
  debugFlag = logFile is not None

  if threadCount > 0:
    workerPool = [Worker(outputConfig,
                         sv_list,
                         Nsigma,
                         tcxo,
                         lpf,
                         debugFlag) for _ in range(threadCount)]

    for worker in workerPool:
      worker.start()
    maxTaskListSize = threadCount * 2
  else:
    workerPool = None
    task = Task(outputConfig,
                sv_list,
                noiseSigma=Nsigma,
                tcxo=tcxo,
                signalFilters=lpf,
                generateDebug=debugFlag)
    maxTaskListSize = 1

  workerPutIndex = 0
  workerGetIndex = 0
  activeTasks = 0

  totalSampleCounter = 0
  taskQueuedCounter = 0
  taskReceivedCounter = 0

  totalEncodeTime_s = 0.
  totalWaitTime_s = 0.

  while True:
    while activeTasks < maxTaskListSize and totalSampleCounter < nSamples:
      # We have space in the task backlog and not all batchIntervals are issued
      userTime0_s = userTime_s
      userTimeX_s = userTime_s + deltaUserTime_s
      sampleCount = outputConfig.SAMPLE_BATCH_SIZE

      if totalSampleCounter + sampleCount > nSamples:
        # Last interval may contain less than full batch size of samples
        sampleCount = nSamples - totalSampleCounter
        userTimeX_s = userTime0_s + float(sampleCount) / \
            outputConfig.SAMPLE_RATE_HZ

      params = (userTime0_s, sampleCount, totalSampleCounter)
      # print ">>> ", userTime0_s, sampleCount, totalSampleCounter,
      # workerPutIndex
      if workerPool is not None:
        workerPool[workerPutIndex].queueIn.put(params)
        workerPutIndex = (workerPutIndex + 1) % threadCount
      else:
        task.update(userTime0_s, sampleCount, totalSampleCounter)
      activeTasks += 1

      # Update parameters for the next batch interval
      userTime_s = userTimeX_s
      totalSampleCounter += sampleCount
      taskQueuedCounter += 1

    # What for the data only if we have something to wait
    if taskReceivedCounter == taskQueuedCounter and \
       totalSampleCounter == nSamples:
      # No more tasks to issue to generator
      # No more tasks to wait
      break

    try:
      if workerPool is not None:
        # Wait for the first task
        worker = workerPool[workerGetIndex]
        waitStartTime_s = time.time()
        # print "waiting data from worker", workerGetIndex
        result = worker.queueOut.get()
        # print "Data received from worker", workerGetIndex
        workerGetIndex = (workerGetIndex + 1) % threadCount
        waitDuration_s = time.time() - waitStartTime_s
        totalWaitTime_s += waitDuration_s
      else:
        result = task.perform()
    except:
      exType, exValue, exTraceback = sys.exc_info()
      traceback.print_exception(exType, exValue, exTraceback, file=sys.stderr)
      result = None
    taskReceivedCounter += 1
    activeTasks -= 1

    if result is None:
      print "Error in processor; aborting."
      break

    (inputParams, signalSamples, debugData) = result
    (_userTime0_s, _sampleCount, _firstSampleIndex) = inputParams
    # print "<<< ", _userTime0_s, _sampleCount, _firstSampleIndex

    if logFile is not None:
      # Data from all satellites is collected. Now we can dump the debug matrix

      userTimeAll_s = debugData[0]
      for smpl_idx in range(_sampleCount):
        logFile.write("{},{},".format(_firstSampleIndex + smpl_idx,
                                      userTimeAll_s[smpl_idx]))
        for svIdx in range(len(sv_list)):
          # signalSource = signalSources[svIdx]
          sv_bands = debugData[svIdx]
          for band in sv_bands:
            sv_sigs = band[0]
            doppler = band[1]
            idx = band[2]
            codes = band[3]
            logFile.write("{},{},{},{}".format(sv_sigs[smpl_idx],
                                               doppler[smpl_idx],
                                               idx[smpl_idx],
                                               codes[smpl_idx]))
        # End of line
        logFile.write("\n")

    encodeStartTime_s = time.time()
    # Feed data into encoder
    encodedSamples = encoder.addSamples(signalSamples)
    signalSamples = None

    if len(encodedSamples) > 0:
      _count += len(encodedSamples)
      encodedSamples.tofile(outputFile)
      encodedSamples = None
    encodeDuration_s = time.time() - encodeStartTime_s
    totalEncodeTime_s += encodeDuration_s

  print "MAIN: Encode duration:", totalEncodeTime_s
  print "MAIN: wait duration:", totalWaitTime_s

  encodedSamples = encoder.flush()
  if len(encodedSamples) > 0:
    encodedSamples.tofile(outputFile)

  if debugFlag:
    logFile.close()

  if workerPool is not None:
    for worker in workerPool:
      worker.queueIn.put(None)
    for worker in workerPool:
      try:
        statistics = worker.queueOut.get(timeout=2)
        print "Statistics:", statistics
      except:
        exType, exValue, exTraceback = sys.exc_info()
        traceback.print_exception(
            exType, exValue, exTraceback, file=sys.stderr)
      worker.queueIn.close()
      worker.queueOut.close()
      worker.terminate()
      worker.join()