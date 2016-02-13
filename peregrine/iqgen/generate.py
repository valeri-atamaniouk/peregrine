# Copyright (C) 2016 Swift Navigation Inc.
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
from peregrine.iqgen.bits.low_pass_filter import LowPassFilter

from peregrine.iqgen.bits import signals

import scipy
import time

import threading
import Queue

class Task(object):
  '''
  Period computation task. This object performs a batch computation of signal
  in the specified range.
  '''

  def __init__(self,
               outputConfig,
               userTime0_s,
               nSamples,
               signalSources,
               noiseSigma=None,
               signalFilters=None,
               logFile=None,
               firstSampleIndex=None):
    '''
    Parameters
    ----------
    outputConfig : object
    userTime0_s : float
    nSamples : long
    signalSources : array-like
    noiseSigma : float
    signalFilters : array-like
    logFile : object
    firstSampleIndex : long
    '''

    self.queueOut = Queue.Queue(1)
    self.outputConfig = outputConfig
    self.signalSources = signalSources
    self.signalFilters = signalFilters
    self.logFile = logFile
    self.noiseSigma = noiseSigma
    self.signals = scipy.ndarray((4, outputConfig.SAMPLE_BATCH_SIZE), dtype=float)
    if noiseSigma is not None:
      # Initialize signal array with noise
      self.noise = noiseSigma * scipy.randn(4, outputConfig.SAMPLE_BATCH_SIZE)
    else:
      self.noise = None
    self.nSamples = outputConfig.SAMPLE_BATCH_SIZE
    self.update(userTime0_s, nSamples, firstSampleIndex)

  def update(self, userTime0_s, nSamples, firstSampleIndex=None):
    self.userTime0_s = userTime0_s
    self.firstSampleIndex = firstSampleIndex

    if (self.nSamples != nSamples):
      self.signals.resize((4, nSamples))
      if self.noiseSigma is not None:
        # Initialize signal array with noise
        self.noise = self.noiseSigma * scipy.randn(4, nSamples)
      self.nSamples = nSamples

  def perform(self):
    outputConfig = self.outputConfig
    signalSources = self.signalSources
    signalFilters = self.signalFilters

    logFile = self.logFile

    userTime0_s = self.userTime0_s
    userTimeX_s = userTime0_s + float(self.nSamples) / \
                                float(outputConfig.SAMPLE_RATE_HZ)

    userTimeAll_s = scipy.linspace(userTime0_s,
                                   userTimeX_s,
                                   self.nSamples,
                                   endpoint=False)

    noise = self.noise
    sigs = self.signals
    sigs.fill(0.)
    if noise is not None:
      # Initialize signal array with noise
      sigs += noise

    if logFile is not None:
      debugData = []

    # Sum up signals for all SVs
    for signalSource in signalSources:
      # Add signal from source (satellite) to signal accumulator
      t = signalSource.getBatchSignals(userTimeAll_s,
                                       sigs,
                                       outputConfig)
      # Debugging output
      if logFile is not None:
        debugData.append(t)
      t = None

    if logFile is not None:
      # Data from all satellites is collected. Now we can dump the debug matrix

      totalSampleCounter = self.firstSampleIndex
      for smpl_idx in range(len(userTimeAll_s)):
        logFile.write("{},{},".format(totalSampleCounter,
                                       userTimeAll_s[smpl_idx]))
        for svIdx in range(len(signalSources)):
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
        totalSampleCounter += 1
      logFile.flush()  # Flush the batch data into file

    if signalFilters is list:
      # Filter signal values through LPF, BPF or another
      for i in range(len(self.filters)):
        filterObject = signalFilters[i]
        if filterObject is not None:
          sigs[i][:] = filterObject.filter(sigs[i])

    self.queueOut.put(sigs)

  def reportError(self):
    self.queueOut.put(None)

class Worker(threading.Thread):

  def __init__(self, queueIn):
    super(Worker, self).__init__()
    self.queueIn = queueIn
    self.daemon = True

  def run(self):
    while True:
      task = self.queueIn.get()
      if task is None:
        break
      try:
        task.perform()
      except:
        exType, exValue, exTraceback = sys.exc_info()
        traceback.print_exception(exType, exValue, exTraceback, file=sys.stderr)
        task.reportError()

class WorkerPool(object):
  def __init__(self, threadCount):
    self.taskQueue = Queue.Queue()
    self.workers = [Worker(self.taskQueue) for _ in range(threadCount)]
    for worker in self.workers:
      worker.start()

  def finish(self):
    while True:
      try:
        task = self.taskQueue.get_nowait()
      except Queue.Empty:
        task = None
      if task is None:
        break
      else:
        print "Cancelling task: ", task
        task.queueOut.put(None)
    for _ in range(len(self.workers)):
      self.taskQueue.put(None)
    for worker in self.workers:
      worker.join()

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
                    debugLog=False,
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

  if debugLog:
    _out_txt = open("out.txt", "wt");
  else:
    _out_txt = None

  userTime_s = float(time0S)
  oldPerformanceCounter = 0

  deltaUserTime_s = computeTimeIntervalS(outputConfig)

  workerPool = WorkerPool(threadCount)

  maxTaskListSize = threadCount * 2
  activeTaskList = []
  recycledTaskList = []
  totalSampleCounter = 0
  taskQueuedCounter = 0
  taskReceivedCounter = 0

  while True:
    while len(activeTaskList) < maxTaskListSize and totalSampleCounter < nSamples:
      # We have space in the task backlog and not all batchIntervals are issued
      userTime0_s = userTime_s
      userTimeX_s = userTime_s + deltaUserTime_s
      sampleCount = outputConfig.SAMPLE_BATCH_SIZE

      if totalSampleCounter + sampleCount > nSamples:
        # Last interval may contain less than full batch size of samples
        sampleCount = nSamples - totalSampleCounter
        userTimeX_s = userTime0_s + float(sampleCount) / \
                                    outputConfig.SAMPLE_RATE_HZ
      if not recycledTaskList:
        # Create new task object only when there is no available object in the
        # pool
        task = Task(outputConfig,
                    userTime0_s,
                    sampleCount,
                    sv_list,
                    noiseSigma=Nsigma,
                    signalFilters=lpf,
                    logFile=_out_txt,
                    firstSampleIndex=totalSampleCounter)
      else:
        # Reuse object from the pool
        task = recycledTaskList.pop()
        task.update(userTime0_s, sampleCount, totalSampleCounter)

      # Update parameters for the next batch interval
      userTime_s = userTimeX_s
      totalSampleCounter += sampleCount
      activeTaskList.append(task)
      workerPool.taskQueue.put(task)
      taskQueuedCounter += 1

    # What for the data only if we have something to wait
    if taskReceivedCounter == taskQueuedCounter and \
       totalSampleCounter == nSamples:
      # No more tasks to issue to generator
      # No more tasks to wait
      break

    # Wait for the first task
    task = activeTaskList.pop(0)
    signalSamples = task.queueOut.get()
    recycledTaskList.append(task)
    taskReceivedCounter += 1

    if signalSamples is None:
      print "Error in processor; aborting."
      break

    # Feed data into encoder
    encodedSamples = encoder.addSamples(signalSamples)
    signalSamples = None

    if len(encodedSamples) > 0:
      _count += len(encodedSamples)
      encodedSamples.tofile(outputFile)
      encodedSamples = None

  encodedSamples = encoder.flush()
  if len(encodedSamples) > 0:
    encodedSamples.tofile(outputFile)

  if (debugLog): _out_txt.close()

  workerPool.finish()
