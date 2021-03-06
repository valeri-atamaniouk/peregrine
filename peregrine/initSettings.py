# Copyright (C) 2012 Swift Navigation Inc.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

import defaults

class initSettings:
  def __init__(self):

    self.msToProcess          = 39000                   # Number of ms of samples to perform tracking over (ms)
    self.skipNumberOfBytes    = 0                       # Skip bytes in sample file before loading samples for acquisition (bytes)
    self.IF                   = defaults.IF             # Intermediate frequency of signal in sample file (Hz)
    self.samplingFreq         = defaults.sampling_freq  # Sampling frequency of sample file (Hz)
    self.codeFreqBasis        = defaults.chipping_rate  # Frequency of chipping code (Hz)
    self.codeLength           = defaults.code_length    # Length of chipping code (chips)
    self.acqThreshold         = 21.0                    # SNR (unitless)
    self.acqSanityCheck       = True                    # Check for sats known to be below the horizon
    self.navSanityMaxResid    = 25.0                    # Meters per SV, normalized nav residuals (meters)
    self.abortIfInsane        = True                    # Abort the whole attempt if sanity check fails
    self.useCache             = True
    self.cacheDir             = 'cache'
    self.ephemMaxAge          = 4 * 3600.0              # Reject an ephemeris entry if older than this
