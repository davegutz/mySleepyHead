# EyePatch: class to calculate filters installed in the SleepyHead device
# Copyright (C) 2025 Dave Gutz
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation;
# version 2.1 of the License.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# See http://www.fsf.org/licensing/licenses/lgpl.txt for full license text.

"""
Filter data and try candidate filters to detect sleepiness.
"""
from TFDelay import TFDelay
from myFilters import LagTustin, LagExp, General2Pole, RateLimit, SlidingDeadband, TustinIntegrator, RateLagExp

class Device:
    # Logic constants
    NOMINAL_DT = 0.1  #  From CONTROL_DELAY in SleepyHead (0.1)
    CLOSED_S = 1.0  # Voltage trip set persistence, s ()
    CLOSED_R = 0.5  # Voltage trip reset persistence, s ()


class EyePatch:
    """Container of candidate filters"""

    def __init__(self, data, dt=0.1):
        self.data = data
        self.voltFilter = General2Pole(Device.NOMINAL_DT, 0.1, 0.9, -5., 5.)  # actual dt provided at run time
        self.voltTripConf = TFDelay(False, Device.CLOSED_S, Device.CLOSED_R, Device.NOMINAL_DT)

    def calculate(self, init_time=-4.):
        """Filter data set and calculate candidate filter"""
        success = True
        return success
