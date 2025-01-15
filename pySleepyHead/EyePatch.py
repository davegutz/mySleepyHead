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
import numpy as np


class Device:
    # Logic constants
    NOMINAL_DT = 0.1  #  From CONTROL_DELAY in SleepyHead (0.1)d
    VOLT_CLOSED_S = 0.5  # Voltage trip set persistence, s ()
    VOLT_CLOSED_R = 0.2  # Voltage trip reset persistence, s ()
    OMEGA_N_NOISE = 5.  # Noise filter wn, r/s ()
    ZETA_NOISE = 0.9  # Noise filter damping factor ()
    V3V3Q2 = 3.3 / 2.  # Filter windup limits
    EYE_CL_THR = 1.3

class EyePatch:
    """Container of candidate filters"""

    def __init__(self, data, dt=0.1):
        self.Data = data
        self.VoltFilter = General2Pole(Device.NOMINAL_DT, Device.OMEGA_N_NOISE, Device.ZETA_NOISE, -10., 10., 0., Device.V3V3Q2)  # actual dt provided at run time
        # self.VoltFilter = LagExp(Device.NOMINAL_DT, 1./Device.OMEGA_N_NOISE,0., Device.V3V3Q2)  # actual dt provided at run time
        self.VoltTripConf = TFDelay(False, Device.VOLT_CLOSED_S, Device.VOLT_CLOSED_R, Device.NOMINAL_DT)
        self.time = None
        self.dt = None
        self.eye_voltage = None
        self.eye_voltage_filt = None
        self.eye_voltage_thr = None
        self.eye_cl = None
        self.conf = None
        self.buzz_eye = None
        self.saved = Saved()  # for plots and prints

    def calculate(self, init_time=-4., verbose=True, t_max=None, unit=None):
        """Filter data set and calculate candidate filter"""
        t = self.Data.time
        if t_max is not None:
            t_delt = t - t[0]
            t = t[np.where(t_delt <= t_max)]
        t_len = len(t)

        # time loop
        now = t[0]
        for i in range(t_len):
            now = t[i]
            reset = (t[i] <= init_time) or (t[i] < 0. and t[0] > init_time)
            self.Data.i = i
            self.time = now
            self.eye_voltage = self.Data.eye_voltage[i]
            T = None
            if i == 0:
                T = t[1] - t[0]
            else:
                candidate_dt = t[i] - t[i - 1]
                if candidate_dt > 1e-6:
                    T = candidate_dt

            # Run filters
            self.eye_voltage_filt = self.VoltFilter.calculate(self.eye_voltage, reset, T)
            self.eye_cl = self.eye_voltage_filt < Device.EYE_CL_THR
            self.conf = self.VoltTripConf.calculate(self.eye_cl, Device.VOLT_CLOSED_S, Device.VOLT_CLOSED_R, T, reset)

            # Log
            self.save(t[i], T)

            # Print initial
            if i == 0 and verbose:
                print('time=', t[i])
                print(' object   T  reset  time   eye_voltage  filt_dt filt_reset eye_voltage_filt  filt_a  filt_b  filt_in filt_out')
            if verbose:
                # print('EyePatch:  ', "{:8.6f}".format(T), "  ", reset, str(self))
                # print('EyePatch:  ', "{:8.6f}".format(T), "  ", reset, str(self), repr(self.VoltFilter.AB2), repr(self.VoltFilter.Tustin))
                # print('EyePatch:  ', "{:8.6f}".format(T), "  ", reset, repr(self.VoltFilter.AB2))
                print('EyePatch:  ', "{:8.6f}".format(T), "  ", reset, repr(self.VoltTripConf), "{:2d}".format(self.conf))

        # Data
        if verbose:
            print('   time mo.eye_voltage ')
            print('time=', now)
            print('EyePatch:  ', str(self))

        return self.saved

    def save(self, time, dt):  # Filter
        """Log EyePatch"""
        self.saved.time.append(self.time)
        self.saved.dt.append(self.dt)
        self.saved.eye_voltage.append(self.eye_voltage)
        self.saved.eye_voltage_filt.append(self.eye_voltage_filt)
        self.saved.eye_voltage_thr.append(self.eye_voltage_thr)
        self.saved.eye_cl.append(self.eye_cl)
        self.saved.conf.append(self.conf)
        self.saved.buzz_eye.append(self.buzz_eye)

    def __str__(self):
        return "{:9.3f}".format(self.time) + "{:9.3f}".format(self.eye_voltage) + "{:9.3f}".format(self.eye_voltage_filt)

class Saved:
    # For plot savings.   A better way is 'Saver' class in pyfilter helpers and requires making a __dict__
    def __init__(self):
        self.time = []
        self.dt = []
        self.eye_voltage = []
        self.eye_voltage_filt = []
        self.eye_voltage_thr = []
        self.eye_cl = []
        self.conf = []
        self.buzz_eye = []


