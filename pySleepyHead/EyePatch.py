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
    NOMINAL_DT = 0.1  #  From CONTROL_DELAY in SleepyHead (0.1)
    CLOSED_S = 1.0  # Voltage trip set persistence, s ()
    CLOSED_R = 0.5  # Voltage trip reset persistence, s ()
    OMEGA_N_NOISE = 0.1  # Noise filter wn, r/s ()
    ZETA_NOISE = 0.9  # Noise filter damping factor ()
    V3V3Q2 = 3.3 / 2.  # Filter windup limits

class EyePatch:
    """Container of candidate filters"""

    def __init__(self, data, dt=0.1):
        self.Data = data
        self.voltFilter = General2Pole(Device.NOMINAL_DT, Device.OMEGA_N_NOISE, Device.ZETA_NOISE,
            0., Device.V3V3Q2)  # actual dt provided at run time
        self.voltTripConf = TFDelay(False, Device.CLOSED_S, Device.CLOSED_R, Device.NOMINAL_DT)
        self.time = None
        self.dt = None
        self.eye_voltage = None
        self.eye_voltage_filt = None
        self.eye_voltage_thr = None
        self.eye_cl = None
        self.conf = None
        self.buzz_eye = None
        self.saved = Saved()  # for plots and prints

    def calculate(self, init_time=-4., scale_in=None, verbose=True, t_max=None, unit=None):
        """Filter data set and calculate candidate filter"""
        t = self.Data.time
        if t_max is not None:
            t_delt = t - t[0]
            t = t[np.where(t_delt <= t_max)]
        volt_in = self.Data.eye_voltage
        t_len = len(t)

        # time loop
        now = t[0]
        for i in range(t_len):
            now = t[i]
            reset = (t[i] <= init_time) or (t[i] < 0. and t[0] > init_time)
            self.Data.i = i
            T = None
            if i == 0:
                T = t[1] - t[0]
            else:
                candidate_dt = t[i] - t[i - 1]
                if candidate_dt > 1e-6:
                    T = candidate_dt

            # Basic reset model verification is to init to the input data
            # Tried hard not to re-implement solvers in the Python verification  tool
            # Also, BTW, did not implement signal selection or tweak logic
            if reset:
                1 == 1  # place holder


            # Log
            mon.save(t[i], T, mon.soc, sim.voc)

            # Print initial
            if i == 0 and verbose:
                print('time=', t[i])
                print('mon:  ', str(mon))
            if verbose:
                print("{:9.3f}".format(t[i]), "{:4.0f}".format(self.Data.chm[i]),
                      "{:9.3f}".format(self.Data.ib[i]), "{:12.7f}".format(self.Data.soc[i]), "{:9.3f}".format(self.Data.dv_hys[i]),
                      "{:9.3f}".format(sim.saved.dv_hys[i]), "{:9.3f}".format(mon.saved.ib[i]), "{:12.7f}".format(mon.saved.soc[i]),
                      "{:4.0f}".format(mon.sat), "{:9.3f}".format(mon.saved.dv_hys[i]))
        # Data
        if verbose:
            print(
                '   time mo.chm so.chm so.ib_in_s so.dv_hys  mo.ib mo.soc mo.dv_hys   smv.ib_in_s sim.ibs sim.ioc sim.sat sim.dis sim.dv_dot smv.dv_hys  mv.ib  mv.soc mon.ibs  mon.ioc   mon.sat   mon.dis    mon.dv_dot  mv.dv_hys')
            print('time=', now)
            print('mon:  ', str(mon))

        return mon

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


