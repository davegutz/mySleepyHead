# Sensors: class to calculate filters installed in the SleepyHead device
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
from myFilters import General2Pole, LongTermShortTermFilter, LagExp, RateLagExp
from MahonyAHRS import MahonyAHRS
from TFDelay import TFDelay
from Wag import Wag
import numpy as np


class Device:
    # Logic constants
    NOMINAL_DT = 0.1  #  From CONTROL_DELAY in SleepyHead (0.1)
    MAX_DT_EYE = 0.1  # Maximum filter update time in call to prevent aliasing, sec (0.1)
    MAX_DT_HEAD = 0.2  # Maximum filter update time in call to prevent aliasing, sec (0.2)
    HEAD_S = 0.04  # Persistence head sense set, sec (0.04)  Head needs little; heavily filtered by Mahony
    HEAD_R = 0.04  # Persistence head sense reset, sec (0.04)
    TAU_ST = 0.4  # Short term filter time constant, sec (0.4)
    TAU_LT = 10.  # Long term filter time constant, sec (10.)
    FLT_THR_NEG = -1.3e6  # hardcoded in C++
    FRZ_THR_NEG = -0.3e6  # hardcoded in C++
    FLT_THR_POS = 0.2  # LTST filter positive dltst fault threshold, v (0.2)
    FRZ_THR_POS = 0.03  # LTST filter positive dltst freeze threshold, v (0.03)
    G_MAX = 4.  # Max G value, g's (4.)
    W_MAX = 34.9  # Max G value, g's (34.9)
    D_MAX = 2000.  # Max rotational value, deg/s (34.9*180/pi) limit of hardware
    TAU_E_FILT = 0.1  # Tau rate filter, sec (0.1)
    TAU_FILT = 0.1  # Tau filter, sec (0.1)
    TAU_HEAD_RATE_FILT = 0.2  # Tau filter, sec (0.1)
    WN_Q_FILT = 5.  # Quiet filter-2 natural frequency, r/s (5.)
    ZETA_Q_FILT = 0.9  # Quiet fiter-2 damping factor (0.9)
    TAU_Q_FILT = 0.1  # Quiet rate time constant, sec (0.1)
    QUIET_S = 0.4  # Quiet set persistence, sec (0.4)
    R_SCL = 10.  # Quiet reset persistence scalar on QUIET_S ('up 1 down 10')
    QUIET_R = QUIET_S / R_SCL  # Quiet reset persistence, calculated
    kp_def = 10.  # Proportional gain Kp (10.0)
    ki_def = 2.  # Integral gain Ki (2.0)
    G_QUIET_THR = 0.06  # g's quiet detection threshold, small is more sensitive (0.06)
    O_QUIET_THR = 45.8  # rps quiet detection threshold, small is more sensitive (0.4)
    EYE_S = 1.7  # Persistence eye closed IR sense set, sec (1.7)
    EYE_R = 0.5  # Persistence eye closed IR sense reset, sec (0.5)
    OFF_S = 0.04  # Persistence glasses off IR sense set, sec (0.04)
    OFF_R = 8.  # Persistence glasses off IR sense reset, sec (8.0)
    GLASSES_OFF_VOLTAGE = 2.8  # Glasses off voltage, V (2.8) above this value assumed off and reset until clear for 3 seconds (user reset)
    SHAKE_S = 0.2  # Persistence head shake motion sense set, sec (0.2) update time is 0.1
    SHAKE_R = 4.0  # Persistence head shake motion sense reset, sec (4.0)
    pitch_thr_def_forte = 17.   # Threshold sleep detect screech (17.), deg
    roll_thr_def_forte = 17.  # Threshold sleep detect screech (17.), deg
    pitch_thr_def_piano = 12.  # Threshold sleep detect buzz only (12.), deg
    roll_thr_def_piano = 12.  # Threshold sleep detect buzz only (12.), deg
    v3v3_nom = 3.3  # IR detector power supply, v (3.3)
    YAW_SET = 0.2  # Persistence to detect yaw motion, sec (0.2)
    YAW_HOLD_1 = 3.0  # Persistence of first yaw motion in in_1 direction to allow others to set, sec (3.0)
    YAW_HOLD_2 = 2.0  # Persistence of second yaw motion in in_2 direction to allow final to set, sec (2.0)
    YAW_HOLD_3 = 1.0  # Persistence of final yaw motion in in_1 confirmation direction to allow downstream timers to set, sec (1.0)
    YAW_RATE_LOW = -60.  # Yaw rate to declare right motion detected, deg/sec (-25.)
    YAW_RATE_HIGH = 60.  # Yaw rat to declare left motion detected, deg/sec (25.)
    YAW_RESET_S = 0.2  # Persistence of reset flag to latch in, sec (0.2)
    YAW_RESET_R = 5.0  # Hold time of yaw head wag reset used to reset eye filters and silence alarm temporarily, sec (5.0)
    YAW_WRAP_DETECT = 180.  # Abs value of yaw change since last update that indicates yaw_deg has wrapped around, deg (180.)
    YAW_WRAP_MAG = 360.  # Abs value of wrap to apply when wrap detected.  If incoming signal is 0-360 this should be 360.  (360.)
    EYE_INIT_TIME = 20.  # Eye init time based on elapsed time, sec (20.)


class Sensors:
    """Container of candidate filters"""

    def __init__(self, data, dt=0.1):
        self.Data = data

        # Head filters
        self.X_Filt = LagExp(Device.NOMINAL_DT, Device.TAU_FILT, -Device.G_MAX, Device.G_MAX)
        self.Y_Filt = LagExp(Device.NOMINAL_DT, Device.TAU_FILT, -Device.G_MAX, Device.G_MAX)
        self.Z_Filt = LagExp(Device.NOMINAL_DT, Device.TAU_FILT, -Device.G_MAX, Device.G_MAX)
        self.G_Filt = LagExp(Device.NOMINAL_DT, Device.TAU_FILT, -Device.G_MAX, Device.G_MAX)
        self.GQuietFilt = General2Pole(Device.NOMINAL_DT, Device.WN_Q_FILT, Device.ZETA_Q_FILT,
                                       min_=-Device.G_MAX, max_=Device.G_MAX)
        self.GQuietRate = RateLagExp(Device.NOMINAL_DT, Device.TAU_Q_FILT, -Device.G_MAX, Device.G_MAX)
        self.GQuietPer = TFDelay(True, Device.QUIET_S, Device.QUIET_R, Device.NOMINAL_DT)
        self.A_Filt = LagExp(Device.NOMINAL_DT, Device.TAU_FILT, -Device.D_MAX, Device.D_MAX)
        self.B_Filt = LagExp(Device.NOMINAL_DT, Device.TAU_FILT, -Device.D_MAX, Device.D_MAX)
        self.C_Filt = LagExp(Device.NOMINAL_DT, Device.TAU_FILT, -Device.D_MAX, Device.D_MAX)
        self.O_Filt = LagExp(Device.NOMINAL_DT, Device.TAU_FILT, -Device.D_MAX, Device.D_MAX)
        self.OQuietFilt = General2Pole(Device.NOMINAL_DT, Device.WN_Q_FILT, Device.ZETA_Q_FILT,
                                       min_=-Device.D_MAX, max_=Device.D_MAX)
        self.OQuietRate = RateLagExp(Device.NOMINAL_DT, Device.TAU_Q_FILT, -Device.D_MAX, Device.D_MAX)
        self.OQuietPer = TFDelay(True, Device.QUIET_S, Device.QUIET_R, Device.NOMINAL_DT)
        self.TrackFilter = MahonyAHRS(self.Data, sample_period=Device.NOMINAL_DT, Kp=Device.kp_def,
                                      Ki=Device.ki_def)
        self.yaw_LRL_detect = Wag(init=False, nom_dt=Device.NOMINAL_DT, set_all=Device.YAW_SET,
                                    hold_1=Device.YAW_HOLD_1, hold_2=Device.YAW_HOLD_2, hold_3=Device.YAW_HOLD_3)
        self.yaw_RLR_detect = Wag(init=False, nom_dt=Device.NOMINAL_DT, set_all=Device.YAW_SET,
                                    hold_1=Device.YAW_HOLD_1, hold_2=Device.YAW_HOLD_2, hold_3=Device.YAW_HOLD_3)
        self.YawResetPer = TFDelay(False, Device.YAW_RESET_S, Device.YAW_RESET_R, Device.NOMINAL_DT)

        # Eye filters
        self.LTST_Filter = LongTermShortTermFilter(dt, tau_lt=Device.TAU_LT, tau_st=Device.TAU_ST,
                                                   flt_thr_neg=Device.FLT_THR_NEG, frz_thr_neg=Device.FRZ_THR_NEG,
                                                   flt_thr_pos=Device.FLT_THR_POS, frz_thr_pos=Device.FRZ_THR_POS)
        self.HeadNodPerF = TFDelay(True, Device.EYE_S, Device.EYE_R, Device.NOMINAL_DT)
        self.HeadNodPerP = TFDelay(True, Device.EYE_S, Device.EYE_R, Device.NOMINAL_DT)
        self.EyeClosedPer = TFDelay(False, Device.EYE_S, Device.EYE_R, Device.NOMINAL_DT)
        self.GlassesOffPer = TFDelay(True, Device.OFF_S, Device.OFF_R, Device.NOMINAL_DT)
        self.HeadShakePer = TFDelay(False, Device.SHAKE_S, Device.SHAKE_R, Device.NOMINAL_DT)

        # Rates
        self.EyeRateFilt = RateLagExp(Device.NOMINAL_DT, Device.TAU_E_FILT, -Device.v3v3_nom, Device.v3v3_nom)
        self.RollRateFilt = RateLagExp(Device.NOMINAL_DT, Device.TAU_HEAD_RATE_FILT, -Device.D_MAX, Device.D_MAX)
        self.PitchRateFilt = RateLagExp(Device.NOMINAL_DT, Device.TAU_HEAD_RATE_FILT, -Device.D_MAX, Device.D_MAX)
        self.YawRateFilt = RateLagExp(Device.NOMINAL_DT, Device.TAU_HEAD_RATE_FILT, -Device.D_MAX, Device.D_MAX)

        # data
        self.time = None
        self.T = None
        self.reset = True
        self.eye_reset = None
        self.head_reset = None
        self.eye_voltage_norm = None
        self.eye_voltage_filt = None
        self.eye_voltage_flt = None
        self.a_raw = None
        self.b_raw = None
        self.c_raw = None
        self.o_raw = None
        self.a_filt = None
        self.b_filt = None
        self.c_filt = None
        self.o_filt = None
        self.o_qrate = None
        self.o_quiet = None
        self.o_is_quiet = None
        self.o_is_quiet_sure = None
        self.x_raw = None
        self.y_raw = None
        self.z_raw = None
        self.g_raw = None
        self.x_filt = None
        self.y_filt = None
        self.z_filt = None
        self.g_filt = None
        self.g_qrate = None
        self.g_quiet = None
        self.g_is_quiet = None
        self.g_is_quiet_sure = None
        self.max_nod_f = None
        self.max_nod_f_confirmed = None
        self.max_nod_p = None
        self.max_nod_p_confirmed = None
        self.head_buzz_f = None
        self.head_buzz_p = None
        self.eye_closed = None
        self.eye_closed_confirmed = None
        self.flt_LTST = None
        self.eye_buzz = None
        self.cf = 1.
        self.dltst = None
        self.fault = False
        self.freeze = False
        self.input = None
        self.lt_state = None
        self.st_state = None
        self.frz_thr_pos = Device.FRZ_THR_POS
        self.flt_thr_pos = Device.FLT_THR_POS
        self.roll_deg = None
        self.pitch_deg = None
        self.yaw_deg = None
        self.G_QUIET_THR = None
        self.O_QUIET_THR = None
        self.eye_rate = None
        self.roll_rate = None
        self.pitch_rate = None
        self.yaw_rate = None
        self.eye_reset_LRL = None
        self.eye_reset_RLR = None
        self.yaw_eye_reset = None
        self.saved = Saved()  # for plots and prints

    def calculate(self, init_time=-4., verbose=True, t_max=None, use_eye_reset_in=False):
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
            elapsed_time = self.Data.elapsed_time[i]
            self.reset = (t[i] <= init_time) or (t[i] < 0. and t[0] > init_time)
            self.Data.i = i
            self.time = now

            # Inputs
            self.eye_voltage_norm = self.Data.eye_voltage_norm[i]
            self.a_raw = self.Data.a_raw[i]
            self.b_raw = self.Data.b_raw[i]
            self.c_raw = self.Data.c_raw[i]
            self.o_raw = np.sqrt(self.a_raw*self.a_raw + self.b_raw*self.b_raw + self.c_raw*self.c_raw)
            self.x_raw = self.Data.x_raw[i]
            self.y_raw = self.Data.y_raw[i]
            self.z_raw = self.Data.z_raw[i]
            self.g_raw = np.sqrt(self.x_raw*self.x_raw + self.y_raw*self.y_raw + self.z_raw*self.z_raw)
            self.pitch_deg = self.Data.pitch_deg[i]
            self.roll_deg = self.Data.roll_deg[i]
            self.yaw_deg = self.Data.yaw_deg[i]

            # Update time
            self.T = None
            if i == 0:
                self.T = t[1] - t[0]
            else:
                candidate_dt = t[i] - t[i - 1]
                if candidate_dt > 1e-6:
                    self.T = candidate_dt
                else:
                    self.T = Device.NOMINAL_DT
            reset_eye = True
            if elapsed_time >= Device.EYE_INIT_TIME:
                reset_eye = False
            # Run filters
            delta_pitch = self.Data.delta_pitch[i]
            delta_roll = self.Data.delta_roll[i]
            if use_eye_reset_in is True:
                self.filter_eye(reset_eye, self.Data.eye_reset[i])
            else:
                self.filter_eye(reset_eye)
            self.filter_head(self.reset, delta_pitch, delta_roll)
            self.quiet_decisions(self.reset)

            # Log
            self.save(t[i], self.T)

            # Print initial
            if i == 0 and verbose:
                print('time=', t[i], end='')
                print(' <header> ')
            if verbose:
                # print('Sensors:  ', "{:8.6f}".format(T), "  ", self.reset, str(self))
                # print('Sensors:  ', "{:8.6f}".format(T), "  ", self.reset, str(self), repr(self.VoltFilter.AB2), repr(self.VoltFilter.Tustin))
                # print('Sensors:  ', "{:8.6f}".format(T), "  ", self.reset, repr(self.VoltFilter.AB2))
                # print('Sensors:  ', "{:8.6f}".format(T), "  ", self.reset, repr(self.VoltTripConf), "{:2d}".format(self.eye_closed_confirmed))
                # print("{:9.6}  ".format(self.time), repr(self.LTST_Filter), "eye_closed {:d}".format(self.eye_closed))
                # print("{:9.6}  ".format(self.time), "yaw_rate {:9.6f}".format(self.Data.yaw_rate[i]), "yaw_rate_ver {:9.6f}".format(self.yaw_rate))
                # self.TrackFilter.pp8()
                pass
                # print("{:9.6}  ".format(self.time),
                #       "voltage {:f} {:f}".format(self.Data.eye_voltage_norm[i], self.eye_voltage_norm),
                #       "yaw_rate {:f} {:f}".format(self.Data.yaw_rate[i], self.yaw_rate),
                #       "glasses_off_ver {:d}".format(self.eye_voltage_norm > Device.GLASSES_OFF_VOLTAGE),
                #       "reset_RLR_ver {:d}".format(self.eye_reset_RLR),
                #       "reset_LRL_ver {:d}".format(self.eye_reset_LRL),
                #       "eye_reset {:d} {:d}".format(int(self.Data.eye_reset[i]), self.eye_reset))

        # Data
        if verbose:
            print('   time mo.eye_voltage_norm ')
            print('time=', now)
            print('Sensors:  ', str(self.LTST_Filter))

        return self.saved

    def filter_eye(self, reset, eye_reset_in=None):
        glasses_off = self.eye_voltage_norm > Device.GLASSES_OFF_VOLTAGE
        if glasses_off or self.eye_reset_RLR or self.eye_reset_LRL:
            glasses_reset = True
        else:
            glasses_reset = False
        if eye_reset_in is None:
            self.eye_reset = reset or self.GlassesOffPer.calculate(glasses_reset, Device.OFF_S, Device.OFF_R, self.T, reset)
        else:
            self.eye_reset = bool(eye_reset_in)
        self.eye_closed = self.LTST_Filter.calculate(self.eye_voltage_norm, self.eye_reset,
                                                     min(self.T, Device.MAX_DT_EYE), -Device.FRZ_THR_POS)
        self.eye_closed_confirmed = self.EyeClosedPer.calculate(self.eye_closed, Device.EYE_S, Device.EYE_R,
                                                                self.T, self.eye_reset)
        self.eye_buzz = self.eye_closed_confirmed
        self.eye_rate = self.EyeRateFilt.calculate(self.eye_voltage_norm, reset, min(self.T, Device.MAX_DT_EYE))
        # print("{:7.3f} ".format(self.eye_rate), end='')
        # self.EyeRateFilt.__repr__()

    def filter_head(self, reset, delta_pitch=0., delta_roll=0.):
        # Gs
        self.x_filt = self.X_Filt.calculate_tau(self.x_raw, reset, Device.TAU_FILT, min(self.T, Device.MAX_DT_HEAD) )
        self.y_filt = self.Y_Filt.calculate_tau(self.y_raw, reset, Device.TAU_FILT, min(self.T, Device.MAX_DT_HEAD) )
        self.z_filt = self.Z_Filt.calculate_tau(self.z_raw, reset, Device.TAU_FILT, min(self.T, Device.MAX_DT_HEAD) )
        self.g_filt = self.G_Filt.calculate_tau(self.g_raw, reset, Device.TAU_FILT, min(self.T, Device.MAX_DT_HEAD) )
        # Cannot run these at 0.1 seconds.  They are run at 0.02 in application
        self.g_qrate = self.GQuietRate.calculate(self.g_raw-1., reset, min(self.T, Device.MAX_DT_HEAD))
        self.g_quiet = self.GQuietFilt.calculate(self.g_qrate, reset, min(self.T, Device.MAX_DT_HEAD))

        # Angles
        self.a_filt = self.A_Filt.calculate_tau(self.a_raw, reset, Device.TAU_FILT, min(self.T, Device.MAX_DT_HEAD) )
        self.b_filt = self.B_Filt.calculate_tau(self.b_raw, reset, Device.TAU_FILT, min(self.T, Device.MAX_DT_HEAD) )
        self.c_filt = self.C_Filt.calculate_tau(self.c_raw, reset, Device.TAU_FILT, min(self.T, Device.MAX_DT_HEAD) )
        self.o_filt = self.O_Filt.calculate_tau(self.o_raw, reset, Device.TAU_FILT, min(self.T, Device.MAX_DT_HEAD) )
        self.o_qrate = self.OQuietRate.calculate(self.o_raw-1., reset, min(self.T, Device.MAX_DT_HEAD))
        self.o_quiet = self.OQuietFilt.calculate(self.o_qrate, reset, min(self.T, Device.MAX_DT_HEAD))

        self.TrackFilter.updateIMU(gyroscope_=np.array([self.a_raw, self.b_raw, self.c_raw]),
                                   accelerometer=np.array([self.x_raw, self.y_raw, self.z_raw]),
                                   sample_time=self.T, reset=reset)
        self.roll_deg = self.TrackFilter.roll_deg + delta_roll
        self.pitch_deg = self.TrackFilter.pitch_deg + delta_pitch
        self.yaw_deg = self.TrackFilter.yaw_deg
        self.G_QUIET_THR = Device.G_QUIET_THR
        self.O_QUIET_THR = Device.O_QUIET_THR

        # Rates
        self.roll_rate = self.RollRateFilt.calculate(self.roll_deg, reset, min(self.T, Device.MAX_DT_HEAD))
        self.pitch_rate = self.PitchRateFilt.calculate(self.pitch_deg, reset, min(self.T, Device.MAX_DT_HEAD))
        self.yaw_rate = self.YawRateFilt.calculate(self.yaw_deg, reset, min(self.T, Device.MAX_DT_HEAD),
                                                   wrap_detect=Device.YAW_WRAP_DETECT, wrap_mag_=Device.YAW_WRAP_MAG)
        # print(self.YawRateFilt.__repr__())

        # Head nod
        self.max_nod_f = max( abs(self.pitch_deg)- Device.pitch_thr_def_forte,
                              abs(self.roll_deg) - Device.roll_thr_def_forte )
        self.max_nod_p = max( abs(self.pitch_deg)- Device.pitch_thr_def_piano,
                              abs(self.roll_deg) - Device.roll_thr_def_piano )
        self.head_reset = reset or self.HeadShakePer.calculate( (not self.o_is_quiet_sure and not self.g_is_quiet_sure),
                                                                Device.SHAKE_S, Device.SHAKE_R, self.T, reset )
        self.max_nod_f_confirmed = self.HeadNodPerF.calculate( self.max_nod_f > 0 and not self.head_reset,
                                                               Device.HEAD_S, Device.HEAD_R, self.T, reset)
        self.max_nod_p_confirmed = self.HeadNodPerP.calculate( self.max_nod_p > 0 and not self.head_reset,
                                                               Device.HEAD_S, Device.HEAD_R, self.T, reset)

        # Head buzz
        self.head_buzz_f = self.max_nod_f_confirmed
        self.head_buzz_p = self.max_nod_p_confirmed

        # Yaw reset used to reset eye logic
        yaw_rate_right = self.yaw_rate <= Device.YAW_RATE_LOW
        yaw_rate_left = self.yaw_rate >= Device.YAW_RATE_HIGH
        self.eye_reset_LRL = self.yaw_LRL_detect.calculate(reset=self.reset, dt=self.T, in_1=yaw_rate_left,
                                                           in_2=yaw_rate_right)
        self.eye_reset_RLR = self.yaw_RLR_detect.calculate(reset=self.reset, dt=self.T, in_1=yaw_rate_right,
                                                           in_2=yaw_rate_left)
        self.yaw_eye_reset = self.YawResetPer.calculate(self.eye_reset_LRL or self.eye_reset_RLR, Device.YAW_RESET_S,
                                                        Device.YAW_RESET_R, self.T, reset)

    def quiet_decisions(self, reset):
        self.o_is_quiet = abs(self.o_quiet) <= Device.O_QUIET_THR
        self.o_is_quiet_sure = self.OQuietPer.calculate(self.o_is_quiet, Device.QUIET_S, Device.QUIET_R, self.T, reset)
        self.g_is_quiet = abs(self.g_quiet) <= Device.G_QUIET_THR
        self.g_is_quiet_sure = self.GQuietPer.calculate(self.g_is_quiet, Device.QUIET_S, Device.QUIET_R, self.T, reset)

    def save(self, time, dt):  # Filter
        """Log Sensors"""
        self.saved.reset.append(self.reset)
        self.saved.time.append(time)
        self.saved.head_reset.append(self.head_reset)
        self.saved.eye_reset.append(self.eye_reset)
        self.saved.T.append(dt)
        self.saved.eye_voltage_norm.append(self.eye_voltage_norm)
        self.saved.a_raw.append(self.a_raw)
        self.saved.b_raw.append(self.b_raw)
        self.saved.c_raw.append(self.c_raw)
        self.saved.o_raw.append(self.o_raw)
        self.saved.x_raw.append(self.x_raw)
        self.saved.y_raw.append(self.y_raw)
        self.saved.z_raw.append(self.z_raw)
        self.saved.g_raw.append(self.g_raw)
        self.saved.eye_voltage_filt.append(self.eye_voltage_filt)
        self.saved.eye_voltage_flt.append(self.eye_voltage_flt)
        self.saved.eye_closed.append(self.eye_closed)
        self.saved.eye_closed_confirmed.append(self.eye_closed_confirmed)
        self.saved.eye_buzz.append(self.eye_buzz)
        self.saved.flt_LTST.append(self.flt_LTST)
        self.saved.cf.append(self.LTST_Filter.cf)
        self.saved.dltst.append(self.LTST_Filter.dltst)
        self.saved.freeze.append(self.LTST_Filter.freeze)
        self.saved.lt_state.append(self.LTST_Filter.lt_state)
        self.saved.st_state.append(self.LTST_Filter.st_state)
        self.saved.frz_thr_pos.append(Device.FRZ_THR_POS)
        self.saved.flt_thr_pos.append(Device.FLT_THR_POS)
        self.saved.max_nod_f.append(self.max_nod_f)
        self.saved.max_nod_f_confirmed.append(self.max_nod_f_confirmed)
        self.saved.max_nod_p.append(self.max_nod_p)
        self.saved.max_nod_p_confirmed.append(self.max_nod_p_confirmed)
        self.saved.head_buzz_f.append(self.head_buzz_f)
        self.saved.head_buzz_p.append(self.head_buzz_p)
        self.saved.head_buzz.append(self.head_buzz_f)  # yes, that's right
        self.saved.pitch_deg.append(self.pitch_deg)
        self.saved.roll_deg.append(self.roll_deg)
        self.saved.yaw_deg.append(self.yaw_deg)
        self.saved.cf.append(self.cf)
        self.saved.o_quiet.append(self.o_quiet)
        self.saved.o_is_quiet.append(self.o_is_quiet)
        self.saved.o_is_quiet_sure.append(self.o_is_quiet_sure)
        self.saved.g_quiet.append(self.g_quiet)
        self.saved.g_is_quiet.append(self.g_is_quiet)
        self.saved.g_is_quiet_sure.append(self.g_is_quiet_sure)
        self.saved.G_QUIET_THR.append(self.G_QUIET_THR)
        self.saved.O_QUIET_THR.append(self.O_QUIET_THR)
        self.saved.eye_rate.append(self.eye_rate)
        self.saved.roll_rate.append(self.roll_rate)
        self.saved.pitch_rate.append(self.pitch_rate)
        self.saved.yaw_rate.append(self.yaw_rate)
        self.saved.eye_reset_LRL.append(self.eye_reset_LRL)
        self.saved.eye_reset_RLR.append(self.eye_reset_RLR)
        self.saved.yaw_eye_reset.append(self.yaw_eye_reset)

    def __str__(self):
        strg = ("{:9.3f}".format(self.time) + "{:9.3f}".format(self.eye_voltage_norm) +
                "{:9.3f}".format(self.eye_voltage_filt))
        return strg

class Saved:
    # For plot savings.   A better way is 'Saver' class in pyfilter helpers and requires making a __dict__
    def __init__(self):
        self.reset = []
        self.time = []
        self.head_reset = []
        self.eye_reset = []
        self.T = []
        self.eye_voltage_norm = []
        self.a_raw = []
        self.b_raw = []
        self.c_raw = []
        self.o_raw = []
        self.x_raw = []
        self.y_raw = []
        self.z_raw = []
        self.g_raw = []
        self.eye_voltage_filt = []
        self.eye_voltage_flt = []
        self.eye_closed = []
        self.eye_closed_confirmed = []
        self.eye_buzz = []
        self.flt_LTST = []
        self.cf = []
        self.dltst = []
        self.freeze = []
        self.lt_state = []
        self.st_state = []
        self.frz_thr_pos = []
        self.flt_thr_pos = []
        self.max_nod_f = []
        self.max_nod_f_confirmed = []
        self.max_nod_p = []
        self.max_nod_p_confirmed = []
        self.head_buzz_f = []
        self.head_buzz_p = []
        self.head_buzz = []
        self.roll_deg = []
        self.pitch_deg = []
        self.yaw_deg = []
        self.cf = []
        self.o_quiet = []
        self.o_is_quiet = []
        self.o_is_quiet_sure = []
        self.g_quiet = []
        self.g_is_quiet = []
        self.g_is_quiet_sure = []
        self.O_QUIET_THR = []
        self.G_QUIET_THR = []
        self.eye_rate = []
        self.roll_rate = []
        self.pitch_rate = []
        self.yaw_rate = []
        self.eye_reset_LRL = []
        self.eye_reset_RLR = []
        self.yaw_eye_reset = []
