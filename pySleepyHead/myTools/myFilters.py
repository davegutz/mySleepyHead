# Filter classes
# Copyright (C) 2023 Dave Gutz
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

__author__ = 'Dave Gutz <davegutz@alum.mit.edu>'
__version__ = '$Revision: 1.1 $'
__date__ = '$Date: 2022/06/10 13:15:02 $'

from gc import freeze

import numpy as np
import os


class RateLimit:
    def __init__(self):
        self.rate_min = 0.
        self.rate_max = 0.
        self.y = 0.
        self.y_past = 0.
        self.rate = 0.
        self.dt = 0.

    def update(self, x, reset, dt, min_, max_):
        self.rate_min = min_
        self.rate_max = max_
        self.dt = dt
        if reset:
            self.y = x
            self.y_past = x
        else:
            self.y = max(min(x, self.y_past + self.rate_max*self.dt), self.y_past + self.rate_min*self.dt)
        self.rate = (self.y - self.y_past) / self.dt
        self.y_past = self.y
        return self.y


class SlidingDeadband:
    def __init__(self, hdb):
        self.z = 0.
        self.hdb = hdb

    def update(self, in_):
        self.z = max(min(self.z, in_+self.hdb), in_-self.hdb)
        return self.z

    def update_res(self, in_, reset):
        if reset:
            self.z = in_
            return self.z
        out = self.update(in_)
        return out


# 1-pole filters
class DiscreteFilter:
    # Base class for 1-pole filters

    def __init__(self, dt, tau, min_, max_):
        self.dt = dt
        self.tau = tau
        self.min = min_
        self.max = max_
        self.rate = 0.

    def __str__(self, prefix=''):
        s = prefix + "DiscreteFilter:\n"
        s += "  dt       =    {:7.3f}  // Update time, s\n".format(self.dt)
        s += "  tau      =    {:7.3f}  // Natural frequency, r/s\n".format(self.tau)
        s += "  min      =    {:7.3f}  // Min output\n".format(self.min)
        s += "  max      =    {:7.3f}  // Max output\n".format(self.max)
        s += "  rate     =    {:7.3f}  // Calculated rate\n".format(self.rate)
        return s

    def calculate_(self, reset):
        if reset:
            self.rate = 0.
        return self.rate

    def calculate(self, in_, reset, dt):
        raise NotImplementedError

    def calculate_tau(self, in_, reset, dt, tau):
        raise NotImplementedError

    def assign_coeff(self, tau):
        raise NotImplementedError

    def rate_state(self, in_):
        raise NotImplementedError

    def rate_state_calc(self, in_, dt, reset):
        raise NotImplementedError

    def state(self):
        raise NotImplementedError


# 1-pole filters
class DiscreteFilter2:
    # Base class for 2-pole filters

    def __init__(self, dt, omega_n, zeta, min_, max_):
        self.dt = dt
        self.omega_n = omega_n
        self.zeta = zeta
        self.min = min_
        self.max = max_
        self.rate = 0.

    def __str__(self, prefix=''):
        s = prefix + "DiscreteFilter2:\n"
        s += "  dt       =    {:7.3f}  // Update time, s\n".format(self.dt)
        s += "  omega_n  =    {:7.3f}  // Natural frequency, r/s\n".format(self.omega_n)
        s += "  zeta     =    {:7.3f}  // Damping factor\n".format(self.zeta)
        s += "  min      =    {:7.3f}  // Min output\n".format(self.min)
        s += "  max      =    {:7.3f}  // Max output\n".format(self.max)
        s += "  rate     =    {:7.3f}  // Calculated rate\n".format(self.rate)
        return s

    def calculate_(self, in_, reset):
        if reset:
            self.rate = 0.
        return self.rate

    def calculate(self, in_, reset, dt):
        raise NotImplementedError

    def assign_coeff(self, tau):
        raise NotImplementedError

    def rate_state(self, in_, reset):
        raise NotImplementedError

    def rate_state_calc(self, in_, dt, reset):
        raise NotImplementedError

    def state(self):
        raise NotImplementedError


class DiscreteIntegrator:
    """Generic integrator"""

    def __init__(self, dt, min_, max_, a, b, c):
        self.dt = dt
        self.a = a
        self.b = b
        self.c = c
        self.lim = False
        self.max = max_
        self.min = min_
        self.state = 0.
        self.rate_state = 0.

    def __repr__(self):
        return ("{:8.6f}".format(self.dt) +
                # "{:2d}".format(self.reset) +
                "{:7.3f}".format(self.a) +
                "{:7.3f}".format(self.b) +
                "{:7.3f}".format(self.c) +
                "{:7.3g}".format(self.min) +
                "{:7.3g}".format(self.max) +
                "{:9.3g}".format(self.state) +
                "{:9.3g}".format(self.rate_state))

    def __str__(self, prefix=''):
        s = prefix + "DiscreteIntegrator:\n"
        s += "  dt       =    {:7.3f}  // Update time, s\n".format(self.dt)
        s += "  a        =    {:7.3f}  // Coefficient\n".format(self.a)
        s += "  b        =    {:7.3f}  // Coefficient\n".format(self.b)
        s += "  c        =    {:7.3f}  // Coefficient\n".format(self.c)
        s += "  min      =    {:7.3f}  // Min output\n".format(self.min)
        s += "  max      =    {:7.3f}  // Max output\n".format(self.max)
        s += "  state    =    {:7.3f}  // State\n".format(self.state)
        s += "  rate_state=   {:7.3f}  // Rate state\n".format(self.rate_state)
        return s

    def new_state(self, new_state):
        self.state = max(min(new_state, self.max), self.min)
        self.rate_state = 0.

    def calculate_(self, in_, reset, init_value):
        if reset:
            self.state = init_value
            self.rate_state = 0.
        else:
            self.state += (self.a*in_ + self.b*self.rate_state) * self.dt / self.c
        if self.state < self.min:
            self.state = self.min
            self.lim = True
            self.rate_state = 0.
        elif self.state > self.max:
            self.state = self.max
            self.lim = True
            self.rate_state = 0.
        else:
            self.lim = False
            self.rate_state = in_
        return self.state

    def calculate(self, in_, dt, reset, init_value):
        self.dt = dt
        if reset:
            self.state = init_value
            self.rate_state = 0.
        else:
            self.state += (self.a*in_ + self.b*self.rate_state) * self.dt / self.c
        if self.state < self.min:
            self.state = self.min
            self.lim = True
            self.rate_state = 0.
        elif self.state > self.max:
            self.state = self.max
            self.lim = True
            self.rate_state = 0.
        else:
            self.lim = False
            self.rate_state = in_
        return self.state


class AB2Integrator(DiscreteIntegrator):
    """AB2 Integrator"""

    def __init__(self, dt, min_, max_):
        DiscreteIntegrator.__init__(self, dt, min_, max_, 3., -1., 2.)

    def __str__(self, prefix=''):
        s = prefix + "AB2Integrator:\n"
        s += "\n  "
        s += DiscreteIntegrator.__str__(self, prefix + 'AB2Integrator:')
        return s


class InlineExpLag:
    """Inline exponential lag"""
    def __init__(self, tau):
        self.tau = tau
        self.rstate = 0.
        self.lstate = 0.
        self.inp = 0.
        self.rate = 0.

    def update(self, inp, T, reset=False):
        if reset:
            self.lstate = inp
            self.rstate = inp
        if T > 0:
            eTt = np.exp(-T/self.tau)
            meTt = 1. - eTt
            a = self.tau/T - eTt/meTt
            b = 1./meTt - self.tau/T
            c = meTt/T
        # noinspection PyUnboundLocalVariable
        self.rate = c * (a*self.rstate + b*inp - self.lstate)
        self.rstate = inp
        self.lstate = self.lstate + T*self.rate
        return self.lstate


class TustinIntegrator(DiscreteIntegrator):
    """Tustin Integrator"""

    def __init__(self, dt, min_, max_):
        DiscreteIntegrator.__init__(self, dt, min_, max_, 1., 1., 2.)

    def __str__(self, prefix=''):
        s = prefix + "TustinIntegrator:\n"
        s += "\n  "
        s += DiscreteIntegrator.__str__(self, prefix + 'TustinIntegrator:')
        return s

    def set(self, value=None):
        if value is not None:
            self.state = value
            self.rate_state = 0.


class General2Pole(DiscreteFilter2):
    # General 2-Pole filter variable update rate and limits, poor aliasing characteristics

    def __init__(self, dt, omega_n, zeta, minv_=-1.e12, maxv_=1.e12, min_=-1.e6, max_=1.e6):
        DiscreteFilter2.__init__(self, dt, omega_n, zeta, min_, max_)
        self.a = 2. * self.zeta * self.omega_n
        self.b = self.omega_n * self.omega_n
        self.assign_coeff(dt)
        self.AB2 = AB2Integrator(dt, minv_, maxv_)
        self.Tustin = TustinIntegrator(dt, min_, max_)
        self.in_ = 0.
        self.out_ = 0.
        self.accel = 0.
        self.dt = None
        self.reset = None
        self.saved = Saved2()

    def __repr__(self):
        return ("{:8.6f}".format(self.dt) +
                "{:2d}".format(self.reset) +
                "{:7.3f}".format(self.a) +
                "{:7.3f}".format(self.b) +
                "{:7.3f}".format(self.in_) +
                "{:7.3f}".format(self.accel) +
                "{:7.3f}".format(self.out_) )

    def __str__(self, prefix=''):
        s = prefix + "General2Pole:\n"
        s += DiscreteFilter2.__str__(self, prefix + 'General2Pole:')
        s += "\n  "
        s += self.AB2.__str__(prefix + 'General2Pole:')
        s += "\n  "
        s += self.Tustin.__str__(prefix + 'General2Pole:')
        s += "  a        =    {:7.3f}  // Discrete coefficient\n".format(self.a)
        s += "  b        =    {:7.3f}  // Discrete coefficient\n".format(self.b)
        s += "  accel    =    {:7.3f}  // Input\n".format(self.accel)
        s += "  in_      =    {:7.3f}  // Input\n".format(self.in_)
        s += "  out_     =    {:7.3f}  // Output\n".format(self.out_)
        return s

    def assign_coeff(self, dt):
        self.dt = dt

    def calculate_(self, in_, reset):
        self.in_ = in_
        self.reset = reset
        self.rate_state(self.in_, reset)

    def calculate(self, in_, reset, dt):
        self.in_ = in_
        self.reset = reset
        self.assign_coeff(dt)
        self.rate_state_calc(self.in_, dt, reset)
        self.out_ = self.Tustin.state
        return self.out_

    def rate_state(self, in_, reset):
        if reset:
            self.accel = 0.
        else:
            self.accel = self.b * (in_ - self.Tustin.state) - self.a * self.AB2.state
        self.Tustin.calculate(self.AB2.calculate(self.accel, self.dt, reset, 0), self.dt, reset, in_)
        if self.Tustin.lim:
            self.AB2.new_state(0)

    def rate_state_calc(self, in_, dt, reset):
        self.assign_coeff(dt)
        self.rate_state(in_, reset)

    def save(self, time):
        self.saved.time.append(time)
        self.saved.in_.append(self.in_)
        self.saved.out_.append(self.out_)
        self.saved.accel.append(self.accel)


class LagExp(DiscreteFilter):
    # Exponential lag calculator

    def __init__(self, dt, tau, min_, max_):
        DiscreteFilter.__init__(self, dt, tau, min_, max_)
        self.a = 0.
        self.b = 0.
        self.c = 0.
        self.rate = 0.
        self.rstate = 0.
        self.state = 0.
        self.assign_coeff(tau)
        self.saved = Saved1()
        self.in_ = 0.
        self.out_ = 0.

    def __str__(self, prefix=''):
        s = prefix + "LagExp:"
        s += "\n  "
        s += DiscreteFilter.__str__(self, prefix + 'LagExp:')
        s += "  a        =    {:7.3f}  // Discrete coefficient\n".format(self.a)
        s += "  b        =    {:7.3f}  // Discrete coefficient\n".format(self.b)
        s += "  c        =    {:7.3f}  // Discrete coefficient\n".format(self.c)
        s += "  tau      =    {:7.3f}  // Time constant, s\n".format(self.tau)
        s += "  in_      =    {:7.3f}  // Input\n".format(self.in_)
        s += "  out_     =    {:7.3f}  // Output\n".format(self.out_)
        return s

    def absorb(self, other):
        self.state = other.state
        self.rstate = other.rstate

    def assign_coeff(self, _tau):
        self.tau = _tau
        if self.dt > 0:
            eTt = np.exp(-self.dt / self.tau)
            meTt = 1. - eTt
            self.a = self.tau / self.dt - eTt / meTt
            self.b = 1.0 / meTt - self.tau / self.dt
            self.c = meTt / self.dt

    def calc_state_(self, in_):
        self.rate = self.c * (self.a * self.rstate + self.b * in_ - self.state)
        self.rstate = in_
        self.state = max(min(self.state + self.dt * self.rate, self.max), self.min)
        # print('in_', in_, 'rate', self.rate, 'state', self.state)

    def calc_state(self, in_, dt):
        self.dt = dt
        self.assign_coeff(self.tau)
        self.calc_state_(in_)

    def calculate(self, in_, reset, dt):
        self.in_ = in_
        if reset:
            self.state = self.in_
            self.rstate = self.in_
        self.calc_state(self.in_, dt)
        self.out_ = self.state
        return self.out_

    def calculate_tau(self, in_, reset, dt, tau_):
        self.in_ = in_
        self.tau = tau_
        if reset:
            self.state = self.in_
            self.rstate = self.in_
        self.calc_state(self.in_, dt)
        self.out_ = self.state
        return self.out_

    def save(self, time):
        self.saved.time.append(time)
        self.saved.rate.append(self.rate)
        self.saved.state.append(self.rstate)
        self.saved.state.append(self.state)
        self.saved.in_.append(self.in_)
        self.saved.out_.append(self.out_)


class RateLagExp(DiscreteFilter):
    # Exponential lag calculator

    def __init__(self, dt, tau, min_, max_):
        DiscreteFilter.__init__(self, dt, tau, min_, max_)
        self.a = 0.
        self.b = 0.
        self.c = 0.
        self.rate = 0.
        self.rstate = 0.
        self.state = 0.
        self.assign_coeff(tau)
        self.saved = Saved1()
        self.in_ = 0.
        self.out_ = 0.

    def __str__(self, prefix=''):
        s = prefix + "LagExp:"
        s += "\n  "
        s += DiscreteFilter.__str__(self, prefix + 'LagExp:')
        s += "  a        =    {:7.3f}  // Discrete coefficient\n".format(self.a)
        s += "  b        =    {:7.3f}  // Discrete coefficient\n".format(self.b)
        s += "  c        =    {:7.3f}  // Discrete coefficient\n".format(self.c)
        s += "  tau      =    {:7.3f}  // Time constant, s\n".format(self.tau)
        s += "  in_      =    {:7.3f}  // Input\n".format(self.in_)
        s += "  out_     =    {:7.3f}  // Output\n".format(self.out_)
        return s

    def absorb(self, other):
        self.state = other.state
        self.rstate = other.rstate

    def assign_coeff(self, _tau):
        self.tau = _tau
        if self.dt > 0:
            eTt = np.exp(-self.dt / self.tau)
            meTt = 1. - eTt
            self.a = self.tau / self.dt - eTt / meTt
            self.b = 1.0 / meTt - self.tau / self.dt
            self.c = meTt / self.dt

    def calc_state_(self, in_):
        self.rate = self.c * (self.a * self.rstate + self.b * in_ - self.state)
        self.rstate = in_
        self.state = max(min(self.state + self.dt * self.rate, self.max), self.min)
        # print('in_', in_, 'rate', self.rate, 'state', self.state)

    def calc_state(self, in_, dt):
        self.dt = dt
        self.assign_coeff(self.tau)
        self.calc_state_(in_)

    def calculate(self, in_, reset, dt):
        self.in_ = in_
        if reset:
            self.state = self.in_
            self.rstate = self.in_
        self.rate_state_(self.in_, dt)
        self.out_ = self.rate
        return self.out_

    def calculate_tau(self, in_, reset, dt, tau_):
        self.in_ = in_
        self.tau = tau_
        if reset:
            self.state = self.in_
            self.rstate = self.in_
        self.calc_state(self.in_, dt)
        self.out_ = self.state
        return self.out_

    def rate_state_(self, in_, dt):
        self.dt = dt
        self.rate = self.c * (self.a * self.rstate + self.b * in_ - self.state)
        self.rstate = in_
        self.state = max(min(self.state + self.dt * self.rate, self.max), self.min)

    def __repr__(self):
        print("RateLagExp: tau_ = {:7.3f}".format(self.tau),
        " T =  {:7.3f}".format(self.dt),
        " min =  {:7.3f}".format(self.min),
        " max =  {:7.3f}".format(self.max),
        " a =  {:7.3f}".format(self.a),
        " b =  {:7.3f}".format(self.b),
        " c =  {:7.3f}".format(self.c),
        " in =  {:7.3f}".format(self.in_),
        " lstate =  {:7.3f}".format(self.state),
        " rstate =  {:7.3f}".format(self.rstate),
        " rate =  {:7.3f}".format(self.rate),
              )

    def save(self, time):
        self.saved.time.append(time)
        self.saved.rate.append(self.rate)
        self.saved.state.append(self.rstate)
        self.saved.state.append(self.state)
        self.saved.in_.append(self.in_)
        self.saved.out_.append(self.out_)


class LagTustin(DiscreteFilter):
    # Tustin lag calculator, non-pre-warped

    def __init__(self, dt, tau, min_, max_):
        DiscreteFilter.__init__(self, dt, tau, min_, max_)
        self.a = 0.
        self.b = 0.
        self.rate = 0.
        self.state = 0.
        self.assign_coeff(tau)
        self.saved = Saved1()
        self.in_ = 0.
        self.out_ = 0.

    def __str__(self, prefix=''):
        s = prefix + "LagTustin:"
        s += "\n  "
        s += DiscreteFilter.__str__(self, prefix + 'LagTustin:')
        s += "  a        =    {:7.3f}  // Discrete coefficient\n".format(self.a)
        s += "  b        =    {:7.3f}  // Discrete coefficient\n".format(self.b)
        s += "  in_      =    {:7.3f}  // Input\n".format(self.in_)
        s += "  out_     =    {:7.3f}  // Output\n".format(self.out_)
        return s

    def assign_coeff(self, tau):
        self.tau = tau
        self.a = 2.0 / (2.0 * self.tau + self.dt)
        self.b = (2.0 * self.tau - self.dt) / (2.0 * self.tau + self.dt)

    def calc_state_(self, in_):
        self.rate = max(min(self.a * (in_ - self.state), self.max), self.min)
        self.state = max(min(in_ * (1.0 - self.b) + self.state * self.b, self.max), self.min)

    def calc_state(self, in_, dt):
        self.dt = dt
        self.assign_coeff(self.tau)
        self.calc_state_(in_)

    def calculate(self, in_, reset, dt):
        self.in_ = in_
        if reset:
            self.state = self.in_
        self.calc_state(self.in_, dt)
        self.out_ = self.state
        return self.out_

    def save(self, time):
        self.saved.time.append(time)
        self.saved.rate.append(self.rate)
        self.saved.state.append(self.state)
        self.saved.in_.append(self.in_)
        self.saved.out_.append(self.out_)


class LongTermShortTermFilter:
    """Dynamic change detection of a normally unchanging signal, see US Patent """
    # Tustin lag calculator, non-pre-warped

    def __init__(self, dt, tau_lt, tau_st, flt_thr_neg=-1.e6, frz_thr_neg=-1e5, flt_thr_pos=1e6, frz_thr_pos=1e5):
        self.tau_lt = tau_lt
        self.tau_st = tau_st
        self.dt = dt
        self.frz_thr_neg = frz_thr_neg
        self.flt_thr_neg = flt_thr_neg
        self.frz_thr_pos = frz_thr_pos
        self.flt_thr_pos = flt_thr_pos
        self.klt = self.dt / self.tau_lt
        self.kst = self.dt / self.tau_st
        self.cf = 1.
        self.dltst = None
        self.fault = False
        self.freeze = False
        self.reset = True
        self.input = None
        self.lt_state = None
        self.st_state = None

    def __repr__(self, prefix=''):
        s = prefix + "LTST filter:"
        s += "  reset {:d}".format(self.reset)
        s += "  dt {:9.6f}".format(self.dt)
        s += "  input {:7.3f}".format(self.input)
        s += "  lt_state {:7.3f}".format(self.lt_state)
        s += "  st_state {:7.3f}".format(self.st_state)
        s += "  dltst {:7.3f}".format(self.dltst)
        s += "  freeze {:d}".format(self.freeze)
        s += "  cf {:7.3f}".format(self.cf)
        s += "  fault {:d}".format(self.fault)
        return s

    def __str__(self, prefix=''):
        s = prefix + "LTST filter:"
        s += "\n  "
        s += "  klt      =    {:7.3f}  // Long term gain, r/s\n".format(self.klt)
        s += "  kst      =    {:7.3f}  // Short term gain, r/s\n".format(self.kst)
        s += "  cf       =    {:7.3f}  // Confidence that filter unfaulted\n".format(self.cf)
        s += "  dltst    =    {:7.3f}  // Filter difference, units of input\n".format(self.dltst)
        s += "  dt       =    {:9.6f}  // Update time, s\n".format(self.dt)
        s += "  fault    =    {:d}  // Detected a fault\n".format(self.fault)
        s += "  freeze   =    {:d}  // Long term frozen; suspect a fault\n".format(self.freeze)
        s += "  flt_thr_neg=  {:7.3f}  // Negative drift high threshold declaring fault\n".format(self.flt_thr_neg)
        s += "  flt_thr_pos=  {:7.3f}  // Positive drift fault threshold declaring fault\n".format(self.flt_thr_pos)
        s += "  frz_thr_neg=  {:7.3f}  // Negative drift fault threshold declaring freeze, units of input\n".format(self.frz_thr_neg)
        s += "  frz_thr_pos=  {:7.3f}  // Positive drift fault threshold declaring freeze, units of input\n".format(self.frz_thr_pos)
        s += "  input    =    {:7.3f}  // Filter input, units of input\n".format(self.input)
        s += "  lt_state =    {:7.3f}  // Filter long term state, units of input\n".format(self.lt_state)
        s += "  reset    =    {:d}     // Initializing \n".format(self.reset)
        s += "  st_state =    {:7.3f}  // Filter short term state, units of input\n".format(self.st_state)
        s += "  tau_lt   =    {:7.3f}  // Filter long term time constant, s\n".format(self.tau_lt)
        s += "  tau_st   =    {:7.3f}  // Filter short term time constant, s\n".format(self.tau_st)
        return s

    def assign_coeff(self, dt):
        self.dt = dt
        self.klt = self.dt / self.tau_lt
        self.kst = self.dt / self.tau_st

    def calculate(self, in_, reset, dt, lt_bias_init=0):
        self.reset = reset
        self.dt = dt
        self.input = in_
        self.assign_coeff(self.dt)
        if self.reset:
            self.lt_state = self.input + lt_bias_init
            self.st_state = self.input
            self.dltst = 0.
            self.cf = 1.
            self.freeze = False
            self.fault = False
            return self.fault
        if not self.freeze:
            self.lt_state = self.input * self.klt + self.lt_state * (1. - self.klt)
        self.st_state = self.input * self.kst + self.st_state * (1. - self.kst)
        self.dltst = self.lt_state - self.st_state
        if self.dltst <= 0:
            self.freeze = self.dltst <= self.frz_thr_neg
            self.fault = self.dltst <= self.flt_thr_neg
            if self.freeze is np.True_:
                self.cf = max(min( 1. - (self.frz_thr_neg - self.dltst) / (self.frz_thr_neg - self.flt_thr_neg), 1.), 0.)
            else:
                self.cf = 1.
        else:
            self.freeze = self.dltst >= self.frz_thr_pos
            self.fault = self.dltst >= self.flt_thr_pos
            if self.freeze is np.True_:
                self.cf = max(min( 1. - (self.dltst - self.frz_thr_pos) / (self.flt_thr_pos - self.frz_thr_pos), 1.), 0.)
            else:
                self.cf = 1.
        return self.fault


class Saved1:
    # For plot 1st order filter savings.
    # A better way is 'Saver' class in pyfilter helpers and requires making a __dict__
    def __init__(self):
        self.time = []
        self.rstate = []
        self.state = []
        self.rate = []
        self.in_ = []
        self.out_ = []


class Saved2:
    # For plot 1st order filter savings.
    # A better way is 'Saver' class in pyfilter helpers and requires making a __dict__
    def __init__(self):
        self.time = []
        self.in_ = []
        self.out_ = []
        self.accel = []
        self.AB2_state = []
        self.AB2_rate_state = []
        self.Tustin_state = []
        self.Tustin_rate_state = []
        
        
if __name__ == '__main__':
    import sys
    import doctest
    from datetime import datetime

    doctest.testmod(sys.modules['__main__'])
    import matplotlib.pyplot as plt


    def overall(filter_1=Saved1(), filter_2=Saved2(), filename='', fig_files=None, plot_title=None, fig_list=None):
        if fig_files is None:
            fig_files = []

        fig_list.append(plt.figure())
        plt.subplot(211)
        plt.title(plot_title)
        plt.plot(filter_1.time, filter_1.in_, color='blue', label='in 1')
        plt.plot(filter_1.time, filter_1.out_, color='green', label='out 1')
        plt.legend(loc=3)
        plt.subplot(212)
        plt.plot(filter_2.time, filter_2.in_, color='blue', label='in 2')
        plt.plot(filter_2.time, filter_2.out_, color='green', label='out 2')
        plt.legend(loc=3)
        fig_file_name = filename + "_" + str(len(fig_list)) + ".png"
        fig_files.append(fig_file_name)
        plt.savefig(fig_file_name, format="png")

        return fig_list, fig_files


    class Pulsar:
        def __init__(self):
            self.time_last_hold = 0.
            self.time_last_rest = -100000.
            self.holding = False
            self.resting = True
            self.index = -1
            self.amp = [1.,     0.,     -1.,    1.]
            self.dur = [20.,    0.,     40.,    40.]
            self.rst = [20.,    20.,    40.,    40.]
            self.pulse_value = self.amp[0]
            self.end_time = self.time_end()

        def calculate(self, time):
            if self.resting and time >= self.time_last_rest + self.rst[self.index]:
                if time < self.end_time:
                    self.index += 1
                self.resting = False
                self.holding = True
                self.time_last_hold = time
                self.pulse_value = self.amp[self.index]
            elif self.holding and time >= self.time_last_hold + self.dur[self.index]:
                self.index += 0  # only advance after resting
                self.resting = True
                self.holding = False
                self.time_last_rest = time
                self.pulse_value = 0.
            return self.pulse_value

        def time_end(self):
            time = 0
            for du in self.dur:
                time += du
            for rs in self.rst:
                time += rs
            return time


    def main():
        # Setup to run the transients
        dt = 2
        pull = Pulsar()
        time_end = pull.time_end()
        # time_end = 50.

        filter_1 = LagTustin(dt, 5., -10., 10.)
        # print(filter_1)
        # filter_2 = General2Pole(dt, 1., .707, -10., 10.)
        filter_2 = General2Pole(dt, 0.1, 0.9, -10., 10.)
        # print(filter_2)

        # Executive tasks
        t = np.arange(0, time_end + dt, dt)

        # time loop
        for i in range(len(t)):
            in_ = pull.calculate(t[i])
            reset = (t[i] <= 1)

            # Models
            filter_1.calculate(in_, reset, dt)
            filter_2.calculate(in_, reset, dt)
            # print("t,in,accel,rate_state,state,out=", t[i],filter_2.in_,
            #   filter_2.accel, filter_2.AB2.state, filter_2.Tustin.state, filter_2.out_)

            # Plot stuff
            filter_1.save(t[i])
            filter_2.save(t[i])

        # Data
        # print(filter_1)
        # print(filter_2)

        # Plots
        fig_list = []
        fig_files = []
        date_time = datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
        filename = os.path.split(__file__)[1].split('.')[0]
        plot_title = filename + '   ' + date_time

        overall(filter_1.saved, filter_2.saved, filename, fig_files, plot_title=plot_title, fig_list=fig_list)
        plt.show()


    if __name__ == '__main__':
        main()
