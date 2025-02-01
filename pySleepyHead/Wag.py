# Detect sway motion
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
import numpy as np
from matplotlib.bezier import NonIntersectingPathException
from numpy import arange

from TFDelay import TFDelay


class Wag:
    """Go once with in_by_2 then with in_by_1 then complete with in_by_2"""
    def __init__(self, init=False, nom_dt=0.1, set_all=0.1, hold_1=3., hold_2=2., hold_3=5.):
        self.reset = None
        self.T = nom_dt
        self.in_by_2 = init  # First input
        self.in_by_1 = init  # Second input
        self.output = init
        self.time_set = set_all
        self.time_reset_1 = hold_1
        self.time_reset_2 = hold_2
        self.time_reset_3 = hold_3
        self.timer_1 = TFDelay(self.in_by_2, self.time_set, self.time_reset_1, self.T)
        self.timer_2 = TFDelay(self.in_by_2, self.time_set, self.time_reset_2, self.T)
        self.timer_3 = TFDelay(self.in_by_2, self.time_set, self.time_reset_3, self.T)
        self.state_1 = init
        self.state_2 = init

    def __repr__(self):
        s = "Wag:"
        s += "  reset =  {:2d}".format(self.reset)
        s += "  T   =  {:5.3f}".format(self.T)
        s += "  time_set = {:7.3f}".format(self.time_set)
        s += "  time_reset_1 = {:7.3f} time_reset_2 = {:7.3f}".format(self.time_reset_1, self.time_reset_2, self.time_reset_3)
        s += "  inputx2 = {:7.3f} inputx1 = {:7.3f}".format(self.in_by_2, self.in_by_1)
        s += "  state_1 = {:7.3f} state_2 = {:7.3f} state_3= {:7.3f}".format(self.state_1, self.state_2, self.output)
        return s

    def calculate(self, reset, dt, in_1, in_2):
            self.reset = reset
            self.T = dt
            self.in_by_2 = in_1
            self.in_by_1 = in_2
            self.state_1 = self.timer_1.calculate(in_=self.in_by_2, t_true=self.time_set, t_false=self.time_reset_1,
                                                  reset=self.reset, dt=self.T)
            self.state_2 = self.timer_2.calculate(in_=(self.state_1 and self.in_by_1), t_true=self.time_set,
                                                  t_false=self.time_reset_2, reset=self.reset, dt=self.T)
            self.output = self.timer_3.calculate(in_=(self.state_2 and self.in_by_2), t_true=self.time_set,
                                                 t_false=self.time_reset_3, reset=self.reset, dt=self.T)
            return self.output


def main(dt=0.1, set_all=0.2, hold_1=3., hold_2=2., hold_3=5.):
    my_wag = Wag(init=False, nom_dt=dt, set_all=set_all, hold_1=hold_1, hold_2=hold_2, hold_3=hold_3)
    freq_hz = 0.6
    freq_rps = freq_hz * 2. * np.pi  # r/s
    period = 1. / freq_hz
    low_thresh = -25.
    high_thresh = 25.
    reset = True
    low = False
    high = False
    for i in arange(0, 100):
        t = dt * i
        signal = -100.*np.sin(t * freq_rps)
        if t > period:
            signal = 0.
        low = signal <= low_thresh
        high = signal >= high_thresh
        triggered = my_wag.calculate(reset=reset, dt=dt, in_1=low, in_2=high)
        print("{:5.1f}".format(t), "reset:", reset, " signal:{:5.1f}".format(signal),
              " low{:2d}:".format(low),
              " high{:2d}:".format(high),
              " inx2{:2d}:".format(my_wag.in_by_2),
              " inx1{:2d}:".format(my_wag.in_by_1),
              " state_1{:2d}:".format(my_wag.state_1),
              " state_2{:2d}:".format(my_wag.state_2),
              " output{:2d}:".format(my_wag.output))
        reset = False


# import cProfile
# if __name__ == '__main__':
#     cProfile.run('main()')
#

if __name__ == '__main__':
    main()
