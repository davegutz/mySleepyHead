# Delay class debounce logic
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
__date__ = '$Date: 2022/06/02 13:15:02 $'

import numpy as np
import os


class TFDelay:
    # Use variable resistor to create tfdteresis from an RC circuit

    def __init__(self, in_=False, t_true=0., t_false=0., dt=0.1):
        # Defaults
        self.timer = 0
        self.t_true = t_true
        self.t_false = t_false
        self.reset = None
        self.nt = int(max(round(self.t_true/dt)+1, 0))
        self.nf = int(max(round(self.t_false/dt)+1, 0))
        self.dt = dt
        self.in_ = in_
        self.out = self.in_
        if self.t_true == 0.0:
            self.nt = 0
        if self.t_false == 0.0:
            self.nf = 0
        if in_:
            self.timer = self.nf
        else:
            self.time = -self.nt
        self.saved = Saved()

    def __repr__(self):
        return ("{:8.6f}".format(self.dt) +
                "{:2d}".format(self.reset) +
                "{:2d}".format(self.nt) +
                "{:2d}".format(self.nf) +
                "{:2d}".format(self.timer) +
                "{:2d}".format(self.in_) +
                "{:2d}".format(self.out))

    def __str__(self, prefix=''):
        s = prefix + "TFDelay:\n"
        s += "  dt  =  {:5.1f}  // s\n".format(self.dt)
        s += "  t_t  = {:5.1f}  // s\n".format(self.t_true)
        s += "  t_f  = {:5.1f}  // s\n".format(self.t_false)
        s += "  nt   = {:2d}  // count\n".format(self.nt)
        s += "  nf   = {:2d}  // count\n".format(self.nf)
        s += "  timer= {:2d}  // count\n".format(self.timer)
        s += "  in   = {:2d}  // count\n".format(self.in_)
        s += "  out  = {:2d}  // count\n".format(self.out)
        return s

    def calculate1(self, in_):
        if self.timer >= 0:
            if in_:
                self.timer = self.nf
            else:
                self.timer -= 1
                if self.timer < 0:
                    self.timer = -self.nt
        else:
            if not in_:
                self.timer = -self.nt
            else:
                self.timer += 1
                if self.timer >= 0:
                    self.timer = self.nf
        self.out = self.timer > 0
        return self.out

    def calculate2(self, in_, reset):
        self.reset = reset
        if self.reset:
            if in_:
                self.timer = self.nf
            else:
                self.timer = -self.nt
            out = in_
        else:
            out = self.calculate1(in_)
        return out

    def calculate3(self, in_, t_true, t_false):
        self.nt = int(max(round(t_true / self.dt)+1, 0))
        self.nf = int(max(round(t_false / self.dt)+1, 0))
        return self.calculate1(in_)

    def calculate4t(self, in_, t_true, t_false, dt):
        self.dt = dt
        self.t_true = t_true
        self.t_false = t_false
        self.nt = int(max(round(self.t_true / self.dt)+1, 0))
        self.nf = int(max(round(self.t_false / self.dt)+1, 0))
        return self.calculate1(in_)

    def calculate4r(self, in_, t_true, t_false, reset):
        self.reset = reset
        if self.reset > 0:
            if in_:
                self.timer = self.nf
            else:
                self.timer = -self.nt
        return self.calculate3(in_, t_true, t_false)

    def calculate(self, in_, t_true, t_false, dt, reset):
        self.reset = reset
        self.in_ = in_
        if self.reset:
            if self.in_:
                self.timer = self.nf
            else:
                self.timer = -self.nt
        self.out = self.calculate4t(self.in_, t_true, t_false, dt)
        return self.out

    def save(self, time):
        self.saved.time.append(time)
        self.saved.reset.append(self.reset)
        self.saved.timer.append(self.timer)
        self.saved.in_.append(self.in_)
        self.saved.nt.append(self.nt)
        self.saved.nf.append(self.nf)
        self.saved.dt.append(self.dt)
        self.saved.out.append(self.out)

    def state(self):
        return self.timer > 0.


class Saved:
    # For plot savings.   A better way is 'Saver' class in pyfilter helpers and requires making a __dict__
    def __init__(self):
        self.time = []
        self.reset = []
        self.in_ = []
        self.out = []
        self.timer = []
        self.nf = []
        self.nt = []
        self.dt = []


if __name__ == '__main__':
    import sys
    import doctest
    from datetime import datetime

    doctest.testmod(sys.modules['__main__'])
    import matplotlib.pyplot as plt


    def overall(tfd=TFDelay().saved, filename='', fig_files=None, plot_title=None, fig_list=None):
        if fig_files is None:
            fig_files = []

        fig_list.append(plt.figure())
        plt.subplot(211)
        plt.title(plot_title)
        plt.plot(tfd.time, tfd.timer, color='red', linestyle='-', label='timer')
        plt.plot(tfd.time, tfd.nf, color='blue', linestyle='--', label='nf')
        plt.plot(tfd.time, tfd.nt, color='green', linestyle='-.', label='nt')
        plt.legend(loc=3)
        plt.subplot(212)
        plt.plot(tfd.time, tfd.in_, color='red', linestyle='-', label='in')
        plt.plot(tfd.time, tfd.out, color='blue', linestyle='--', label='out')
        plt.legend(loc=3)
        fig_file_name = filename + "_" + str(len(fig_list)) + ".png"
        fig_files.append(fig_file_name)
        plt.savefig(fig_file_name, format="png")

        return fig_list, fig_files


    def main():
        # Setup to run the transients
        dt = 10.
        ttg = 800.
        tfg = 100.
        tt = 8.
        tf = 1.
        time_end = 1500.

        tfd_long = TFDelay(in_=False, t_true=ttg, t_false=tfg, dt=dt)
        tfd_short = TFDelay(in_=False, t_true=tt, t_false=tf, dt=dt)

        # Executive tasks
        t = np.arange(0, time_end + dt, dt)

        # time loop
        for i in range(len(t)):
            init_tfd = (t[i] < 100)
            if 100. <= t[i] < 1000.:
                inp = True
            else:
                inp = False

            # Models
            tfd_long.calculate(inp, ttg, tfg, dt, init_tfd)
            tfd_short.calculate(inp, tt, tf, dt, init_tfd)

            # Plot stuff
            tfd_long.save(t[i])
            tfd_short.save(t[i])

        # Data
            if t[i] == 900.:
                print('long:  ', str(tfd_long))
                print('short:  ', str(tfd_short))

        # Plots
        fig_list = []
        fig_files = []
        date_time = datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
        filename = os.path.split(__file__)[1].split('.')[0]
        plot_title = filename + '   ' + date_time

        fig_list, fig_files = overall(tfd_long.saved, filename, fig_files, plot_title='long '+plot_title, fig_list=fig_list)
        overall(tfd_short.saved, filename, fig_files, plot_title='short '+plot_title, fig_list=fig_list)

        # unite_pictures_into_pdf(outputPdfName=filename+'_'+date_time+'.pdf', save_pdf_path='figures')
        # cleanup_fig_files(fig_files)
        plt.show()


    if __name__ == '__main__':
        main()
