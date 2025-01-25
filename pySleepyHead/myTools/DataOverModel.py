# GP_batteryEKF - general purpose battery class for EKF use
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

"""Define a general purpose battery model
Filter Observer for SOC Estimation of Commercial Power-Oriented LFP Lithium Battery Cells.
Dependencies:
    - numpy      (everything)
    - matplotlib (plots)
    - reportlab  (figures, pdf)
"""

import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from unite_pictures import unite_pictures_into_pdf, cleanup_fig_files
from Colors import Colors
import re
from local_paths import version_from_data_file, local_paths
import os
import sys
if sys.platform == 'darwin':
    import matplotlib
    matplotlib.use('tkagg')
plt.rcParams.update({'figure.max_open_warning': 0})


def plq(plt_, sx, st, sy, yt, slr=1., add=0., color='black', linestyle='-', label=None, marker=None,
        markersize=None, markevery=None):
    if (sx is not None and sy is not None and hasattr(sx, st) and hasattr(sy, yt) and
            len(getattr(sy, yt)) > 0 and getattr(sy, yt)[0] is not None):
        try:
            yscld = getattr(sy, yt) * slr + add
        except TypeError:
            yscld = np.array(getattr(sy, yt)) * slr + add
        try:
            plt_.plot(getattr(sx, st), yscld, color=color, linestyle=linestyle, label=label, marker=marker,
                      markersize=markersize, markevery=markevery)
        except ValueError:
            pass


def write_clean_file(path_to_data, type_=None, hdr_key=None, unit_key=None, skip=1, comment_str='#'):
    """First line with hdr_key defines the number of fields to be imported cleanly"""
    import os
    (path, basename) = os.path.split(path_to_data)
    version = version_from_data_file(path_to_data)
    (path_to_temp, save_pdf_path, dum) = local_paths(version)
    csv_file = path_to_temp+'/'+basename.replace('.csv', type_ + '.csv', 1)
    # Header
    have_header_str = None
    num_fields = 0
    with open(path_to_data, "r", encoding='cp437') as input_file:
        with open(csv_file, "w") as output:
            try:
                for line in input_file:
                    if line.__contains__('FRAG'):
                        print(Colors.fg.red, "\n\n\nDataOverModel(write_clean_file): Heap fragmentation error\
                         detected in Particle.  Decrease NSUM constant and re-run\n\n", Colors.reset)
                        return None
                    if line.__contains__(hdr_key):
                        if have_header_str is None:
                            have_header_str = True  # write one title only
                            output.write(line)
                            num_fields = line.count(',')  # first line with hdr_key defines number of fields
            except IOError:
                print("DataOverModel381:", line)  # last line
    # Data
    num_lines = 0
    num_lines_in = 0
    num_skips = 1
    length = 0
    unit_key_found = False
    with (open(path_to_data, "r", encoding='cp437') as input_file):  # reads all characters even bad ones
        with open(csv_file, "a") as output:
            for line in input_file:
                if line.__contains__(unit_key) and not line.__contains__('Config:'):
                    unit_key_found = True
                    if line.count(",") == num_fields and line.count(";") == 0 and \
                            re.search(r'[^a-zA-Z0-9+-_.:, ]', line[:-1]) is None and \
                            (num_lines == 0 or ((num_lines_in+1) % skip) == 0) and line.count(comment_str) == 0:
                        output.write(line)
                        num_lines += 1
                    else:
                        print('discarding: ', line)
                        num_skips += 1
                    num_lines_in += 1
    if not num_lines:
        csv_file = None
        print("I(write_clean_file): no data to write")
        if not unit_key_found:
            print("W(write_clean_file):  unit_key not found in ", basename, ".  Looking with '{:s}'".format(unit_key))
    else:
        print("Wrote(write_clean_file):", csv_file, num_lines, "lines", num_skips, "skips", length, "fields")
    return csv_file


class SavedData:
    def __init__(self, data=None, sel=None, ekf=None, time_end=None, zero_zero=False, zero_thr=0.02, sync_cTime=None):
        i_end = 0
        if data is None:
            self.i = 0
            self.cTime = None
            self.time = None
            self.eye_voltage_norm = None
            self.unit = None  # text title
            self.eye_voltage_flt = None
            self.eye_closed = None
            self.eye_closed_confirmed = None
            self.max_nod_f = None
            self.max_nod_p = None
            self.head_buzz = None
            self.eye_buzz = None
            self.lt_state = None
            self.st_state = None
            self.dltst = None
            self.freeze = None
        else:
            self.i = 0
            self.cTime = np.array(data.cTime)
            self.time = np.array(data.cTime)
            self.eye_voltage_norm = np.array(data.eye_voltage_norm)
            # manage data shape
            # Find first non-zero ib and use to adjust time
            # Ignore initial run of non-zero ib because resetting from previous run
            if zero_zero:
                self.zero_end = 0
            elif sync_cTime is not None:
                self.zero_end = np.where(self.cTime < sync_cTime[0])[0][-1] + 2
            else:
                try:
                    self.zero_end = 0
                    # stop after first non-zero
                    while self.zero_end < len(self.eye_voltage_norm) and abs(self.eye_voltage_norm[self.zero_end]) < zero_thr:
                        self.zero_end += 1
                    self.zero_end -= 1  # backup one
                    if self.zero_end == len(self.eye_voltage_norm) - 1:
                        print(Colors.fg.red, f"\n\nLikely ib is zero throughout the data.  Check setup and retry\n\n",
                              Colors.reset)
                        self.zero_end = 0
                    elif self.zero_end == -1:
                        # print(Colors.fg.red, f"\n\nLikely ib is noisy throughout the data.  Check setup and retry\n\n",
                        #       Colors.reset)
                        self.zero_end = 0
                except IOError:
                    self.zero_end = 0
            self.time_ref = self.time[self.zero_end]
            self.time -= self.time_ref
            self.time_min = self.time / 60.
            self.time_day = self.time / 3600. / 24.

            # Truncate
            if time_end is None:
                i_end = len(self.time)
                if sel is not None:
                    self.c_time_s = np.array(sel.c_time) - self.time_ref
                    i_end = min(i_end, len(self.c_time_s))
                if ekf is not None:
                    self.c_time_e = np.array(ekf.c_time) - self.time_ref
                    i_end = min(i_end, len(self.c_time_e))
            else:
                i_end = np.where(self.time <= time_end)[0][-1] + 1
                if sel is not None:
                    self.c_time_s = np.array(sel.c_time) - self.time_ref
                    i_end_sel = np.where(self.c_time_s <= time_end)[0][-1] + 1
                    i_end = min(i_end, i_end_sel)
                    self.zero_end = min(self.zero_end, i_end-1)
                if ekf is not None:
                    self.c_time_e = np.array(ekf.c_time) - self.time_ref
                    i_end_ekf = np.where(self.c_time_e <= time_end)[0][-1] + 1
                    i_end = min(i_end, i_end_ekf)
                    self.zero_end = min(self.zero_end, i_end - 1)
            self.cTime = self.cTime[:i_end]
            self.time = np.array(self.time[:i_end])
            self.eye_voltage_norm = np.array(data.eye_voltage_norm[:i_end])
            self.eye_voltage_flt = np.array(data.eye_voltage_flt[:i_end])
            self.eye_closed = np.array(data.eye_closed[:i_end])
            self.eye_closed_confirmed = np.array(data.eye_closed_confirmed[:i_end])
            self.max_nod_f = np.array(data.max_nod_f[:i_end])
            self.max_nod_p = np.array(data.max_nod_p[:i_end])
            self.head_buzz = np.array(data.head_buzz[:i_end])
            self.eye_buzz = np.array(data.eye_buzz[:i_end])
            self.lt_state = np.array(data.lt_state[:i_end])
            self.st_state = np.array(data.st_state[:i_end])
            self.dltst = np.array(data.dltst[:i_end])
            self.freeze = np.array(data.freeze[:i_end])

        if sel is None:
            self.c_time_s = None
        else:
            self.c_time_s = None
        if ekf is None:
            self.c_time_e = None
        else:
            self.c_time_e = None

    def __str__(self):
        s = "{},".format(self.unit[self.i])
        s += "{:13.3f},".format(self.cTime[self.i])
        s += "{:13.6f},".format(self.time[self.i])
        s += "{:8.3f},".format(self.eye_voltage_norm[self.i])
        s += "{:7.2f},".format(self.eye_voltage_flt[self.i])
        s += "{:5.2f},".format(self.eye_closed[self.i])
        s += "{:5.2f},".format(self.eye_closed_confirmed[self.i])
        s += "{:5.2f},".format(self.max_nod_f[self.i])
        s += "{:10.6f},".format(self.max_nod_p[self.i])
        s += "{:7.3f},".format(self.head_buzz[self.i])
        return s

    def mod(self):
        return self.mod_data[self.zero_end]

    def __str__(self):
        s = "{},".format(self.unit[self.i])
        s += "{:13.3f},".format(self.cTime[self.i])
        return s


if __name__ == '__main__':
    import sys
    import doctest

    doctest.testmod(sys.modules['__main__'])
    if sys.platform == 'darwin':
        import matplotlib
        matplotlib.use('tkagg')
    plt.rcParams['axes.grid'] = True

    def compare_print(mo, mv):
        s = " time,      ib,                   vb,              dv_dyn,          voc_stat,\
                    voc,        voc_ekf,         y_ekf,               soc_ekf,      soc,\n"
        for i in range(len(mv.time)):
            s += "{:7.3f},".format(mo.time[i])
            s += "{:11.3f},".format(mo.ib[i])
            s += "{:9.3f},".format(mv.ib[i])
            s += "{:9.2f},".format(mo.vb[i])
            s += "{:5.2f},".format(mv.vb[i])
            s += "{:9.2f},".format(mo.dv_dyn[i])
            s += "{:5.2f},".format(mv.dv_dyn[i])
            s += "{:9.2f},".format(mo.voc_stat[i])
            s += "{:5.2f},".format(mv.voc_stat[i])
            s += "{:9.2f},".format(mo.voc[i])
            s += "{:5.2f},".format(mv.voc[i])
            s += "{:9.2f},".format(mo.voc_ekf[i])
            s += "{:5.2f},".format(mv.voc_ekf[i])
            s += "{:13.6f},".format(mo.y_ekf[i])
            s += "{:9.6f},".format(mv.y_ekf[i])
            s += "{:7.3f},".format(mo.soc_ekf[i])
            s += "{:5.3f},".format(mv.soc_ekf[i])
            s += "{:7.3f},".format(mo.soc[i])
            s += "{:5.3f},".format(mv.soc[i])
            s += "\n"
        return s


    def main(data_file_old_txt, unit_key):
        # Trade study inputs
        # i-->0 provides continuous anchor to reset filter (why?)  i shifts important --> 2 current sensors,
        #   hyst in ekf
        # saturation provides periodic anchor to reset filter
        # reset soc periodically anchor user display
        # tau_sd creating an anchor.   So large it's just a pass through
        # TODO:  temp sensitivities and mitigation

        # Config inputs
        # from MonSimNomConfig import *

        # Transient  inputs
        time_end = None
        zero_zero_in = False
        # time_end = 1500.

        # Load data (must end in .txt) txt_file, type, hdr_key, unit_key
        data_file_clean = write_clean_file(data_file_old_txt, type_='_mon', hdr_key='unit,',
                                           unit_key=unit_key)
        data_file_sim_clean = write_clean_file(data_file_old_txt, type_='_sim', hdr_key='unit_m',
                                               unit_key='unit_sim,')

        # Load
        mon_old_raw = np.genfromtxt(data_file_clean, delimiter=',', names=True, dtype=float).view(np.recarray)
        mon_old = SavedData(mon_old_raw, time_end, zero_zero=zero_zero_in)
        try:
            sim_old_raw = np.genfromtxt(data_file_sim_clean, delimiter=',', names=True, dtype=float).view(np.recarray)
            sim_old = SavedDataSim(mon_old.time_ref, sim_old_raw, time_end)
        except IOError:
            sim_old = None

        # Run model
        date_ = datetime.now().strftime("%y%m%d")
        mon_file_save = data_file_clean.replace(".csv", "_rep.csv")
        if data_file_sim_clean:
            sim_file_save = data_file_sim_clean.replace(".csv", "_rep.csv")

        # Plots
        fig_list = []
        fig_files = []
        date_time = datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
        filename = data_file_clean.split('/')[-1].replace('.csv', '-') + os.path.split(__file__)[1].split('.')[0]
        plot_title = filename + '   ' + date_time
        fig_list, fig_files = dom_plot(mon_old, None, sim_old, None, None, filename, fig_files,
                                       plot_title=plot_title, fig_list=fig_list, ref_str='', test_str='_ver')
        unite_pictures_into_pdf(outputPdfName=filename+'_'+date_time+'.pdf',
                                save_pdf_path='../dataReduction/figures')
        cleanup_fig_files(fig_files)

        plt.show()


    # python DataOverModel.py("../dataReduction/rapidTweakRegressionTest20220711.txt", "pro_2022")

    """
    PyCharm Sample Run Configuration Parameters (right click in pyCharm - Modify Run Configuration:
        "../dataReduction/slowTweakRegressionTest20220711.txt" "pro_2022"
        "../dataReduction/serial_20220624_095543.txt"    "pro_2022"
        "../dataReduction/real world rapid 20220713.txt" "soc0_2022"
        "../dataReduction/real world Xp20 20220715.txt" "soc0_2022"
    
    PyCharm Terminal:
    python DataOverModel.py "../dataReduction/serial_20220624_095543.txt" "pro_2022"
    python DataOverModel.py "../dataReduction/ampHiFail20220731.txt" "pro_2022"
    

    android:
    python Python/DataOverModel.py "USBTerminal/serial_20220624_095543.txt" "pro_2022"
    """

    if __name__ == "__main__":
        import sys
        print(sys.argv[1:])
        main(sys.argv[1], sys.argv[2])
