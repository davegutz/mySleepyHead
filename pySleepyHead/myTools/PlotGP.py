# PlotGP - general purpose plotting
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

"""Define a general purpose battery model including Randles' model and SoC-VOV model as well as Kalman filtering
Filter Observer for SOC Estimation of Commercial Power-Oriented LFP Lithium Battery Cells.
Dependencies:
    - numpy      (everything)
    - matplotlib (plots)
    - reportlab  (figures, pdf)
"""
import numpy as np
import matplotlib.pyplot as plt
from myFilters import InlineExpLag
from DataOverModel import plq
# below suppresses runtime error display******************
# import os
# os.environ["KIVY_NO_CONSOLELOG"] = "1"
# from kivy.utils import platform  # failed experiment to run BLE data plotting realtime on android
# if platform != 'linux':
#     from unite_pictures import unite_pictures_into_pdf, cleanup_fig_files
import sys
if sys.platform == 'darwin':
    import matplotlib
    matplotlib.use('tkagg')

plt.rcParams.update({'figure.max_open_warning': 0})


def gp_plot(mo, mv, filename, fig_files=None, plot_title=None, fig_list=None, ref_str='_ref', test_str='_test'):
    fig_list.append(plt.figure())  # GP 1
    plt.subplot(231)
    plt.title(plot_title + ' GP 1')
    # plt.plot(mo.time, mo.eye_voltage_norm, 1, color='black', linestyle='-', label='eye_voltage_norm' + ref_str, marker=None, markersize=None, markevery=None)
    plq(plt, mo, 'time', mo, 'eye_voltage_norm', color='black', linestyle='-', label='eye_voltage_norm' + ref_str)
    plq(plt, mv, 'time', mv, 'eye_voltage_filt', color='red', linestyle='-.', label='eye_voltage_filt' + test_str)
    plq(plt, mo, 'time', mo, 'eye_closed_confirmed', color='green', linestyle='-.', label='eye_closed_confirmed' + ref_str)
    plq(plt, mv, 'time', mv, 'eye_closed_confirmed', color='magenta', linestyle=':', label='eye_closed_confirmed' + test_str)
    # plq(plt, mo, 'time', mo, 'eye_voltage_flt', color='red', linestyle='--', label='eye_voltage_flt' + ref_str)
    plt.legend(loc=1)
    plt.subplot(232)
    plq(plt, mo, 'time', mo, 'eye_closed', add=2, color='blue', linestyle='-', label='eye_closed' + ref_str + '2')
    plq(plt, mv, 'time', mv, 'eye_closed', add=2, color='cyan', linestyle='--', label='eye_closed' + test_str + '2')
    plq(plt, mo, 'time', mo, 'eye_closed_confirmed', color='green', linestyle='-.', label='eye_closed_confirmed' + ref_str)
    plq(plt, mv, 'time', mv, 'eye_closed_confirmed', color='magenta', linestyle=':', label='eye_closed_confirmed' + test_str)
    plq(plt, mv, 'time', mv, 'eye_closed_LTST', color='orange', linestyle='--', label='eye_closed_LTST' + test_str)
    plt.legend(loc=1)
    plt.subplot(233)
    plq(plt, mo, 'time', mo, 'max_nod_f', color='cyan', linestyle='-', label='max_nod_f' + ref_str)
    plq(plt, mo, 'time', mo, 'max_nod_p', color='magenta', linestyle='--', label='max_nod_p' + ref_str)
    plt.legend(loc=1)
    plt.subplot(234)
    plq(plt, mo, 'time', mo, 'head_buzz', add=10, color='black', linestyle='-', label='head_buzz' + ref_str + '+10')
    plq(plt, mo, 'time', mo, 'eye_buzz', add=8, color='orange', linestyle='--', label='eye_buzz' + ref_str + '+8')
    plq(plt, mo, 'time', mo, 'freeze', add=6, color='orange', linestyle='--', label='freeze' + ref_str + '+6')
    plt.legend(loc=1)
    plt.subplot(235)
    plq(plt, mv, 'time', mv, 'eye_voltage_norm', color='orange', linestyle='-', label='eye_voltage_norm' + test_str)
    plq(plt, mo, 'time', mo, 'lt_state', color='black', linestyle='-', label='lt_state' + ref_str)
    plq(plt, mv, 'time', mv, 'lt_state', color='green', linestyle='--', label='lt_state' + test_str)
    plq(plt, mo, 'time', mo, 'st_state', color='blue', linestyle='-', label='st_state' + ref_str)
    plq(plt, mv, 'time', mv, 'st_state', color='red', linestyle='--', label='st_state' + test_str)
    plq(plt, mo, 'time', mo, 'dltst', color='blue', linestyle='-', label='dltst' + ref_str)
    plq(plt, mv, 'time', mv, 'dltst', color='red', linestyle='--', label='dltst' + test_str)
    plq(plt, mv, 'time', mv, 'frz_thr_pos', color='blue', linestyle='--', label='frz_thr_pos' + test_str)
    plq(plt, mv, 'time', mv, 'flt_thr_pos', color='black', linestyle='--', label='ffl_thr_pos' + test_str)
    plq(plt, mo, 'time', mo, 'eye_buzz', slr=0.25, add=0.75, color='black', linestyle='-', label='eye_buzz' + ref_str)
    plq(plt, mv, 'time', mv, 'eye_closed_LTST', slr=0.25, add=0.75, color='magenta', linestyle='--', label='eye_closed_LTST' + test_str)
    plt.legend(loc=1)
    plt.subplot(236)
    plq(plt, mv, 'time', mv, 'cf', add=4, color='blue', linestyle='-', label='cf + 4' + test_str)
    plq(plt, mv, 'time', mv, 'flt_LTST', add=2, color='green', linestyle='-', label='flt_LTST + 2' + test_str)
    plq(plt, mv, 'time', mv, 'freeze', color='red', linestyle='-', label='freeze' + test_str)
    plt.legend(loc=1)
    fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    fig_files.append(fig_file_name)
    plt.savefig(fig_file_name, format="png")

    # fig_list.append(plt.figure())  # GP 2
    # plt.subplot(221)
    # plt.title(plot_title + ' GP 2')
    # plt.plot(mo.time, mo.vb, color='black', linestyle='-', label='vb' + ref_str)
    # plt.plot(mv.time, mv.vb, color='orange', linestyle='--', label='vb' + test_str)
    # plt.plot(mo.time, mo.voc, color='blue', linestyle='-.', label='voc' + ref_str)
    # plt.plot(mv.time, mv.voc, color='red', linestyle=':', label='voc' + test_str)
    # plt.plot(mo.time, mo.voc_stat, color='cyan', linestyle='-.', label='voc_stat' + ref_str)
    # plt.plot(mv.time, mv.voc_stat, color='black', linestyle=':', label='voc_stat' + test_str)
    # plt.legend(loc=1)
    # plt.subplot(222)
    # plt.plot(mo.time, mo.dv_hys, linestyle='-', color='black', label='dv_hys' + ref_str)
    # plt.plot(mv.time, mv.dv_hys, linestyle='--', color='orange', label='dv_hys' + test_str)
    # plt.legend(loc=1)
    # plt.subplot(223)
    # plt.plot(mo.time, mo.soc, linestyle='-', color='black', label='soc' + ref_str)
    # plt.plot(mv.time, mv.soc, linestyle='--', color='orange', label='soc' + test_str)
    # plt.legend(loc=1)
    # plt.subplot(224)
    # plq(plt, mo, 'time', mo, 'ib_sel', linestyle='-', color='black', label='ib_sel' + ref_str)
    # plq(plt, so, 'time', so, 'ib_in_s', linestyle='--', color='cyan', label='ib_in_s' + ref_str)
    # plt.plot(mv.time, mv.ib_charge, linestyle='-.', color='orange', label='ib_charge' + test_str)
    # plt.plot(mo.time, mo.ib_diff, linestyle=':', color='red', label='ib_diff' + ref_str)
    # plt.legend(loc=1)
    # fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    # fig_files.append(fig_file_name)
    # plt.savefig(fig_file_name, format="png")
    return fig_list, fig_files
