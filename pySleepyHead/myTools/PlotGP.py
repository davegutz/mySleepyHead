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


def gp_plot(mo, mv, so, sv, smv, filename, fig_files=None, plot_title=None, fig_list=None,
            ref_str='_ref', test_str='_test'):
    fig_list.append(plt.figure())  # GP 1
    plt.subplot(221)
    plt.title(plot_title + ' GP 1')
    # plt.plot(mo.time, mo.eye_voltage, 1, color='black', linestyle='-', label='eye_voltage' + ref_str, marker=None, markersize=None, markevery=None)
    plq(plt, mo, 'time', mo, 'eye_voltage', color='black', linestyle='-', label='eye_voltage' + ref_str)
    # plq(plt, mo, 'time', mo, 'eye_voltage_thr', color='red', linestyle='--', label='eye_voltage_thr' + ref_str)
    plt.legend(loc=1)
    plt.subplot(222)
    plq(plt, mo, 'time', mo, 'eye_cl', color='blue', linestyle='-', label='eye_cl' + ref_str)
    plq(plt, mo, 'time', mo, 'conf', color='green', linestyle='--', label='conf' + ref_str)
    plt.legend(loc=1)
    plt.subplot(223)
    plq(plt, mo, 'time', mo, 'max_nod_f', color='cyan', linestyle='-', label='max_nod_f' + ref_str)
    plq(plt, mo, 'time', mo, 'max_nod_p', color='magenta', linestyle='--', label='max_nod_p' + ref_str)
    plt.legend(loc=1)
    plt.subplot(224)
    plq(plt, mo, 'time', mo, 'buzz', color='red', linestyle='-', label='buzz' + ref_str)
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
