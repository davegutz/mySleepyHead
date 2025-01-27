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
    plq(plt, mo, 'time', mo, 'max_nod_f', color='cyan', linestyle='-', label='max_nod_f' + ref_str)
    plq(plt, mo, 'time', mo, 'max_nod_p', color='magenta', linestyle='--', label='max_nod_p' + ref_str)
    plt.legend(loc=1)
    plt.subplot(232)
    plq(plt, mo, 'time', mo, 'eye_voltage_norm', color='black', linestyle='-', label='eye_voltage_norm' + ref_str)
    plq(plt, mv, 'time', mv, 'eye_voltage_norm', color='orange', linestyle='-', label='eye_voltage_norm' + test_str)
    plq(plt, mv, 'time', mv, 'eye_voltage_filt', color='red', linestyle='-.', label='eye_voltage_filt' + test_str)
    plq(plt, mo, 'time', mo, 'lt_state', color='black', linestyle='-', label='lt_state' + ref_str)
    plq(plt, mv, 'time', mv, 'lt_state', color='green', linestyle='--', label='lt_state' + test_str)
    plq(plt, mo, 'time', mo, 'st_state', color='blue', linestyle='-', label='st_state' + ref_str)
    plq(plt, mv, 'time', mv, 'st_state', color='red', linestyle='--', label='st_state' + test_str)
    plq(plt, mo, 'time', mo, 'dltst', color='blue', linestyle='-', label='dltst' + ref_str)
    plq(plt, mv, 'time', mv, 'dltst', color='red', linestyle='--', label='dltst' + test_str)
    plq(plt, mo, 'time', mo, 'FRZ_THR_POS', color='yellow', linestyle='--', label='FRZ_THR_POS' + ref_str)
    plq(plt, mv, 'time', mv, 'frz_thr_pos', color='blue', linestyle='-.', label='frz_thr_pos' + test_str)
    plq(plt, mo, 'time', mo, 'FLT_THR_POS', color='orange', linestyle='--', label='FLT_THR_POS' + ref_str)
    plq(plt, mv, 'time', mv, 'flt_thr_pos', color='black', linestyle='-.', label='flt_thr_pos' + test_str)
    plq(plt, mo, 'time', mo, 'eye_buzz', slr=0.25, add=0.75, color='red', linestyle='-', label='eye_buzz' + ref_str + '*.75+.25')
    plq(plt, mv, 'time', mv, 'eye_buzz', slr=0.25, add=0.75, color='blue', linestyle='--', label='eye_buzz' + test_str + '*.75+.25')
    plt.legend(loc=1)
    ax = plt.subplot(133)
    plq(plt, mo, 'time', mo, 'eye_buzz', add=14, color='red', linestyle='-', label='eye_buzz+' + ref_str + '+14')
    plq(plt, mv, 'time', mv, 'eye_buzz', add=14, color='blue', linestyle='--', label='eye_buzz' + test_str + '14')
    plq(plt, mo, 'time', mo, 'eye_closed_confirmed', add=12, color='red', linestyle='-', label='eye_closed_confirmed' + ref_str + '+12')
    plq(plt, mv, 'time', mv, 'eye_closed_confirmed', add=12, color='blue', linestyle='--', label='eye_closed_confirmed+' + test_str + '+12')
    plq(plt, mo, 'time', mo, 'eye_closed', add=10, color='red', linestyle='-', label='eye_closed' + ref_str + '10')
    plq(plt, mv, 'time', mv, 'eye_closed', add=10, color='blue', linestyle='--', label='eye_closed' + test_str + '10')
    plq(plt, mo, 'time', mo, 'head_buzz', add=8, color='red', linestyle='-', label='head_buzz' + ref_str + '+8')
    plq(plt, mv, 'time', mv, 'head_buzz', add=8, color='blue', linestyle='--', label='head_buzz' + test_str + '+8')
    plq(plt, mo, 'time', mo, 'freeze', add=6, color='red', linestyle='-', label='freeze' + ref_str + '+6')
    plq(plt, mv, 'time', mv, 'freeze', add=6, color='blue', linestyle='--', label='freeze' + test_str + '+6')
    plq(plt, mv, 'time', mv, 'cf', add=4, color='blue', linestyle='--', label='cf' + test_str + '+4')
    plq(plt, mo, 'time', mo, 'head_reset', add=2, color='red', linestyle='-', label='head_reset' + ref_str + '+2')
    plq(plt, mv, 'time', mv, 'head_reset', add=2, color='blue', linestyle='--', label='head_reset' + test_str + '+2')
    plq(plt, mo, 'time', mo, 'eye_reset', add=0, color='red', linestyle='-', label='eye_reset' + ref_str + '+0')
    plq(plt, mv, 'time', mv, 'eye_reset', add=0, color='blue', linestyle='--', label='eye_reset' + test_str + '+0')
    plq(plt, mo, 'time', mo, 'reset', add=-2, color='red', linestyle='-', label='reset' + ref_str + '-2')
    plq(plt, mv, 'time', mv, 'reset', add=-2, color='blue', linestyle='--', label='reset' + test_str + '-2')
    plt.ylim([-4, 16])
    ax.set_yticks(np.arange(-4, 16, 2))
    plt.legend(loc=1)
    plt.subplot(234)
    plq(plt, mv, 'time', mv, 'o_is_quiet', add=8, color='blue', linestyle='-', label='o_is_quiet' + test_str + '+8')
    plq(plt, mv, 'time', mv, 'g_is_quiet', add=8, color='magenta', linestyle='--', label='g_is_quiet' + test_str + '+8')
    plq(plt, mv, 'time', mv, 'o_is_quiet_sure', add=6, color='blue', linestyle='-', label='o_is_quiet_sure' + test_str + '+6')
    plq(plt, mv, 'time', mv, 'g_is_quiet_sure', add=6, color='magenta', linestyle='--', label='g_is_quiet_sure' + test_str + '+6')
    plq(plt, mo, 'time', mo, 'head_reset', add=4, color='red', linestyle='-', label='head_reset' + ref_str + '+4')
    plq(plt, mv, 'time', mv, 'head_reset', add=4, color='blue', linestyle='--', label='head_reset' + test_str + '+4')
    plq(plt, mo, 'time', mo, 'g_quiet', add=2, color='red', linestyle='-', label='g_quiet' + ref_str + '+2')
    plq(plt, mv, 'time', mv, 'g_quiet', add= 2, color='blue', linestyle='--', label='g_quiet' + test_str + '+2')
    plq(plt, mv, 'time', mv, 'G_QUIET_THR', add= 2, color='red', linestyle=':', label='G_QUIET_THR' + test_str + '+2')
    plq(plt, mv, 'time', mv, 'G_QUIET_THR', slr=-1, add= 2, color='red', linestyle=':', label='G_QUIET_THR' + test_str + '+2')
    plq(plt, mo, 'time', mo, 'o_quiet', add=-4, color='red', linestyle='-', label='o_quiet' + ref_str + '-4')
    plq(plt, mv, 'time', mv, 'o_quiet', add=-4, color='magenta', linestyle='--', label='o_quiet' + test_str + '-4')
    plq(plt, mv, 'time', mv, 'O_QUIET_THR', add=-4, color='magenta', linestyle=':', label='O_QUIET_THR' + test_str + '-4')
    plq(plt, mv, 'time', mv, 'O_QUIET_THR', slr=-1, add=-4, color='magenta', linestyle=':', label='O_QUIET_THR' + test_str + '-4')
    plt.legend(loc=1)
    plt.subplot(235)
    plq(plt, mv, 'time', mv, 'x_raw', color='pink', linestyle='--', label='x_raw' + test_str)
    plq(plt, mv, 'time', mv, 'y_raw', color='cyan', linestyle='-.', label='y_raw' + test_str)
    plq(plt, mv, 'time', mv, 'z_raw', color='red', linestyle='-', label='z_raw' + test_str)
    plq(plt, mv, 'time', mv, 'g_raw', color='orange', linestyle='--', label='g_raw' + test_str)
    plq(plt, mv, 'time', mv, 'a_raw', add= 2, color='cyan', linestyle='-', label='a_raw' + test_str + '+2')
    plq(plt, mv, 'time', mv, 'b_raw', add= 2, color='magenta', linestyle='--', label='b_raw' + test_str + '+2')
    plq(plt, mv, 'time', mv, 'c_raw', add= 2, color='blue', linestyle='-.', label='c_raw' + test_str + '+2')
    plq(plt, mv, 'time', mv, 'o_raw', add= 2, color='black', linestyle='-', label='o_raw' + test_str + '+2')
    plt.legend(loc=1)
    fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    fig_files.append(fig_file_name)
    plt.savefig(fig_file_name, format="png")

    return fig_list, fig_files
