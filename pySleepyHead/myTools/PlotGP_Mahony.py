# PlotGP_Mahony - general purpose plotting
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
    plt.subplot(111)
    plt.title(plot_title + ' Mahony 1')
    plq(plt, mo, 'time', mo, 'head_reset', add=8, color='red', linestyle='-', label='head_reset' + ref_str + '+8')
    plq(plt, mv, 'time', mv, 'x_raw', add=6, color='pink', linestyle='--', label='x_raw' + test_str + '+6')
    plq(plt, mv, 'time', mv, 'y_raw', add=4, color='cyan', linestyle='-.', label='y_raw' + test_str + '+4')
    plq(plt, mv, 'time', mv, 'z_raw', add=2, color='red', linestyle='-', label='z_raw' + test_str + '+2')
    plq(plt, mv, 'time', mv, 'a_raw', add= 0, color='cyan', linestyle='-', label='a_raw' + test_str + '+0')
    plq(plt, mv, 'time', mv, 'b_raw', add= -2, color='magenta', linestyle='--', label='b_raw' + test_str + '-2')
    plq(plt, mv, 'time', mv, 'c_raw', add= -4, color='blue', linestyle='-.', label='c_raw' + test_str + '-4')
    plt.legend(loc=1)
    fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    fig_files.append(fig_file_name)
    plt.savefig(fig_file_name, format="png")

    fig_list.append(plt.figure())  # GP 2
    plt.subplot(131)
    plt.title(plot_title + ' Mahony 2')
    plq(plt, mo, 'time', mo, 'roll_filt', color='red', linestyle='-', label='roll_filt' + ref_str)
    plq(plt, mv, 'time', mv, 'roll_deg', color='blue', linestyle='--', label='roll_deg' + test_str)
    plt.legend(loc=1)
    plt.subplot(132)
    plq(plt, mo, 'time', mo, 'pitch_filt', color='black', linestyle='-', label='pitch_filt' + ref_str)
    plq(plt, mv, 'time', mv, 'pitch_deg', color='green', linestyle='--', label='pitch_deg' + test_str)
    plt.legend(loc=1)
    plt.subplot(133)
    plq(plt, mo, 'time', mo, 'yaw_filt', color='orange', linestyle='-', label='yaw_filt' + ref_str)
    plq(plt, mv, 'time', mv, 'yaw_deg', color='cyan', linestyle='--', label='yaw_deg' + test_str)
    plt.legend(loc=1)
    fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    fig_files.append(fig_file_name)
    plt.savefig(fig_file_name, format="png")

    return fig_list, fig_files
