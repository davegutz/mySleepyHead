# PlotMahony_Mahony - general purpose plotting
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

deg_2_rps = np.pi / 180.
def gp_plot(mo, mv, filename, fig_files=None, plot_title=None, fig_list=None, ref_str='_ref', test_str='_test'):
    fig_list.append(plt.figure())  # Mahony 1
    plt.subplot(111)
    plt.title(plot_title + ' Mahony 1')
    plq(plt, mo, 'time', mo, 'reset', add=8, color='red', linestyle='-', label='head_reset' + ref_str + '+8')
    plq(plt, mo, 'time', mo, 'a_raw', slr=deg_2_rps, add= 6, color='red', linestyle='-', label='a_raw' + ref_str + '+6')
    plq(plt, mv, 'time', mv, 'a_raw', slr=deg_2_rps, add= 6, color='blue', linestyle='--', label='a_raw' + test_str + '+6')
    plq(plt, mo, 'time', mo, 'b_raw', slr=deg_2_rps, add= 4, color='red', linestyle='-', label='b_raw' + ref_str + '+4')
    plq(plt, mv, 'time', mv, 'b_raw', slr=deg_2_rps, add= 4, color='blue', linestyle='--', label='b_raw' + test_str + '+4')
    plq(plt, mo, 'time', mo, 'c_raw', slr=deg_2_rps, add= 2, color='red', linestyle='-', label='c_raw' + ref_str + '+2')
    plq(plt, mv, 'time', mv, 'c_raw', slr=deg_2_rps, add= 2, color='blue', linestyle='--', label='c_raw' + test_str + '+2')
    plq(plt, mo, 'time', mo, 'x_raw', add=0, color='red', linestyle='-', label='x_raw' + ref_str + '+0')
    plq(plt, mv, 'time', mv, 'x_raw', add=0, color='blue', linestyle='--', label='x_raw' + test_str + '+0')
    plq(plt, mo, 'time', mo, 'y_raw', add=-2, color='red', linestyle='-', label='y_raw' + ref_str + '-2')
    plq(plt, mv, 'time', mv, 'y_raw', add=-2, color='blue', linestyle='--', label='y_raw' + test_str + '-2')
    plq(plt, mo, 'time', mo, 'z_raw', add=-4, color='red', linestyle='-', label='z_raw' + ref_str + '-4')
    plq(plt, mv, 'time', mv, 'z_raw', add=-4, color='blue', linestyle='--', label='z_raw' + test_str + '-4')
    plt.legend(loc=1)
    fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    fig_files.append(fig_file_name)
    plt.savefig(fig_file_name, format="png")

    fig_list.append(plt.figure())  # Mahony 2
    plt.subplot(111)
    plt.title(plot_title + ' Mahony 2')
    plq(plt, mo, 'time', mo, 'halfex', add= 6, color='red', linestyle='-', label='halfex' + ref_str + '+6')
    plq(plt, mv, 'time', mv, 'halfex', add= 6, color='blue', linestyle='--', label='halfex' + test_str + '+6')
    plq(plt, mo, 'time', mo, 'halfey', add= 4, color='red', linestyle='-', label='halfey' + ref_str + '+4')
    plq(plt, mv, 'time', mv, 'halfey', add= 4, color='blue', linestyle='--', label='halfey' + test_str + '+4')
    plq(plt, mo, 'time', mo, 'halfez', add= 2, color='red', linestyle='-', label='halfez' + ref_str + '+2')
    plq(plt, mv, 'time', mv, 'halfez', add= 2, color='blue', linestyle='--', label='halfez' + test_str + '+2')
    plq(plt, mo, 'time', mo, 'halfvx', add=0, color='red', linestyle='-', label='halfvx' + ref_str + '+0')
    plq(plt, mv, 'time', mv, 'halfvx', add=0, color='blue', linestyle='--', label='halfvx' + test_str + '+0')
    plq(plt, mo, 'time', mo, 'halfvy', add=-2, color='red', linestyle='-', label='halfvy' + ref_str + '-2')
    plq(plt, mv, 'time', mv, 'halfvy', add=-2, color='blue', linestyle='--', label='halfvy' + test_str + '-2')
    plq(plt, mo, 'time', mo, 'halfvz', add=-4, color='red', linestyle='-', label='halfvz' + ref_str + '-4')
    plq(plt, mv, 'time', mv, 'halfvz', add=-4, color='blue', linestyle='--', label='halfvz' + test_str + '-4')
    plt.legend(loc=1)
    fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    fig_files.append(fig_file_name)
    plt.savefig(fig_file_name, format="png")

    fig_list.append(plt.figure())  # Mahony 3
    plt.subplot(111)
    plt.title(plot_title + ' Mahony 3')
    plq(plt, mo, 'time', mo, 'ifb_x', add= 6, color='red', linestyle='-', label='ifb_x' + ref_str + '+6')
    plq(plt, mv, 'time', mv, 'ifb_x', add= 6, color='blue', linestyle='--', label='ifb_x' + test_str + '+6')
    plq(plt, mo, 'time', mo, 'ifb_y', add= 4, color='red', linestyle='-', label='ifb_y' + ref_str + '+4')
    plq(plt, mv, 'time', mv, 'ifb_y', add= 4, color='blue', linestyle='--', label='ifb_y' + test_str + '+4')
    plq(plt, mo, 'time', mo, 'ifb_z', add= 2, color='red', linestyle='-', label='ifb_z' + ref_str + '+2')
    plq(plt, mv, 'time', mv, 'ifb_z', add= 2, color='blue', linestyle='--', label='ifb_z' + test_str + '+2')
    plt.legend(loc=1)
    fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    fig_files.append(fig_file_name)
    plt.savefig(fig_file_name, format="png")

    fig_list.append(plt.figure())  # Mahony 4
    plt.subplot(111)
    plt.title(plot_title + ' Mahony 4')
    plq(plt, mo, 'time', mo, 'q0', add= 6, color='red', linestyle='-', label='q0' + ref_str + '+6')
    plq(plt, mv, 'time', mv, 'q0', add= 6, color='blue', linestyle='--', label='q0' + test_str + '+6')
    plq(plt, mo, 'time', mo, 'q1', add= 4, color='red', linestyle='-', label='q1' + ref_str + '+4')
    plq(plt, mv, 'time', mv, 'q1', add= 4, color='blue', linestyle='--', label='q1' + test_str + '+4')
    plq(plt, mo, 'time', mo, 'q2', add= 2, color='red', linestyle='-', label='q2' + ref_str + '+2')
    plq(plt, mv, 'time', mv, 'q2', add= 2, color='blue', linestyle='--', label='q2' + test_str + '+2')
    plq(plt, mo, 'time', mo, 'q3', add= 0, color='red', linestyle='-', label='q3' + ref_str + '+0')
    plq(plt, mv, 'time', mv, 'q3', add= 0, color='blue', linestyle='--', label='q3' + test_str + '+0')
    plt.legend(loc=1)
    fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    fig_files.append(fig_file_name)
    plt.savefig(fig_file_name, format="png")

    fig_list.append(plt.figure())  # Mahony 5
    plt.subplot(131)
    plt.title(plot_title + ' Mahony 5')
    plq(plt, mo, 'time', mo, 'roll_deg', color='red', linestyle='-', label='roll_deg' + ref_str)
    plq(plt, mv, 'time', mv, 'roll_deg', color='blue', linestyle='--', label='roll_deg' + test_str)
    plt.legend(loc=1)
    plt.subplot(132)
    plq(plt, mo, 'time', mo, 'pitch_deg', color='black', linestyle='-', label='pitch_deg' + ref_str)
    plq(plt, mv, 'time', mv, 'pitch_deg', color='green', linestyle='--', label='pitch_deg' + test_str)
    plt.legend(loc=1)
    plt.subplot(133)
    plq(plt, mo, 'time', mo, 'yaw_deg', color='orange', linestyle='-', label='yaw_deg' + ref_str)
    plq(plt, mv, 'time', mv, 'yaw_deg', color='cyan', linestyle='--', label='yaw_deg' + test_str)
    plt.legend(loc=1)
    fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    fig_files.append(fig_file_name)
    plt.savefig(fig_file_name, format="png")

    return fig_list, fig_files
