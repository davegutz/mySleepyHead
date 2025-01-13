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


def dom_plot(mo, mv, so, sv, smv, filename, fig_files=None, plot_title=None, fig_list=None, plot_init_in=False,
             ref_str='_ref', test_str='_test'):
    if fig_files is None:
        fig_files = []

    if plot_init_in and hasattr(smv, 'time') and hasattr(so, 'time'):
        fig_list.append(plt.figure())  # init 1
        plt.subplot(221)
        plt.title(plot_title + ' init 1')
        plt.plot(so.time, so.reset_s, color='black', linestyle='-', label='reset_s'+ref_str)
        plq(plt, smv, 'time', smv, 'reset_s', color='red', linestyle='--', label='reset_s'+test_str)
        plt.plot(mo.time, mo.reset, color='magenta', linestyle='-', label='reset'+ref_str)
        plt.plot(mv.time, mv.reset, color='cyan', linestyle='--', label='reset'+test_str)
        plt.legend(loc=1)
        plt.subplot(222)
        plt.plot(so.time, so.Tb_s, color='black', linestyle='-', label='Tb_s'+ref_str)

        plt.plot(sv.time, sv.Tb, color='red', linestyle='--', label='Tb_s'+test_str)
        plt.plot(mo.time, mo.Tb, color='blue', linestyle='-.', label='Tb'+ref_str)
        plt.plot(mv.time, mv.Tb, color='green', linestyle=':', label='Tb'+test_str)
        plt.legend(loc=1)
        plt.subplot(223)
        plt.plot(so.time, so.soc_s, color='black', linestyle='-', label='soc_s'+ref_str)
        plt.plot(sv.time, sv.soc, color='red', linestyle='--', label='soc_s'+test_str)
        plt.plot(mo.time, mo.soc, color='blue', linestyle='-.', label='soc'+ref_str)
        plt.plot(mv.time, mv.soc, color='green', linestyle=':', label='soc'+test_str)
        plt.plot(mo.time, mo.soc_ekf, marker='^', markersize='5', markevery=32, linestyle='None', color='orange',
                 label='soc_ekf'+ref_str)
        plt.plot(mv.time, mv.soc_ekf, marker='+', markersize='5', markevery=32, linestyle='None', color='cyan',
                 label='soc_ekf'+test_str)
        plt.legend(loc=1)
        fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
        fig_files.append(fig_file_name)
        plt.savefig(fig_file_name, format="png")

        return fig_list, fig_files

    if mo.ibmh is not None and hasattr(mo, 'ib_diff_f'):
        fig_list.append(plt.figure())  # 1a
        plt.subplot(221)
        plt.title(plot_title + ' 1a')
        if mo.mod_data[0] == 0:
            plq(plt, mo, 'time', mo, 'idmap', color='black', linestyle='-', label='ib_amp_hdwe'+ref_str)
            plq(plt, mv, 'time', mv, 'ibmh', color='red', linestyle='-.', label='ib_amp_hdwe'+test_str)
            plq(plt, mo, 'time', mo, 'ibnh', color='green', linestyle='--', label='ib_noa_hdwe'+ref_str)
            plq(plt, mv, 'time', mv, 'ibnh', color='blue', linestyle=':', label='ib_noa_hdwe'+test_str)
        else:
            plq(plt, mo, 'time', mo, 'ibmm', color='black', linestyle='-', label='ib_amp_mod'+ref_str)
            plq(plt, mv, 'time', mv, 'ibmm', color='red', linestyle='-.', label='ib_amp_mod'+test_str)
            plq(plt, mo, 'time', mo, 'ibnm', color='green', linestyle='--', label='ib_noa_mod'+ref_str)
            plq(plt, mv, 'time', mv, 'ibnm', color='blue', linestyle=':', label='ib_noa_mod'+test_str)
        plq(plt, mo, 'time', mo, 'ib_sel', add=+1, color='black', linestyle='-', label='ib_sel'+ref_str+'+1')
        plq(plt, mv, 'time', mv, 'ib_sel', add=+1, color='red', linestyle='--', label='ib_sel'+test_str+'+1')
        plq(plt, mo, 'time', mo, 'ib_charge', add=+1, linestyle='-.', color='green', label='ib_charge'+ref_str+'+1')
        plq(plt, mv, 'time', mv, 'ib_charge', add=+1, linestyle=':', color='blue', label='ib_charge'+test_str+'+1')
        plt.legend(loc=1)
        plt.subplot(222)
        plq(plt, mo, 'time', mo, 'ib_sel_stat', color='black', linestyle='-', label='ib_sel_stat'+ref_str)
        plq(plt, mv, 'time', mv, 'ib_sel_stat', color='red', linestyle='--', label='ib_sel_stat'+test_str)
        plq(plt, mo, 'time', mo, 'ib_dec', add=2, color='black', linestyle='-', label='ib_dec'+ref_str+'+2')
        plq(plt, mv, 'time', mv, 'ib_dec', add=2, color='red', linestyle='--', label='ib_dec'+test_str+'+2')
        plt.legend(loc=1)
        plt.subplot(223)
        plq(plt, mo, 'time', mo, 'e_wrap', color='black', linestyle='-', label='e_wrap'+ref_str)
        plq(plt, mv, 'time', mv, 'e_wrap', color='red', linestyle='--', label='e_wrap'+test_str)
        plq(plt, mo, 'time', mo, 'e_wrap_filt', color='black', linestyle='-.', label='e_wrap_filt'+ref_str)
        plt.plot(mo.time, -mo.y_ekf, color='green', linestyle='-.', label='-y_ekf'+ref_str)
        plq(plt, mo, 'time', mo, 'y_ekf_f', slr=-1, color='black', linestyle=':', label='-y_ekf_f'+ref_str)
        plq(plt, mv, 'time', mv, 'y_ekf_f', slr=-1, color='red', linestyle=':', label='-y_ekf_f'+test_str)
        plt.legend(loc=1)
        plt.subplot(224)
        plq(plt, mo, 'time', mo, 'cc_dif', color='black', linestyle='-', label='cc_diff'+ref_str)
        plq(plt, mv, 'time', mv, 'cc_dif', color='red', linestyle='--', label='cc_diff'+test_str)
        plt.legend(loc=1)
        fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
        fig_files.append(fig_file_name)
        plt.savefig(fig_file_name, format="png")

        # fig_list.append(plt.figure())  # DOM 1
        # plt.subplot(331)
        # plt.title(plot_title + ' DOM 1')
        # plt.plot(mo.time, mo.ib_charge, color='green', linestyle='-', label='ib_charge'+ref_str)
        # plt.plot(mv.time, mv.ib_charge, linestyle='--', color='red', label='ib_charge'+test_str)
        # plt.plot(mo.time, mo.ib_diff, color='black', linestyle='-.', label='ib_diff'+ref_str)
        # plt.plot(mo.time, mo.ib_diff_f, color='magenta', linestyle=':', label='ib_diff_f'+ref_str)
        # plt.plot(mo.time, mo.ibd_thr, color='black', linestyle='--', label='ib_diff_thr'+ref_str)
        # plt.plot(mo.time, -mo.ibd_thr, color='black', linestyle='--')
        # plt.legend(loc=1)
        # plt.subplot(338)
        # mod_min = min(min(mo.mod_data), min(mv.mod_data))
        # plt.plot(mo.time, mo.mod_data-mod_min, color='blue', linestyle='-', label='mod'+ref_str+'-'+str(mod_min))
        # plt.plot(mv.time, mv.mod_data-mod_min, color='red', linestyle='--', label='mod'+test_str+'-'+str(mod_min))
        # plt.plot(mo.time, mo.sat+2, color='black', linestyle='-',  label='sat'+ref_str+'+2')
        # plt.plot(mv.time, np.array(mv.sat)+2, color='green', linestyle='--', label='sat'+test_str+'+2')
        # plt.plot(mo.time, np.array(mo.bms_off)+4, color='red', linestyle='-', label='bms_off'+ref_str+'+4')
        # plt.plot(mv.time, np.array(mv.bms_off)+4, color='green', linestyle='--', label='bms_off'+test_str+'+4')
        # if so is not None:
        #     plt.plot(so.time, np.array(so.bms_off_s)+4, color='blue', linestyle='-.', label='bms_off_s'+ref_str+'+4')
        # if smv is not None:
        #     if hasattr(smv, 'bmso_s'):
        #         plt.plot(smv.time, np.array(smv.bmso_s)+4, color='orange', linestyle=':',
        #                  label='bms_off_s'+test_str+'+4')
        #     elif hasattr(smv, 'bms_off_s'):
        #         plt.plot(smv.time, np.array(smv.bms_off_s) + 4, color='orange', linestyle=':',
        #                  label='bms_off_s' + test_str + '+4')
        # plt.plot(mo.time, mo.sel, color='red', linestyle='-.', label='sel'+ref_str)
        # plt.plot(mv.time, mv.sel, color='blue', linestyle=':', label='sel'+test_str)
        # plt.plot(mo.time, mo.ib_sel_stat-2, color='black', linestyle='-', label='ib_sel_stat'+ref_str+'-2')
        # plt.plot(mo.time, mo.vb_sel-2, color='green', linestyle='--', label='vb_sel_stat'+ref_str+'-2')
        # plt.plot(mo.time, mo.preserving-2, color='cyan', linestyle='-.', label='preserving'+ref_str+'-2')
        # plt.legend(loc=1)
        # plt.subplot(333)
        # plt.plot(mo.time, mo.vb, color='green', linestyle='-', label='vb'+ref_str)
        # plt.plot(mv.time, mv.vb, color='orange', linestyle='--', label='vb'+test_str)
        # plt.legend(loc=1)
        # plt.subplot(334)
        # plt.plot(mo.time, mo.voc_stat, color='green', linestyle='-', label='voc_stat'+ref_str)
        # plt.plot(mv.time, mv.voc_stat, color='orange', linestyle='--', label='voc_stat'+test_str)
        # plt.plot(mo.time, mo.vsat, color='blue', linestyle='-', label='vsat'+ref_str)
        # plt.plot(mv.time, mv.vsat, color='red', linestyle='--', label='vsat'+test_str)
        # plt.plot(mo.time, mo.voc_soc+0.1*1, color='black', linestyle='-', label='voc_soc'+ref_str+'+0.1')
        # plt.plot(mo.time, mo.voc+0.1*1, color='cyan', linestyle='--', label='voc'+ref_str+'+0.1')
        # plt.plot(mv.time, np.array(mv.voc)+0.1*1, color='red', linestyle='-.', label='voc'+test_str+'+0.1')
        # plt.legend(loc=1)
        # plt.subplot(335)
        # plt.plot(mo.time, mo.e_wrap, color='magenta', linestyle='--', label='e_wrap'+ref_str)
        # plt.plot(mv.time, mv.e_wrap, color='cyan', linestyle='--', label='e_wrap'+test_str)
        # plq(plt, mo, 'time', mo, 'e_wrap_filt', color='black', linestyle='-', label='e_wrap_filt'+ref_str)
        # plq(plt, mv, 'time', mv, 'e_wrap_filt', color='red', linestyle='-', label='e_wrap_filt'+test_str)
        # plt.plot(mo.time, mo.ewh_thr, color='red', linestyle='-.', label='ewh_thr'+ref_str)
        # plt.plot(mo.time, mo.ewl_thr, color='red', linestyle='-.', label='ewl_thr'+ref_str)
        # plt.ylim(-1, 1)
        # plt.legend(loc=1)
        # plt.subplot(336)
        # plt.plot(mo.time, mo.wh_flt+8, color='blue', linestyle='-', label='wrap_hi_flt'+ref_str+'+8')
        # plq(plt, mo, 'time', mo, 'wh_m_flt', add=+8, color='red', linestyle='--', label='wrap_hi_m_flt'+ref_str+'+8')
        # plq(plt, mo, 'time', mo, 'wh_n_flt', add=8, color='orange', linestyle='-.', label='wrap_hi_n_flt'+ref_str+'+8')
        # plt.plot(mo.time, mo.wl_flt+6, color='blue', linestyle='-', label='wrap_lo_flt'+ref_str+'+6')
        # plq(plt, mo, 'time', mo, 'wl_m_flt', add=+6, color='red', linestyle='--', label='wrap_lo_m_flt'+ref_str+'+6')
        # plq(plt, mo, 'time', mo, 'wl_n_flt', add=+6, color='orange', linestyle='-.', label='wrap_lo_n_flt'+ref_str+'+6')
        # plt.plot(mo.time, mo.wh_fa+4, color='blue', linestyle='-', label='wrap_hi_fa'+ref_str+'+2')
        # plq(plt, mo, 'time', mo, 'wh_m_fa', add=4, color='red', linestyle='--', label='wrap_hi_m_fa'+ref_str+'+2')
        # plq(plt, mo, 'time', mo, 'wh_m_fa', add=+4, color='red', linestyle='--', label='wrap_hi_m_fa'+ref_str+'+2')
        # plq(plt, mo, 'time', mo, 'wh_n_fa', add=+4, color='orange', linestyle='-.', label='wrap_hi_n_fa'+ref_str+'+2')
        # plq(plt, mo, 'time', mo, 'wl_fa', add=+2, color='red', linestyle='--', label='wrap_lo_fa'+ref_str+'+2')
        # plq(plt, mo, 'time', mo, 'wv_fa', add=+2, color='orange', linestyle='-.', label='wrap_vb_fa'+ref_str+'+2')
        # plt.plot(mo.time, mo.ccd_fa, color='green', linestyle='-', label='cc_diff_fa'+ref_str)
        # plt.plot(mo.time, mo.red_loss, color='blue', linestyle='--', label='red_loss'+ref_str)
        # plt.legend(loc=1)
        # plt.subplot(337)
        # plt.plot(mo.time, mo.cc_dif, color='black', linestyle='-', label='cc_diff'+ref_str)
        # plt.plot(mo.time, mo.ccd_thr, color='red', linestyle='--', label='cc_diff_thr'+ref_str)
        # plt.plot(mo.time, -mo.ccd_thr, color='red', linestyle='--')
        # plt.ylim(-.01, .01)
        # plt.legend(loc=1)
        # plt.subplot(332)
        # # plt.plot(mo.time, mo.ib_rate, color='orange', linestyle='--', label='ib_rate'+ref_str)
        # plt.plot(mo.time, mo.ib_quiet, color='black', linestyle='-', label='ib_quiet'+ref_str)
        # plt.plot(mo.time, mo.ibq_thr, color='red', linestyle='--', label='ib_quiet_thr'+ref_str)
        # plt.plot(mo.time, -mo.ibq_thr, color='red', linestyle='--')
        # plt.plot(mo.time, mo.dscn_flt-4, color='blue', linestyle='-.', label='dscn_flt'+ref_str+'-4')
        # plt.plot(mo.time, mo.dscn_fa+4, color='red', linestyle='-', label='dscn_fa'+ref_str+'+4')
        # plt.legend(loc=1)
        # plt.subplot(339)
        # plt.plot(mo.time, mo.ib_diff_flt+2, color='cyan', linestyle='-', label='ib_diff_flt'+ref_str+'+2')
        # plt.plot(mo.time, mo.ib_diff_fa+2, color='magenta', linestyle='--', label='ib_diff_fa'+ref_str+'+2')
        # plt.plot(mo.time, mo.vb_flt, color='blue', linestyle='-.', label='vb_flt'+ref_str)
        # plt.plot(mo.time, mo.vb_fa, color='black', linestyle=':', label='vb_fa'+ref_str)
        # plt.plot(mo.time, mo.tb_flt, color='red', linestyle='-', label='tb_flt'+ref_str)
        # plt.plot(mo.time, mo.tb_fa, color='cyan', linestyle='--', label='tb_fa'+ref_str)
        # plt.plot(mo.time, mo.ib_sel_stat-2, color='black', linestyle='-', label='ib_sel_stat'+ref_str+'-2')
        # plt.plot(mo.time, mo.vb_sel-2, color='green', linestyle='--', label='vb_sel_stat'+ref_str+'-2')
        # plt.plot(mo.time, mo.tb_sel-2, color='red', linestyle='-.', label='tb_sel_stat'+ref_str+'-2')
        # plt.plot(mo.time, np.array(mo.chm)-0.5, color='blue', linestyle='-', label='chm'+ref_str+'-0.5')
        # plt.plot(mv.time, np.array(mv.chm)-0.5, color='red', linestyle='--', label='chm'+test_str+'-0.5')
        # plt.legend(loc=1)
        # fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
        # fig_files.append(fig_file_name)
        # plt.savefig(fig_file_name, format="png")

    fig_list.append(plt.figure())  # DOM 2
    plt.subplot(321)
    plt.title(plot_title + ' DOM 2')
    plt.plot(mo.time, mo.dv_dyn, color='green', linestyle='-', label='dv_dyn'+ref_str)
    plt.plot(mv.time, mv.dv_dyn, color='orange', linestyle='--', label='dv_dyn'+test_str)
    plt.legend(loc=1)
    plt.subplot(322)
    plt.plot(mo.time, mo.voc_stat, color='green', linestyle='-', label='voc_stat'+ref_str)
    plt.plot(mv.time, mv.voc_stat, color='orange', linestyle='--', label='voc_stat'+test_str)
    plt.legend(loc=1)
    plt.subplot(323)
    plt.plot(mo.time, mo.voc, color='green', linestyle='-', label='voc'+ref_str)
    plt.plot(mv.time, mv.voc, color='orange', linestyle='--', label='voc'+test_str)
    plt.plot(mo.time, mo.voc_ekf, color='blue', linestyle='-.', label='voc_ekf'+ref_str)
    plt.plot(mv.time, mv.voc_ekf, color='red', linestyle=':', label='voc_ekf'+test_str)
    plt.legend(loc=1)
    plt.subplot(324)
    plt.plot(mo.time, mo.y_ekf, color='green', linestyle='-', label='y_ekf'+ref_str)
    plt.plot(mv.time, mv.y_ekf, color='orange', linestyle='--', label='y_ekf'+test_str)
    plq(plt, mv, 'time', mv, 'y_filt', color='black', linestyle='-.', label='y_filt'+test_str)
    plq(plt, mv, 'time', mv, 'y_filt2', color='cyan', linestyle=':', label='y_filt2'+test_str)
    plt.legend(loc=1)
    plt.subplot(325)
    plt.plot(mo.time, mo.dv_hys, color='green', linestyle='-', label='dv_hys'+ref_str)
    plt.plot(mv.time, mv.dv_hys, color='cyan', linestyle='--', label='dv_hys'+test_str)
    if hasattr(sv, 'time'):
        import myTables
        lut_vb = myTables.TableInterp1D(np.array(mo.time), np.array(mo.vb))
        n = len(sv.time)
        voc_req = np.zeros((n, 1))
        dv_hys_req = np.zeros((n, 1))
        voc_stat_req = np.zeros((n, 1))
        if hasattr(smv, 'dv_dyn_s'):
            for i in range(n):
                voc_req[i] = lut_vb.interp(sv.time[i]) - smv.dv_dyn_s[i]
                voc_stat_req[i] = smv.voc_stat_s[i]
                dv_hys_req[i] = voc_req[i] - smv.voc_stat_s[i]
            plt.plot(smv.time, np.array(smv.dv_hys_s)+0.1, color='red', linestyle='-', label='dv_hys_s'+test_str+'+0.1')
            plt.plot(sv.time, np.array(dv_hys_req)+0.1, color='black', linestyle='--', label='dv_hys_req_s'+test_str+'+0.1')

    if so is not None:
        import myTables
        lut_vb = myTables.TableInterp1D(np.array(mo.time), np.array(mo.vb))
        n = len(so.time)
        voc_req = np.zeros((n, 1))
        dv_hys_req = np.zeros((n, 1))
        voc_stat_req = np.zeros((n, 1))
        for i in range(n):
            voc_req[i] = lut_vb.interp(so.time[i]) - so.dv_dyn_s[i]
            voc_stat_req[i] = so.voc_stat_s[i]
            dv_hys_req[i] = voc_req[i] - so.voc_stat_s[i]
        plt.plot(so.time, np.array(so.dv_hys_s)-0.1, color='magenta',  linestyle='-', label='dv_hys_s-0.1'+ref_str)
        plt.plot(so.time, np.array(dv_hys_req)-0.1, color='orange', linestyle='--', label='dv_hys_req_s-0.1'+ref_str)
    plt.legend(loc=1)
    plt.subplot(326)
    plt.plot(mo.time, mo.Tb, color='green', linestyle='-', label='temp_c'+ref_str)
    plt.plot(mv.time, mv.Tb, color='orange', linestyle='--', label='temp_c'+test_str)
    plt.plot(mo.time, mo.chm, color='black', linestyle='-', label='mon_chm'+ref_str)
    plq(plt, so, 'time', so, 'chm_s', color='cyan', linestyle='--', label='sim_chm'+ref_str)
    plt.ylim(0., 50.)
    plt.legend(loc=1)
    fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    fig_files.append(fig_file_name)
    plt.savefig(fig_file_name, format="png")

    fig_list.append(plt.figure())  # DOM 3
    plt.subplot(221)
    plt.title(plot_title + ' DOM 3')
    plt.plot(mo.time, mo.soc, color='blue', linestyle='-', label='soc'+ref_str)
    plt.plot(mv.time, mv.soc, color='red', linestyle='--', label='soc'+test_str)
    plt.legend(loc=1)
    plt.subplot(222)
    plt.plot(mo.time, mo.soc_ekf, color='green', linestyle='-', label='soc_ekf'+ref_str)
    plt.plot(mv.time, mv.soc_ekf, color='orange', linestyle='--', label='soc_ekf'+test_str)
    plt.legend(loc=1)
    plt.subplot(223)
    plt.plot(mo.time, mo.soc_s, color='blue', linestyle='-.', label='soc_s'+ref_str)
    plt.plot(mv.time, mv.soc_s, color='red', linestyle=':', label='soc_s'+test_str)
    plt.legend(loc=1)
    plt.subplot(224)
    plt.plot(mo.time, mo.soc, color='blue', linestyle='-', label='soc'+ref_str)
    plt.plot(mv.time, mv.soc, color='red', linestyle='--', label='soc'+test_str)
    plt.plot(mo.time, mo.soc_s, color='green', linestyle='-.', label='soc_s'+ref_str)
    plt.plot(mv.time, mv.soc_s, color='orange', linestyle=':', label='soc_s'+test_str)
    plt.plot(mo.time, mo.soc_ekf, color='cyan', linestyle='-', label='soc_ekf'+ref_str)
    plt.plot(mv.time, mv.soc_ekf, color='black', linestyle='--', label='soc_ekf'+test_str)
    plt.legend(loc=1)
    fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    fig_files.append(fig_file_name)
    plt.savefig(fig_file_name, format="png")

    fig_list.append(plt.figure())  # DOM 4
    plt.subplot(131)
    plt.title(plot_title + ' DOM 4')
    plt.plot(mo.time, mo.soc, color='orange', linestyle='-', label='soc'+ref_str)
    plt.plot(mv.time, mv.soc, color='green', linestyle='--', label='soc'+test_str)
    plq(plt, smv, 'time', smv, 'soc_s', color='black', linestyle='-.', label='soc_s'+test_str)
    plt.plot(mo.time, mo.soc_ekf, color='red', linestyle=':', label='soc_ekf'+ref_str)
    plt.plot(mv.time, mv.soc_ekf, color='cyan', linestyle=':', label='soc_ekf'+test_str)
    plt.legend(loc=1)
    plt.subplot(132)
    plt.plot(mo.time, mo.vb, color='orange', linestyle='-', label='vb'+ref_str)
    plq(plt, mo, 'time', mo, 'vb_h', color='cyan', linestyle='--', label='vb_hdwe'+ref_str)
    plt.plot(mv.time, mv.vb, color='green', linestyle='-.', label='vb'+test_str)
    plq(plt, smv, 'time', smv, 'vb_s', color='black', linestyle=':', label='vb_s'+test_str)
    plt.legend(loc=1)
    plt.subplot(133)
    plt.plot(mo.soc, mo.vb, color='orange', linestyle='-', label='vb'+ref_str)
    plq(plt, mo, 'soc', mo, 'vb_h', color='cyan', linestyle='--', label='vb_hdwe'+ref_str)
    plt.plot(mv.soc, mv.vb, color='green', linestyle='-.', label='vb'+test_str)
    plq(plt, smv, 'soc_s', smv, 'vb_s', color='black', linestyle=':', label='vb_s'+test_str)
    plt.legend(loc=1)
    fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    fig_files.append(fig_file_name)
    plt.savefig(fig_file_name, format="png")

    fig_list.append(plt.figure())  # DOM 4a
    plt.subplot(311)
    plt.title(plot_title + ' DOM 4a')
    plt.plot(mo.time, mo.ib, color='orange', linestyle='-', label='ib'+ref_str)
    plt.plot(mv.time, mv.ib, color='green', linestyle='--', label='ib'+test_str)
    plt.plot(mo.time, mo.ib_charge, color='red', linestyle='-.', label='ib_charge'+ref_str)
    plt.plot(mv.time, mv.ib_charge, color='black', linestyle=':', label='ib_charge'+test_str)
    plt.legend(loc=1)
    plt.subplot(312)
    plq(plt, smv, 'time', smv, 'soc_s', color='black', linestyle='-.', label='soc_s'+test_str)
    plt.plot(mo.time, mo.soc, color='red', linestyle=':', label='soc'+ref_str)
    plt.plot(mv.time, mv.soc, color='cyan', linestyle=':', label='soc'+test_str)
    plt.legend(loc=1)
    plt.subplot(313)
    plt.plot(mo.time, mo.Tb, color='red', linestyle=':', label='Tb'+ref_str)
    plt.plot(mv.time, mv.Tb, color='cyan', linestyle=':', label='Tb'+test_str)
    plt.legend(loc=1)
    fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    fig_files.append(fig_file_name)
    plt.savefig(fig_file_name, format="png")

    # fig_list.append(plt.figure())  # DOM 5
    # plt.subplot(231)
    # plt.title(plot_title + ' DOM 5')
    # plt.plot(mo.time, mo.ib_charge, color='black', linestyle='-', label='ib_charge' + ref_str)
    # plt.plot(mv.time, mv.ib_charge, linestyle='--', color='blue', label='ib_charge' + test_str)
    # plt.plot(mo.time, mo.ib_diff_flt + 2, color='green', linestyle='-', label='ib_diff_flt' + ref_str + '+2')
    # plq(plt, mv, 'time', mv, 'ib_diff_flt', add=+2, color='red', linestyle='--', label='ib_diff_flt' + test_str + '+2')
    # plt.plot(mo.time, mo.ib_diff_fa + 2, color='magenta', linestyle='-', label='ib_diff_fa' + ref_str + '+2')
    # plq(plt, mv, 'time', mv, 'ib_diff_fa', add=+2, color='cyan', linestyle='--', label='ib_diff_fa' + test_str + '+2')
    # plt.legend(loc=1)
    # plt.subplot(232)
    # plq(plt, mo, 'time', mo, 'wh_flt', add=+12, color='green', linestyle='-', label='wrap_hi_flt' + ref_str + '+12')
    # plq(plt, mv, 'time', mv, 'wh_flt', add=+12, color='red', linestyle='--', label='wrap_hi_flt' + test_str + '+12')
    # plq(plt, mo, 'time', mo, 'wh_m_flt', add=+10, color='green', linestyle='-', label='wrap_hi_m_flt' + ref_str + '+10')
    # plq(plt, mv, 'time', mv, 'wh_m_flt', add=+10, color='red', linestyle='--', label='wrap_hi_m_flt' + test_str + '+10')
    # plq(plt, mo, 'time', mo, 'wh_n_flt', add=+8, color='green', linestyle='-', label='wrap_hi_n_flt' + ref_str + '+8')
    # plq(plt, mv, 'time', mv, 'wh_n_flt', add=+8, color='red', linestyle='--', label='wrap_hi_n_flt' + test_str + '+8')
    # plq(plt, mo, 'time', mo, 'wl_flt', add=+6, color='green', linestyle='-', label='wrap_lo_flt' + ref_str + '+6')
    # plq(plt, mv, 'time', mv, 'wl_flt', add=+6, color='red', linestyle='--', label='wrap_lo_flt' + test_str + '+6')
    # plq(plt, mo, 'time', mo, 'wl_m_flt', add=+4, color='green', linestyle='-', label='wrap_lo_m_flt' + ref_str + '+4')
    # plq(plt, mv, 'time', mv, 'wl_m_flt', add=+4, color='red', linestyle='--', label='wrap_lo_m_flt' + test_str + '+4')
    # plq(plt, mo, 'time', mo, 'wl_n_flt', add=+2, color='green', linestyle='-', label='wrap_lo_n_flt' + ref_str + '+2')
    # plq(plt, mv, 'time', mv, 'wl_n_flt', add=+2, color='red', linestyle='--', label='wrap_lo_n_flt' + test_str + '+2')
    # plt.legend(loc=1)
    # plt.subplot(233)
    # plq(plt, mo, 'time', mo, 'e_wrap', color='magenta', linestyle='--', label='e_wrap' + ref_str)
    # plq(plt, mo, 'time', mo, 'e_wrap_filt', color='black', linestyle='-', label='e_wrap_filt' + ref_str)
    # plq(plt, mv, 'time', mv, 'e_wrap', color='red', linestyle='-.', label='e_wrap' + test_str)
    # plq(plt, mv, 'time', mv, 'e_wrap_filt', color='blue', linestyle='--', label='e_wrap_filt' + test_str)
    # plq(plt, mv, 'time', mv, 'e_wrap_m', color='cyan', linestyle='-.', label='e_wrap_m' + test_str)
    # plq(plt, mo, 'time', mv, 'e_wrap_m_filt', color='blue', linestyle='-', label='e_wrap_m_filt' + test_str)
    # plq(plt, mv, 'time', mv, 'e_wrap_n', color='cyan', linestyle='-.', label='e_wrap_n' + test_str)
    # plq(plt, mv, 'time', mv, 'e_wrap_n_filt', color='green', linestyle='--', label='e_wrap_n_filt' + test_str)
    # plq(plt, mo, 'time', mo, 'cc_dif', color='green', linestyle='-', label='cc_diff'+ref_str)
    # plq(plt, mv, 'time', mv, 'cc_dif', color='red', linestyle='--', label='cc_diff'+test_str)
    # plt.ylim(-1, 1)
    # plt.legend(loc=1)
    # plt.subplot(234)
    # plq(plt, mo, 'time', mo, 'ib_sel_stat', add=-2, color='black', linestyle='-', label='ib_sel_stat' + ref_str + '-2')
    # plq(plt, mv, 'time', mv, 'ib_sel_stat', add=-2, color='blue', linestyle='--', label='ib_sel_stat' + test_str + '-2')
    # plq(plt, mo, 'time', mo, 'tb_flt', color='green', linestyle='-', label='tb_flt' + ref_str)
    # plq(plt, mv, 'time', mv, 'tb_flt', color='red', linestyle='--', label='tb_flt' + test_str)
    # plq(plt, mo, 'time', mo, 'tb_fa', color='magenta', linestyle='-.', label='tb_fa' + ref_str)
    # plq(plt, mv, 'time', mv, 'tb_fa', color='cyan', linestyle=':', label='tb_fa' + test_str)
    # plq(plt, mo, 'time', mo, 'vb_sel', add=+2, color='magenta', linestyle='-', label='vb_sel_stat' + ref_str + '+2')
    # plq(plt, mv, 'time', mv, 'vb_sel', add=+2, color='cyan', linestyle='--', label='vb_sel_stat' + test_str + '+2')
    # plq(plt, mo, 'time', mo, 'tb_sel', add=+6, color='green', linestyle='-', label='tb_sel_stat' + ref_str + '+6')
    # plq(plt, mv, 'time', mv, 'tb_sel', add=+6, color='red', linestyle='--', label='tb_sel_stat' + test_str + '+6')
    # plt.legend(loc=1)
    # plt.subplot(235)
    # plq(plt, mo, 'time', mo, 'wh_fa', add=+12, color='green', linestyle='-', label='wrap_hi_fa' + ref_str + '+12')
    # plq(plt, mv, 'time', mv, 'wh_fa', add=+12, color='red', linestyle='--', label='wrap_hi_fa' + test_str + '+12')
    # plq(plt, mo, 'time', mo, 'wh_m_fa', add=+10, color='green', linestyle='-', label='wrap_hi_m_fa' + ref_str + '+10')
    # plq(plt, mv, 'time', mv, 'wh_m_fa', add=+10, color='red', linestyle='--', label='wrap_hi_m_fa' + test_str + '+10')
    # plq(plt, mo, 'time', mo, 'wh_n_fa', add=+8, color='green', linestyle='-', label='wrap_hi_n_fa' + ref_str + '+8')
    # plq(plt, mv, 'time', mv, 'wh_n_fa', add=+8, color='red', linestyle='--', label='wrap_hi_n_fa' + test_str + '+8')
    # plq(plt, mo, 'time', mo, 'wl_fa', add=+6, color='green', linestyle='-', label='wrap_lo_fa' + ref_str + '+6')
    # plq(plt, mv, 'time', mv, 'wl_fa', add=+6, color='red', linestyle='--', label='wrap_lo_fa' + test_str + '+6')
    # plq(plt, mo, 'time', mo, 'wl_m_fa', add=+4, color='green', linestyle='-', label='wrap_lo_fa' + ref_str + '+4')
    # plq(plt, mv, 'time', mv, 'wl_m_fa', add=+4, color='red', linestyle='--', label='wrap_lo_m_fa' + test_str + '+4')
    # plq(plt, mv, 'time', mo, 'wl_n_fa', add=+2, color='green', linestyle='-.', label='wrap_lo_fa' + ref_str + '+2')
    # plq(plt, mv, 'time', mv, 'wl_n_fa', add=+2, color='red', linestyle='-.', label='wrap_lo_n_fa' + test_str + '+2')
    # plt.legend(loc=1)
    # plt.subplot(236)
    # plt.plot(mo.time, mo.red_loss, color='green', linestyle='-', label='red_loss' + ref_str)
    # plq(plt, mv, 'time', mv, 'red_loss', color='red', linestyle='--', label='red_loss' + test_str)
    # plt.plot(mo.time, mo.wv_fa - 2, color='green', linestyle='-', label='wrap_vb_fa' + ref_str + '-2')
    # plq(plt, mv, 'time', mv, 'wv_fa', add=-2, color='red', linestyle='--', label='wrap_vb_fa' + test_str + '-2')
    # plt.plot(mo.time, mo.ccd_fa - 4, color='green', linestyle='-', label='cc_diff_fa' + ref_str + '-4')
    # plq(plt, mv, 'time', mv, 'ccd_fa', add=-4, color='red', linestyle='--', label='cc_diff_fa' + test_str + '-4')
    # plt.legend(loc=1)

    # fig_list.append(plt.figure())  # DOM 6
    # plt.subplot(221)
    # plt.title(plot_title + ' DOM 6')
    # plq(plt, mo, 'time', mo, 'ibmh', color='blue', linestyle='-', label='ib_amp_hdwe' + ref_str)
    # plq(plt, mo, 'time', mo, 'ibnh', color='green', linestyle='-', label='ib_noa_hdwe' + ref_str)
    # plq(plt, mo, 'time', mo, 'ib_sel', color='red', linestyle='--', label='ib_sel' + ref_str)
    # plq(plt, mo, 'time', mo, 'ib_charge', linestyle=':', color='blue', label='ib_charge' + ref_str)
    # plq(plt, mo, 'time', mo, 'ib_diff', color='orange', linestyle='-', label='ib_diff' + ref_str)
    # plq(plt, mo, 'time', mo, 'ib_diff_f', color='magenta', linestyle='--', label='ib_diff_f' + ref_str)
    # plq(plt, mo, 'time', mo, 'ibd_thr', color='red', linestyle=':', label='ib_diff_thr' + ref_str)
    # plq(plt, mo, 'time', mo, 'ibd_thr', slr=-1., color='red', linestyle=':')
    # plq(plt, mo, 'time', mo, 'ib_diff_flt', add=+2, color='green', linestyle='-', label='ib_diff_flt' + ref_str + '+2')
    # plq(plt, mo, 'time', mo, 'ib_diff_fa', add=+2, color='magenta', linestyle='-', label='ib_diff_fa' + ref_str + '+2')
    # plt.legend(loc=1)
    # plt.subplot(222)
    # plq(plt, mo, 'time', mo, 'wh_m_flt', add=+10, color='green', linestyle='-', label='wrap_hi_m_flt' + ref_str + '+10')
    # plq(plt, mv, 'time', mv, 'wh_m_flt', add=+10, color='blue', linestyle='--', label='wrap_hi_m_flt' + test_str + '+10')
    # plq(plt, mo, 'time', mo, 'wh_m_fa', add=+10, color='red', linestyle='-.', label='wrap_hi_m_fa' + ref_str + '+10')
    # plq(plt, mv, 'time', mv, 'wh_m_fa', add=+10, color='cyan', linestyle=':', label='wrap_hi_m_fa' + test_str + '+10')
    # plq(plt, mo, 'time', mo, 'wh_n_flt', add=+8, color='green', linestyle='-', label='wrap_hi_n_flt' + ref_str + '+8')
    # plq(plt, mv, 'time', mv, 'wh_n_flt', add=+8, color='blue', linestyle='--', label='wrap_hi_n_flt' + test_str + '+8')
    # plq(plt, mo, 'time', mo, 'wh_n_fa', add=+8, color='red', linestyle='-.', label='wrap_hi_n_fa' + ref_str + '+8')
    # plq(plt, mv, 'time', mv, 'wh_n_fa', add=+8, color='cyan', linestyle=':', label='wrap_hi_n_fa' + test_str + '+8')
    # plq(plt, mo, 'time', mo, 'wl_m_flt', add=+4, color='green', linestyle='-', label='wrap_lo_m_flt' + ref_str + '+4')
    # plq(plt, mv, 'time', mv, 'wl_m_flt', add=+4, color='blue', linestyle='--', label='wrap_lo_m_flt' + test_str + '+4')
    # plq(plt, mo, 'time', mo, 'wl_m_fa', add=+4, color='red', linestyle='-.', label='wrap_lo_fa' + ref_str + '+4')
    # plq(plt, mv, 'time', mv, 'wl_m_fa', add=+4, color='cyan', linestyle=':', label='wrap_lo_m_fa' + test_str + '+4')
    # plq(plt, mo, 'time', mo, 'wl_n_flt', add=+2, color='green', linestyle='-', label='wrap_lo_n_flt' + ref_str + '+2')
    # plq(plt, mv, 'time', mv, 'wl_n_flt', add=+2, color='blue', linestyle='--', label='wrap_lo_n_flt' + test_str + '+2')
    # plq(plt, mv, 'time', mo, 'wl_n_fa', add=+2, color='red', linestyle='-.', label='wrap_lo_fa' + ref_str + '+2')
    # plq(plt, mv, 'time', mv, 'wl_n_fa', add=+2, color='cyan', linestyle=':', label='wrap_lo_n_fa' + test_str + '+2')
    # plt.legend(loc=1)
    # plt.subplot(223)
    # plq(plt, mo, 'time', mo, 'e_wm_f', color='blue', linestyle='-', label='e_wrap_m_filt' + ref_str)
    # plq(plt, mo, 'time', mo, 'e_wrap_m_filt', color='blue', linestyle='-', label='e_wrap_m_filt' + ref_str)
    # plq(plt, mv, 'time', mv, 'e_wm_f', color='blue', linestyle='--', label='e_wrap_m_filt' + test_str)
    # plq(plt, mv, 'time', mv, 'e_wrap_m', color='cyan', linestyle='-.', label='e_wrap_m' + test_str)
    # plq(plt, mv, 'time', mv, 'e_wrap_m_trim', color='magenta', linestyle='-', label='e_wrap_m_trim' + test_str)
    # plq(plt, mv, 'time', mv, 'e_wrap_m_filt', color='blue', linestyle='--', label='e_wrap_m_filt' + test_str)
    # # plq(plt, mo, 'time', mo, 'e_wrap_n_filt', color='green', linestyle='-', label='e_wrap_n_filt' + ref_str)
    # plq(plt, mo, 'time', mo, 'ewh_thr', color='red', linestyle='-.', label='ewh_thr' + ref_str)
    # plq(plt, mo, 'time', mo, 'ewl_thr', color='red', linestyle='-.', label='ewl_thr' + ref_str)
    # plq(plt, mv, 'time', mv, 'ewmhi_thr', color='orange', linestyle=':', label='ewmhi_thr' + test_str)
    # plq(plt, mv, 'time', mv, 'ewmlo_thr', color='orange', linestyle=':', label='ewmlo_thr' + test_str)
    # plt.ylim(-0.2, 0.2)
    # plt.legend(loc=1)
    # plt.subplot(224)
    # plq(plt, mo, 'time', mo, 'e_wn_f', color='green', linestyle='-', label='e_wrap_n_filt' + ref_str)
    # plq(plt, mo, 'time', mo, 'e_wrap_n_filt', color='green', linestyle='-', label='e_wrap_n_filt' + ref_str)
    # plq(plt, mv, 'time', mv, 'e_wn_f', color='green', linestyle='--', label='e_wrap_n_filt' + test_str)
    # plq(plt, mv, 'time', mv, 'e_wrap_n', color='cyan', linestyle='-.', label='e_wrap_n' + test_str)
    # plq(plt, mv, 'time', mv, 'e_wrap_n_trim', color='magenta', linestyle='-', label='e_wrap_n_trim' + test_str)
    # plq(plt, mv, 'time', mv, 'e_wrap_n_filt', color='green', linestyle='--', label='e_wrap_n_filt' + test_str)
    # # plq(plt, mo, 'time', mo, 'e_wrap_m_filt', color='blue', linestyle='-', label='e_wrap_m_filt' + ref_str)
    # plq(plt, mo, 'time', mo, 'ewh_thr', color='red', linestyle=':', label='ewh_thr' + ref_str)
    # plq(plt, mo, 'time', mo, 'ewl_thr', color='red', linestyle=':', label='ewl_thr' + ref_str)
    # plq(plt, mv, 'time', mv, 'ewnhi_thr', color='orange', linestyle=':', label='ewnhi_thr' + test_str)
    # plq(plt, mv, 'time', mv, 'ewnlo_thr', color='orange', linestyle=':', label='ewnlo_thr' + test_str)
    # plt.ylim(-1, 1)
    # plt.legend(loc=1)

    fig_list.append(plt.figure())  # Ult 1
    plt.subplot(331)
    plt.title(plot_title + ' Ult 1')
    plq(plt, mo, 'time', mo, 'ibmh', color='green', linestyle='-', label='ib_amp_hdwe' + ref_str)
    plq(plt, mv, 'time', mv, 'ibmh', color='red', linestyle='--', label='ib_amp_hdwe' + test_str)
    plq(plt, mo, 'time', mo, 'ibnh', color='blue', linestyle='-.', label='ib_noa_hdwe' + ref_str)
    plq(plt, mv, 'time', mv, 'ibnh', color='orange', linestyle=':', label='ib_noa_hdwe' + test_str)
    plq(plt, mo, 'time', mo, 'ibmm', add=1., color='green', linestyle='-', label='ib_amp_model' + ref_str + '+1')
    plq(plt, mv, 'time', mv, 'ibmm', add=1., color='red', linestyle='--', label='ib_amp_model' + test_str + '+1')
    plq(plt, mo, 'time', mo, 'ibnm', add=1., color='blue', linestyle='-.', label='ib_noa_model' + ref_str + '+1')
    plq(plt, mv, 'time', mv, 'ibnm', add=1., color='orange', linestyle=':', label='ib_noa_model' + test_str + '+1')
    plq(plt, mo, 'time', mo, 'ib_diff_f', color='magenta', linestyle='-', label='ib_diff_f' + ref_str)
    plq(plt, mv, 'time', mv, 'ib_diff_f', color='cyan', linestyle='--', label='ib_diff_f' + test_str)
    plq(plt, mo, 'time', mo, 'ibd_thr', color='red', linestyle=':', label='ib_diff_thr' + ref_str)
    plq(plt, mo, 'time', mo, 'ibd_thr', slr=-1., color='red', linestyle=':')
    plt.legend(loc=1)
    plt.subplot(334)
    plq(plt, mo, 'time', mo, 'e_wrap', color='magenta', linestyle='--', label='e_wrap' + ref_str)
    plq(plt, mo, 'time', mo, 'e_wrap_filt', color='black', linestyle='-', label='e_wrap_filt' + ref_str)
    plq(plt, mo, 'time', mo, 'e_w_f', color='black', linestyle='-', label='e_wrap_filt' + ref_str)
    plq(plt, mv, 'time', mv, 'e_wrap', color='red', linestyle='-.', label='e_wrap' + test_str)
    plq(plt, mv, 'time', mv, 'e_wrap_filt', color='blue', linestyle='--', label='e_wrap_filt' + test_str)
    plq(plt, mo, 'time', mo, 'e_wrap_n', color='green', linestyle='-', label='e_wrap_n' + ref_str)
    plq(plt, mv, 'time', mv, 'e_wrap_n', color='pink', linestyle='-.', label='e_wrap_n' + test_str)
    plq(plt, mo, 'time', mo, 'e_wrap_n_filt', color='black', linestyle='-', label='e_wrap_n_filt' + ref_str)
    plq(plt, mo, 'time', mo, 'e_wn_f', color='black', linestyle='-', label='e_wrap_n_filt' + ref_str)
    plq(plt, mv, 'time', mv, 'e_wrap_n_filt', color='green', linestyle='--', label='e_wrap_n_filt' + test_str)
    plq(plt, mo, 'time', mo, 'cc_dif', color='green', linestyle='-', label='cc_diff'+ref_str)
    plq(plt, mv, 'time', mv, 'cc_dif', color='red', linestyle='--', label='cc_diff'+test_str)
    plq(plt, mo, 'time', mo, 'ewh_thr', color='red', linestyle='-.', label='ewh_thr' + ref_str)
    plq(plt, mo, 'time', mo, 'ewl_thr', color='red', linestyle='-.', label='ewl_thr' + ref_str)
    plt.ylim(-1, 1)
    plt.legend(loc=1)
    plt.subplot(332)
    plq(plt, mo, 'time', mo, 'tb_sel', add=+6, color='green', linestyle='-', label='tb_sel_stat' + ref_str + '+6')
    plq(plt, mv, 'time', mv, 'tb_sel', add=+6, color='red', linestyle='--', label='tb_sel_stat' + test_str + '+6')
    plq(plt, mo, 'time', mo, 'vb_sel', add=+2, color='magenta', linestyle='-', label='vb_sel_stat' + ref_str + '+2')
    plq(plt, mv, 'time', mv, 'vb_sel', add=+2, color='cyan', linestyle='--', label='vb_sel_stat' + test_str + '+2')
    plq(plt, mo, 'time', mo, 'tb_flt', color='green', linestyle='-', label='tb_flt' + ref_str)
    plq(plt, mv, 'time', mv, 'tb_flt', color='red', linestyle='--', label='tb_flt' + test_str)
    plq(plt, mo, 'time', mo, 'tb_fa', color='magenta', linestyle='-.', label='tb_fa' + ref_str)
    plq(plt, mv, 'time', mv, 'tb_fa', color='cyan', linestyle=':', label='tb_fa' + test_str)
    plq(plt, mo, 'time', mo, 'ib_sel_stat', add=-2, color='black', linestyle='-', label='ib_sel_stat' + ref_str + '-2')
    plq(plt, mv, 'time', mv, 'ib_sel_stat', add=-2, color='blue', linestyle='--', label='ib_sel_stat' + test_str + '-2')
    plt.legend(loc=1)
    plt.subplot(337)
    plq(plt, mo, 'time', mo, 'e_wrap_m_filt', color='green', linestyle='-', label='e_wrap_m_filt' + ref_str)
    plq(plt, mo, 'time', mo, 'e_wm_f', color='green', linestyle='-', label='e_wrap_m_filt' + ref_str)
    plq(plt, mv, 'time', mv, 'e_wrap_m_filt', color='red', linestyle='--', label='e_wrap_m_filt' + test_str)
    plq(plt, mo, 'time', mo, 'e_wrap_m_trim', color='magenta', linestyle='-.', label='e_wrap_m_trim' + ref_str)
    plq(plt, mv, 'time', mv, 'e_wrap_m_trim', color='cyan', linestyle=':', label='e_wrap_m_trim' + test_str)
    plq(plt, mv, 'time', mv, 'ewmhi_thr', color='red', linestyle='-.', label='ewhm_thr' + test_str)
    plq(plt, mv, 'time', mv, 'ewmlo_thr', color='red', linestyle='-.', label='ewlm_thr' + test_str)
    plt.ylim(-0.2, 0.2)
    plt.legend(loc=1)
    plt.subplot(338)
    plt.plot(mo.time, mo.cc_dif, color='black', linestyle='-', label='cc_diff'+ref_str)
    plt.plot(mo.time, mo.ccd_thr, color='red', linestyle='--', label='cc_diff_thr'+ref_str)
    plt.plot(mo.time, -mo.ccd_thr, color='red', linestyle='--')
    plt.ylim(-.01, .01)
    plt.legend(loc=3)
    plt.subplot(133)
    plq(plt, mo, 'time', mo, 'wh_fa', add=+24, color='green', linestyle='-', label='wrap_hi_fa' + ref_str + '+24')
    plq(plt, mv, 'time', mv, 'wh_fa', add=+24, color='red', linestyle='--', label='wrap_hi_fa' + test_str + '+24')
    plq(plt, mo, 'time', mo, 'wh_flt', add=+22, color='green', linestyle='-', label='wrap_hi_flt' + ref_str + '+22')
    plq(plt, mv, 'time', mv, 'wh_flt', add=+22, color='red', linestyle='--', label='wrap_hi_flt' + test_str + '+22')
    plq(plt, mo, 'time', mo, 'wl_fa', add=+20, color='green', linestyle='-', label='wrap_lo_fa' + ref_str + '+20')
    plq(plt, mv, 'time', mv, 'wl_fa', add=+20, color='red', linestyle='--', label='wrap_lo_fa' + test_str + '+20')
    plq(plt, mo, 'time', mo, 'wl_flt', add=+18, color='green', linestyle='-', label='wrap_lo_flt' + ref_str + '+18')
    plq(plt, mv, 'time', mv, 'wl_flt', add=+18, color='red', linestyle='--', label='wrap_lo_flt' + test_str + '+18')

    plq(plt, mo, 'time', mo, 'wh_m_fa', add=+16, color='green', linestyle='-', label='wrap_hi_m_fa' + ref_str + '+16')
    plq(plt, mv, 'time', mv, 'wh_m_fa', add=+16, color='red', linestyle='--', label='wrap_hi_m_fa' + test_str + '+16')
    plq(plt, mo, 'time', mo, 'wh_m_flt', add=+14, color='green', linestyle='-', label='wrap_hi_m_flt' + ref_str + '+14')
    plq(plt, mv, 'time', mv, 'wh_m_flt', add=+14, color='red', linestyle='--', label='wrap_hi_m_flt' + test_str + '+14')
    plq(plt, mo, 'time', mo, 'wl_m_fa', add=+12, color='green', linestyle='-', label='wrap_lo_m_fa' + ref_str + '+12')
    plq(plt, mv, 'time', mv, 'wl_m_fa', add=+12, color='red', linestyle='--', label='wrap_lo_m_fa' + test_str + '+12')
    plq(plt, mo, 'time', mo, 'wl_m_flt', add=+10, color='green', linestyle='-', label='wrap_lo_m_flt' + ref_str + '+10')
    plq(plt, mv, 'time', mv, 'wl_m_flt', add=+10, color='red', linestyle='--', label='wrap_lo_m_flt' + test_str + '+10')

    plq(plt, mo, 'time', mo, 'wh_n_fa', add=+8, color='green', linestyle='-', label='wrap_hi_n_fa' + ref_str + '+8')
    plq(plt, mv, 'time', mv, 'wh_n_fa', add=+8, color='red', linestyle='--', label='wrap_hi_n_fa' + test_str + '+8')
    plq(plt, mo, 'time', mo, 'wh_n_flt', add=+6, color='green', linestyle='-', label='wrap_hi_n_flt' + ref_str + '+6')
    plq(plt, mv, 'time', mv, 'wh_n_flt', add=+6, color='red', linestyle='--', label='wrap_hi_n_flt' + test_str + '+6')
    plq(plt, mo, 'time', mo, 'wl_n_fa', add=+4, color='green', linestyle='-', label='wrap_lo_n_fa' + ref_str + '+4')
    plq(plt, mv, 'time', mv, 'wl_n_fa', add=+4, color='red', linestyle='--', label='wrap_lo_n_fa' + test_str + '+4')
    plq(plt, mo, 'time', mo, 'wl_n_flt', add=+2, color='green', linestyle='-', label='wrap_lo_n_flt' + ref_str + '+2')
    plq(plt, mv, 'time', mv, 'wl_n_flt', add=+2, color='red', linestyle='--', label='wrap_lo_n_flt' + test_str + '+2')

    plq(plt, mo, 'time', mo, 'red_loss', color='green', linestyle='-', label='red_loss' + ref_str)
    plq(plt, mv, 'time', mv, 'red_loss', color='red', linestyle='--', label='red_loss' + test_str)
    plq(plt, mo, 'time', mo, 'wv_fa', add=-2, color='green', linestyle='-', label='wrap_vb_fa' + ref_str + '-2')
    plq(plt, mv, 'time', mv, 'wv_fa', add=-2, color='red', linestyle='--', label='wrap_vb_fa' + test_str + '-2')
    plq(plt, mo, 'time', mo, 'ccd_fa', add=-4, color='green', linestyle='-', label='cc_diff_fa' + ref_str + '-4')
    plq(plt, mv, 'time', mv, 'ccd_fa', add=-4, color='red', linestyle='--', label='cc_diff_fa' + test_str + '-4')
    plq(plt, mo, 'time', mo, 'ib_diff_fa', add=-6, color='green', linestyle='-', label='ib_diff_fa' + ref_str + '-6')
    plq(plt, mv, 'time', mv, 'ib_diff_fa', add=-6, color='red', linestyle='--', label='ib_diff_fa' + test_str + '-6')
    plq(plt, mo, 'time', mo, 'ib_diff_flt', add=-8, color='green', linestyle='-', label='ib_diff_flt' + ref_str + '-8')
    plq(plt, mv, 'time', mv, 'ib_diff_flt', add=-8, color='red', linestyle='--', label='ib_diff_flt' + test_str + '-8')
    plq(plt, mo, 'time', mo, 'ib_dec', color='blue', linestyle='-.', label='ib_dec' + ref_str)
    plq(plt, mv, 'time', mv, 'ib_dec', color='orange', linestyle=':', label='ib_dec' + test_str)
    plt.legend(loc=1)
    plt.subplot(335)
    plt.plot(mo.time, np.array(mo.bms_off) + 4, color='green', linestyle='-', label='bms_off' + ref_str + '+4')
    plt.plot(mv.time, np.array(mv.bms_off) + 4, color='red', linestyle='--', label='bms_off' + test_str + '+4')
    if so is not None:
        plt.plot(so.time, np.array(so.bms_off_s) + 4, color='blue', linestyle='-.', label='bms_off_s' + ref_str + '+4')
    if hasattr(mo, 'mod_data'):
        mod_min = min(min(mo.mod_data), min(mv.mod_data))
    else:
        mod_min = min(mv.mod_data)
    plq(plt, mo, 'time', mo, 'mod_data', add=-mod_min, color='green', linestyle='-', label='mod' + ref_str + '-' + str(mod_min))
    plq(plt, mv, 'time', mv, 'mod_data', add=-mod_min, color='red', linestyle='--', label='mod' + test_str + '-' + str(mod_min))
    if smv is not None:
        if hasattr(smv, 'bmso_s'):
            plt.plot(smv.time, np.array(smv.bmso_s) + 4, color='orange', linestyle=':',
                     label='bms_off_s' + test_str + '+4')
        elif hasattr(smv, 'bms_off_s'):
            plt.plot(smv.time, np.array(smv.bms_off_s) + 4, color='orange', linestyle=':',
                     label='bms_off_s' + test_str + '+4')
    plt.plot(mo.time, mo.sat + 2, color='green', linestyle='-', label='sat' + ref_str + '+2')
    plt.plot(mv.time, np.array(mv.sat) + 2, color='red', linestyle='--', label='sat' + test_str + '+2')
    plt.plot(mo.time, mo.sel, color='black', linestyle='-.', label='sel' + ref_str)
    plt.plot(mv.time, mv.sel, color='blue', linestyle=':', label='sel' + test_str)
    plq(plt, mo, 'time', mo, 'ib_sel_stat', add=-2, color='green', linestyle='-', label='ib_sel_stat' + ref_str + '-2')
    plq(plt, mv, 'time', mv, 'ib_sel_stat', add=-2, color='red', linestyle=':', label='ib_sel_stat' + test_str + '-2')
    plq(plt, mo, 'time', mo, 'vb_sel', add=-2, color='black', linestyle='--', label='vb_sel_stat' + ref_str + '-2')
    plq(plt, mv, 'time', mv, 'vb_sel', add=-2, color='orange', linestyle='-.', label='vb_sel_stat' + test_str + '-2')
    plq(plt, mo, 'time', mo, 'preserving', add=-2, color='blue', linestyle='-.', label='preserving' + ref_str + '-2')
    plt.legend(loc=1)

    fig_file_name = filename + '_' + str(len(fig_list)) + ".png"
    fig_files.append(fig_file_name)
    plt.savefig(fig_file_name, format="png")

    return fig_list, fig_files


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
            self.eye_voltage = None
            self.unit = None  # text title
            self.eye_voltage_thr = None
            self.eye_cl = None
            self.conf = None
            self.max_nod_f = None
            self.max_nod_p = None
        else:
            self.i = 0
            self.cTime = np.array(data.cTime)
            self.time = np.array(data.cTime)
            self.eye_voltage = np.array(data.eye_voltage)
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
                    while self.zero_end < len(self.eye_voltage) and abs(self.eye_voltage[self.zero_end]) < zero_thr:
                        self.zero_end += 1
                    self.zero_end -= 1  # backup one
                    if self.zero_end == len(self.eye_voltage) - 1:
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
            self.eye_voltage = np.array(data.eye_voltage[:i_end])
            self.eye_voltage_thr = np.array(data.eye_voltage_thr[:i_end])
            self.eye_cl = np.array(data.eye_cl[:i_end])
            self.conf = np.array(data.conf[:i_end])
            self.max_nod_f = np.array(data.max_nod_f[:i_end])
            self.max_nod_p = np.array(data.max_nod_p[:i_end])
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
        s += "{},".format(self.hm[self.i])
        s += "{:13.3f},".format(self.cTime[self.i])
        s += "{:8.3f},".format(self.ib[self.i])
        s += "{:7.2f},".format(self.vsat[self.i])
        s += "{:5.2f},".format(self.dv_dyn[self.i])
        s += "{:5.2f},".format(self.voc_stat[self.i])
        s += "{:5.2f},".format(self.voc_ekf[self.i])
        s += "{:10.6f},".format(self.y_ekf[self.i])
        s += "{:7.3f},".format(self.soc_s[self.i])
        s += "{:5.3f},".format(self.soc_ekf[self.i])
        s += "{:5.3f},".format(self.soc[self.i])
        return s

    def mod(self):
        return self.mod_data[self.zero_end]


class SavedDataSim:
    def __init__(self, time_ref, data=None, time_end=None):
        if data is None:
            self.i = 0
            self.time = None
            self.time_min = None
            self.time_day = None
            self.unit = None  # text title
            self.cTime = None  # Control time, s
            self.chm_s = None
            self.qcrs_s = None  # Unit capacity rated scaled, Coulombs
            self.bms_off_s = None
            self.nS_s = None
            self.Tb_s = None
            self.Tbl_s = None
            self.vsat_s = None
            self.voc_s = None
            self.voc_stat_s = None
            self.dv_dyn_s = None
            self.dv_hys_s = None
            self.vb_s = None
            self.ib_in_s = None
            self.ib_charge_s = None
            self.ioc_s = None
            self.ib_s = None
            self.sat_s = None
            self.dq_s = None
            self.soc_s = None
            self.reset_s = None
        else:
            self.i = 0
            self.cTime = np.array(data.c_time)
            self.time = np.array(data.c_time) - time_ref
            # Truncate
            if time_end is None:
                i_end = len(self.time)
            else:
                i_end = np.where(self.time <= time_end)[0][-1] + 1
            self.cTime = self.cTime[:i_end]
            self.time = self.time[:i_end]
            self.time_min = self.time / 60.
            self.time_day = self.time / 3600. / 24.

            self.chm_s = data.chm_s[:i_end]
            if hasattr(data, 'qcrs_s'):
                self.qcrs_s = data.qcrs_s[:i_end]
            self.bms_off_s = data.bmso_s[:i_end]
            if hasattr(data, 'nS_s'):
                self.nS_s = np.array(data.nS_s[:i_end])
            else:
                self.nS_s = np.array(data.bmso_s[:i_end]) * 0 + 1
            self.Tb_s = data.Tb_s[:i_end]
            self.Tbl_s = data.Tbl_s[:i_end]
            self.vb_s = data.vb_s[:i_end]
            self.vsat_s = data.vsat_s[:i_end]
            self.voc_stat_s = data.voc_stat_s[:i_end]
            self.dv_dyn_s = data.dv_dyn_s[:i_end]
            self.voc_s = self.vb_s - self.dv_dyn_s
            self.dv_hys_s = self.voc_s - self.voc_stat_s
            self.ib_s = data.ib_s[:i_end]
            self.ib_in_s = data.ib_in_s[:i_end]
            self.ib_charge_s = data.ib_charge_s[:i_end]
            self.ioc_s = data.ioc_s[:i_end]
            self.sat_s = data.sat_s[:i_end]
            self.dq_s = data.dq_s[:i_end]
            self.soc_s = data.soc_s[:i_end]
            self.reset_s = data.reset_s[:i_end]

    def __str__(self):
        s = "{},".format(self.unit[self.i])
        s += "{:13.3f},".format(self.cTime[self.i])
        s += "{:5.2f},".format(self.Tb_s[self.i])
        s += "{:5.2f},".format(self.Tbl_s[self.i])
        s += "{:5.2f},".format(self.Tbl_s[self.i])
        s += "{:8.3f},".format(self.vsat_s[self.i])
        s += "{:5.2f},".format(self.voc_stat_s[self.i])
        s += "{:5.2f},".format(self.dv_dyn_s[self.i])
        s += "{:5.2f},".format(self.vb_s[self.i])
        s += "{:8.3f},".format(self.ib_s[self.i])
        s += "{:7.3f},".format(self.sat_s[self.i])
        # s += "{:5.3f},".format(self.ddq_s[self.i])
        s += "{:5.3f},".format(self.dq_s[self.i])
        # s += "{:5.3f},".format(self.qcap_s[self.i])
        s += "{:7.3f},".format(self.soc_s[self.i])
        s += "{:d},".format(self.reset_s[self.i])
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
