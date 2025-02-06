# MonSim:  Monitor and Simulator replication of Particle Photon Application
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

""" Python model of what's installed on the Particle Photon.  Includes
a monitor object (MON) and a simulation object (SIM).   The monitor is
the EKF and Coulomb Counter.   The SIM is a battery model, that also has a
Coulomb Counter built in."""
import sys

from Sensors import Sensors
from unite_pictures import unite_pictures_into_pdf, cleanup_fig_files, precleanup_fig_files
import matplotlib.pyplot as plt
from datetime import datetime
from load_data import load_data
from PlotGP import gp_plot
import easygui
from PlotKiller import show_killer
import tkinter.messagebox
from local_paths import version_from_data_file, local_paths
import os

if sys.platform == 'darwin':
    import matplotlib
    matplotlib.use('tkagg')
plt.rcParams['axes.grid'] = True


def compare_run_sim(data_file=None, unit_key=None, time_end_in=None, data_only=False, init_time_in=None):
    print(f"\ncompare_run_sim:\n{data_file=}\n{unit_key=}\n{time_end_in=}\n{data_only=}\n")

    date_time = datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
    date_ = datetime.now().strftime("%y%m%d")
    
    # Transient  inputs
    zero_zero_in = False
    legacy_in = False

    # detect running interactively
    # this is written to run in pwd of call
    if data_file is None:
        path_to_data = easygui.fileopenbox(msg="choose your data file to plot")
        data_file = easygui.filesavebox(msg="pick new file name, cancel to keep", title="get new file name")
        if data_file is None:
            data_file = path_to_data
        else:
            os.rename(path_to_data, data_file)
        unit_key = easygui.enterbox(msg="enter pro0p, pro1a, soc0p, soc1a", title="get unit_key", default="pro1a")

    # Folder operations
    version = version_from_data_file(data_file)
    _, save_pdf_path, _ = local_paths(version)

    # # Load mon v4 (old)
    mon_old, sim_old, f, data_file_clean, temp_flt_file_clean, _ = \
        load_data(data_file, 1, unit_key, zero_zero_in, time_end_in)

    # How to initialize
    init_time = None
    if mon_old is not None:
        if init_time_in is not None:
            init_time = init_time_in
        elif mon_old.time[0] == 0.:  # no initialization flat detected at beginning of recording
            init_time = 1.
        else:
            if init_time_in:
                init_time = init_time_in
            else:
                init_time = -4.
    else:
        tkinter.messagebox.showwarning(message="CompareRunSim:  Data missing.  See monitor window for info.")
        return None, None, None, None, None, None

    # New run
    patch = Sensors(mon_old)
    mon_ver = patch.calculate(init_time=init_time)

    # Plots
    if data_only is False:
        fig_list = []
        fig_files = []
        dir_root_test, data_root_test = os.path.split(data_file_clean)
        data_root_test = data_root_test.replace('.csv', '')
        filename = data_root_test
        plot_title = dir_root_test + '/' + data_root_test + '   ' + date_time
        fig_list, fig_files = gp_plot(mon_old, mon_ver, filename, fig_files,
                                      plot_title=plot_title, fig_list=fig_list, ref_str='',
                                      test_str='_ver')

        # Copies
        precleanup_fig_files(output_pdf_name=filename, path_to_pdfs=save_pdf_path)
        unite_pictures_into_pdf(outputPdfName=filename+'_'+date_time+'.pdf', save_pdf_path=save_pdf_path)
        cleanup_fig_files(fig_files)
        plt.show(block=False)
        string = 'plots ' + str(fig_list[0].number) + ' - ' + str(fig_list[-1].number)
        show_killer(string, 'CompareRunSim', fig_list=fig_list)

    return data_file_clean, mon_old, sim_old, None, None, None


def main():
    if sys.platform == 'win32':
        # path_to_data = 'C:/Users/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/slip_slide_repeats_winks_sleeps.csv'
        # path_to_data = 'C:/Users/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/test_config.csv'
        # path_to_data = 'C:/Users/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/put_on_tilt_wink.csv'
        # path_to_data = 'C:/Users/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/full_rate_test.csv'
        # path_to_data = 'C:/Users/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/rest_test_kitty.csv'
        # path_to_data = 'C:/Users/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/rest_test_kitty2.csv'
        path_to_data = 'C:/Users/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/rpy_test.csv'
        # path_to_data = 'C:/Users/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/rpy_test_noreset.csv'
        # path_to_data = 'C:/Users/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/frequent_false_positive.csv'
        # path_to_data = 'C:/Users/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/road_test.csv'
        # path_to_data = 'C:/Users/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/home_test.csv'
        # path_to_data = 'C:/Users/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/road_test2.csv'
    else:
        path_to_data = '/home/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/road_test.csv'
    # unit_key = 'v20250113_pro0n33iot_Rapid'
    # unit_key = 'v20250113_wearDn33iot_Rapid'
    # unit_key = 'v20250113_wearDn33iot_Rapid'
    # unit_key = 'v20250113_wearDn33iot_Rapid'
    # unit_key = 'v20250113_wearDn33iot_Rapid'
    # unit_key = 'v20250113_wearDn33iot_Rapid'
    # unit_key = 'v20250113_wearDn33iot_Rapid'
    unit_key = 'v20250113_wearDn33iot_Rapid'
    time_end_in = None
    data_only = False
    init_time_in = 0.

    compare_run_sim(data_file=path_to_data, unit_key=unit_key, data_only=data_only, time_end_in=time_end_in,
                    init_time_in=init_time_in)


# import cProfile
# if __name__ == '__main__':
#     cProfile.run('main()')
#


if __name__ == '__main__':
    main()
