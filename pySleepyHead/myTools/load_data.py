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
"""Utility to load data from csv files"""
import platform

import numpy as np
from DataOverModel import SavedData, write_clean_file


def find_sync(path_to_data):
    sync = []
    with open(path_to_data) as in_file:
        for line in in_file:
            if line.__contains__('SYNC'):
                sync.append(float(line.strip().split(',')[-1]))

    if not sync:
        sync = None
    else:
        sync = np.array(sync)
    return sync


def calculate_master_sync(ref, test):
    delta = np.maximum(ref, test)
    return delta


class SyncInfo:
    """Shift time arrays to synchronize two different data sets for CompareRunRun usage"""
    def __init__(self, sav_mon, sync=None):
        self.is_empty = False
        if sync is None or sav_mon is None:
            self.is_empty = True
            return
        self.time_mon = sav_mon.time
        self.sync_cTime = sync
        self.cTime = sav_mon.cTime
        self.cTime_0 = sav_mon.cTime[0]
        self.time = sav_mon.time
        self.int_mon = []
        self.length = len(sync)
        rel = []
        delta = []
        for i in np.arange(self.length):
            rel.append(self.sync_cTime[i] - sav_mon.cTime[0])
            if i == 0:
                delta.append(rel[0])
                self.int_mon.append([np.where(sav_mon.cTime <= sync[i])])
            else:
                delta.append(rel[i] - rel[i-1])
                self.int_mon.append([np.where((sav_mon.cTime <= sync[i]) & (sav_mon.cTime > sync[i-1]))])
        self.int_mon.append([np.where(sav_mon.cTime > sync[self.length-1])])
        self.rel_mon = np.array(rel)
        self.del_mon = np.array(delta)
        return

    def synchronize(self, sync_del):
        """Call this after building two class instances and calling calculate_master_sync to make sync_del"""
        # Init entire time array again.  First sync is always 0
        acc_shift = self.sync_cTime[0]
        self.time_mon = self.cTime.copy()

        # Subsequent sets based on difference to master del
        for i in np.arange(self.length+1):
            if 1 < i:
                acc_shift -= sync_del[i-1] - self.del_mon[i-1]
            self.time_mon[self.int_mon[i]] = (self.time_mon[self.int_mon[i]] - acc_shift).copy()

        return


# Load from files
def load_data(path_to_data, skip, unit_key, zero_zero_in, time_end_in, zero_thr_in=0.02):

    print(f"load_data: \n{path_to_data=}\n{skip=}\n{unit_key=}\n{zero_zero_in=}\n{time_end_in=}\n")

    hdr_key = "key_Rapid,"  # Find one instance of title
    hdr_key_sel = "Analog_s,"  # Find one instance of title
    unit_key_sel = "unit_sel"
    hdr_key_ekf = "unit_e,"  # Find one instance of title
    unit_key_ekf = "unit_ekf"
    hdr_key_sim = "unit_m,"  # Find one instance of title
    unit_key_sim = "unit_sim"

    sync = find_sync(path_to_data)

    data_file_clean = write_clean_file(path_to_data, type_='_mon', hdr_key=hdr_key, unit_key=unit_key, skip=skip)
    if data_file_clean is None:
        return None, None, None, None, None
    if data_file_clean is not None:
        mon_raw = np.genfromtxt(data_file_clean, delimiter=',', names=True, dtype=float).view(np.recarray)
    else:
        mon_raw = None
        print(f"load_data: returning mon=None")

    mon = SavedData(data=mon_raw, time_end=time_end_in, zero_zero=zero_zero_in)

    # Load sim _s v24 portion of real-time run (old)
    data_file_sim_clean = write_clean_file(path_to_data, type_='_sim', hdr_key=hdr_key_sim,
                                           unit_key=unit_key_sim, skip=skip)
    sim = None
    print(f"load_data: returning sim=None")

    # Calculate sync information
    sync_info = SyncInfo(sav_mon=mon, sync=sync)

    # Load fault
    temp_flt_file_clean = write_clean_file(path_to_data, type_='_flt', hdr_key='fltb',
                                           unit_key='unit_f', skip=skip, comment_str='---')

    return mon, sim, None, data_file_clean, temp_flt_file_clean, sync_info

def main():
    if platform.system() == 'Windows':
        path_to_data = 'C:/Users/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/Mahony_test.csv'
    else:
        path_to_data = '/home/daveg/Documents/GitHub/mySleepyHead/pySleepyHead/dataReduction/Mahony_test.csv'

    skip = 1
    unit_key = 'v20250113_wearDn33iot_Rapid,'
    zero_zero_in = False
    time_end_in = None
    load_data(path_to_data=path_to_data, skip=skip, unit_key=unit_key, zero_zero_in=zero_zero_in, time_end_in=time_end_in)


if __name__ == '__main__':
    main()
