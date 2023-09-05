
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import pandas as pd
import os

## ADD SAR_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_logging_exp'))
sys.path.insert(0,BASE_PATH)



## LOAD DATA FILE
from sar_logging.Log_Data_Parser import DataFile
dataPath = f"{BASE_PATH}/sar_projects_exp/System_Identification/Inertia_Estimation/Logs/Source_One_V5/"
fileName = "Ixx_Log.csv"
trial = DataFile(dataPath,fileName,dataType='EXP')


## SYSTEM PARAMETERS
m = 590.0e-3    # [kg]
g = 9.81        # [m/s^2]
D = 22.0e-3     # [m]
L = 24.8e-2     # [m]


def get_time_differences(times, peaks):
    return np.diff(times[peaks])

def plot_data_with_peaks(times, data, peaks):
    plt.figure(figsize=(10, 6))
    plt.plot(times, data, label="Data")
    plt.plot(times[peaks], data[peaks], "x", label="Peaks")
    for i in range(len(peaks) - 1):
        plt.annotate(
            f"{times[peaks[i + 1]] - times[peaks[i]]:.2f}",
            ((times[peaks[i]] + times[peaks[i + 1]]) / 2, data[peaks[i]]),
            textcoords="offset points",
            xytext=(0, 10),
            ha='center'
        )
    plt.legend()
    plt.title("Data with Peaks Marked")
    plt.xlabel("Time")
    plt.ylabel("Value")
    plt.show()


Inertia_arr = []
for k_run in range(0,5):
    ## GRAB DESIRED DATA
    k_ep = 0
    data = trial.grab_stateData(k_ep,k_run,['wx']).flatten()
    times = trial.grab_stateData(k_ep,k_run,['t']).flatten()


    ## FIND PEAKS. YOU CAN ADJUST DISTANCE PARAMETER AS NEEDED
    peaks, _ = find_peaks(data, distance=60)

    ## PLOT DATA WITH PEAKS
    plot_data_with_peaks(times, data, peaks)


    ## CALC PERIOD DATA
    time_diffs = get_time_differences(times, peaks)
    T_avg = np.average(time_diffs)
    T_std = np.std(time_diffs)

    ## CALCULATE INERTIA VALUE
    Inertia = m*g*(D*T_avg)**2/(L*(4*np.pi)**2)
    Inertia_arr.append(Inertia)

    print(f"K_run: {k_run:02d}")
    print("Time differences between peaks [s]:", time_diffs)
    print(f"T_avg: {T_avg:.3f} \t T_std: {T_std:.3f}")
    print(f"Inertia: {Inertia:.3e} [kg*m^2]")

Inertia_avg = np.average(Inertia_arr)
print(f"\n==========================\n")
print(f"I_avg: {Inertia_avg:.3e} [kg*m^2]")



