
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import pandas as pd
import os

## ADD SAR_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = "/home/bhabas/Desktop/sar_experiment"
sys.path.insert(0,BASE_PATH)



## LOAD DATA FILE
dataPath = f"{BASE_PATH}/sar_projects_exp/System_Identification/Inertia_Estimation/Logs/Crazyflie/"
fileName = "Izz_2.csv"

df = pd.read_csv(dataPath + fileName)

## GRAB DESIRED DATA
times = df["gyro.x_x"].to_numpy().flatten()*1e-3
wx = df["gyro.x_y"].to_numpy().flatten()
wy = df["gyro.y_y"].to_numpy().flatten()
wz = df["gyro.z_y"].to_numpy().flatten()

## SYSTEM PARAMETERS
L = 42.6e-2   # [m]
D = 9.3e-2   # [m]
m = 0.037   # [kg]
g = 9.81    # [m/s^2]
data = wz


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
    plt.show(block=True)




## FIND PEAKS. YOU CAN ADJUST DISTANCE PARAMETER AS NEEDED
peaks, _ = find_peaks(data, distance=20)

## PLOT DATA WITH PEAKS
plot_data_with_peaks(times, data, peaks)


## CALC PERIOD DATAI
time_diffs = get_time_differences(times, peaks)
T_avg = np.average(time_diffs)
T_std = np.std(time_diffs)

## CALCULATE INERTIA VALUE
Inertia = m*g*(D*T_avg)**2/(L*(4*np.pi)**2)

print("Time differences between peaks [s]:", time_diffs)
print(f"T_avg: {T_avg:.3f} \t T_std: {T_std:.3f}")
print(f"Inertia: {Inertia:.3e} [kg*m^2]")





