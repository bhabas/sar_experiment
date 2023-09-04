
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os


## ADD SAR_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_logging_exp'))
sys.path.insert(0,BASE_PATH)


from sar_logging.Log_Data_Parser import DataFile
# os.system("clear")

dataPath = f"{BASE_PATH}/sar_logging_exp/local_logs/"
fileName = "Izz_Log_1.csv"
trial = DataFile(dataPath,fileName,dataType='EXP')

k_ep = 0
k_run = 0

trial.plot_state(k_ep,k_run,['wz'])
# trial.plot_convg(saveFig=True)

