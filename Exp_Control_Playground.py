#!/usr/bin/env python3

import time
import rospy
import threading
import os
import numpy as np
import rospy
import rospkg
import sys

from sar_env_exp import SAR_Exp_Interface
from sar_general.Scripts.Control_Playground import cmd_send


if __name__ == '__main__':

    ## INIT GAZEBO ENVIRONMENT
    env = SAR_Exp_Interface()

    ## INITIALIZE CRAZYFLIE
    env.safeMode(True)
    env.cf.setParam("kalman/resetEstimation", 1)
    time.sleep(0.2)
    env.SendCmd('Ctrl_Reset')

    ## INITIALIALIZE LOGGING DATA
    trial_num = 24
    logName = f"Control_Playground--trial_{int(trial_num):02d}.csv"
    env.createCSV(logName)


    ## INIT COMMANDER THREAD
    cmd_thread = threading.Thread(target=cmd_send,args=(env,logName))
    cmd_thread.start()   
    # env.cf.setParam("GTC_Params/SafeModeFlag_p", 0)


    rospy.spin()
