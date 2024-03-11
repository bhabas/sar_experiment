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

def cmd_send(env,logName):

    while True:
        # Converts input number into action name

        command_handlers = {
            'Ctrl_Reset': env.handle_Ctrl_Reset,
            'Pos': env.handle_Pos_Cmd,
            'Vel': env.handle_Vel_Cmd,
            'Stop': env.handle_Stop,
            'Ang_Accel': env.handle_Ang_Accel,
            'Policy': env.handle_Policy,
            'Plane_Pose': env.handle_Plane_Pose,
            'P2P_traj': env.handle_P2P_traj,
            'Global_Vel_traj': env.handle_Global_Vel_traj,
            'Rel_Vel_traj': env.handle_Rel_Vel_traj,
            'Impact_traj': env.handle_Impact_traj,
            'Tumble_Detect': env.handle_Tumble_Detect,
            'Arm_Quad': env.handle_Arm_Quad,
            'Load_Params': env.handle_Load_Params,
            'Start_Logging': env.handle_Start_Logging,
            'Cap_Logging': env.handle_Cap_Logging,
            'Thrust_CMD': env.handle_Thrust_CMD,
            'Motor_CMD': env.handle_Motor_CMD,
        }
        
        try:
            print("========== Command Types ==========")
            print("0: Ctrl_Reset  7: Ang_Accel    10: P2P_traj          20: Tumble_Detect    24: Arm_Quad    90: GZ_Pose_Reset")
            print("1: Pos         8: Policy       11: Global_Vel_traj   21: Load_Params      30: Thrust_CMD  91: GZ_Const_Vel_Traj")
            print("2: Vel         9: Plane_Pose   12: Rel_Vel_traj      22: Start_Logging    31: Motor_CMD   92: GZ_StickyPads")
            print("5: Stop                        13: Impact_traj       23: Cap_Logging")


            val = env.userInput("\nCmd: ",int)
            cmd_action = env.inv_cmd_dict[val]
            if cmd_action in command_handlers:
                command_handlers[cmd_action]()
            else:
                print("Invalid Command: Try again")
        
        except Exception as e:
            print(f"Error: {e}")
            print('\033[93m' + "INVALID INPUT: Try again" + '\x1b[ 0m')
            continue



if __name__ == '__main__':

    ## INIT GAZEBO ENVIRONMENT
    env = SAR_Exp_Interface()

    ## INITIALIZE CRAZYFLIE
    env.cf.setParam("kalman/resetEstimation", 1)
    time.sleep(0.2)


    ## INITIALIALIZE LOGGING DATA
    logName = f"Test_Log.csv"
    # env.createCSV(logName)


    ## INIT COMMANDER THREAD
    cmd_thread = threading.Thread(target=cmd_send,args=(env,logName))
    cmd_thread.start()   


    rospy.spin()
  