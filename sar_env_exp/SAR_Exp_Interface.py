
from sar_env.SAR_Base_Interface import SAR_Base_Interface


## SYSTEM IMPORTS
import numpy as np
import rospy
import time
import getpass
import sys
import rospkg
import os

## LOCAL IMPORTS
crazyswarm_path = rospkg.RosPack().get_path('crazyswarm')
sys.path.insert(1, crazyswarm_path)
from scripts.pycrazyswarm import Crazyswarm


np.set_printoptions(suppress=True)



class SAR_Exp_Interface(SAR_Base_Interface):
    def __init__(self):
        print("[STARTING] CrazyflieEnv is starting...")
        SAR_Base_Interface.__init__(self,Exp_Flag=True)
        os.system("roslaunch sar_launch_exp params.launch")

        ## CRAZYSWARM INITIALIZATION
        cf_yaml = f"{crazyswarm_path}/launch/crazyflies.yaml"
        self.cf_swarm = Crazyswarm(crazyflies_yaml=cf_yaml)
        self.cf = self.cf_swarm.allcfs.crazyflies[0]
        self.timeHelper = self.cf_swarm.timeHelper


        ## SAR PARAMETERS
        self.SAR_Type = rospy.get_param('/SAR_SETTINGS/SAR_Type')
        self.SAR_Config = rospy.get_param('/SAR_SETTINGS/SAR_Config')
        self.modelInitials = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Initials")
        self.modelName = f"{self.SAR_Type}_{self.SAR_Config}"
        self.Done = False
        self.setParams()

        self.logDir = "/home/bhabas/catkin_ws/src/sar_experiment/"

    def setParams(self):

        os.system("roslaunch sar_launch_exp params.launch")
        self.cf.setParam("stabilizer/controller", 5) # Set firmware controller to GTC

        ## SET EXP SETTINGS
        if rospy.get_param("/SAR_SETTINGS/Cam_Active") == True:
            self.cf.setParam("CTRL_Params/isCamActive",1)
        else:
            self.cf.setParam("CTRL_Params/isCamActive",0)


        ## SET EXP SETTINGS
        if rospy.get_param("/SAR_SETTINGS/Policy_Type") == "PARAM_OPTIM":
            self.cf.setParam("CTRL_Params/PolicyType",0)

        elif rospy.get_param("/SAR_SETTINGS/Policy_Type") == "DEEP_RL_SB3":
            self.cf.setParam("CTRL_Params/PolicyType",1)

        elif rospy.get_param("/SAR_SETTINGS/Policy_Type") == "DEEP_RL_ONBOARD":
            self.cf.setParam("CTRL_Params/PolicyType",2)

        
        ## SET CONTROLLER GAIN VALUES
        temp_str = f"/SAR_Type/{self.SAR_Type}/CtrlGains"
        GainsDict = {
            "CTRL_Params/P_kp_xy":      rospy.get_param(f"{temp_str}/P_kp_xy"),
            "CTRL_Params/P_kd_xy":      rospy.get_param(f"{temp_str}/P_kd_xy"), 
            "CTRL_Params/P_ki_xy":      rospy.get_param(f"{temp_str}/P_ki_xy"),
            "CTRL_Params/i_range_xy":   rospy.get_param(f"{temp_str}/i_range_xy"),

            "CTRL_Params/P_kp_z":       rospy.get_param(f"{temp_str}/P_kp_z"),        
            "CTRL_Params/P_kd_z":       rospy.get_param(f"{temp_str}/P_kd_z"),
            "CTRL_Params/P_ki_z":       rospy.get_param(f"{temp_str}/P_ki_z"),
            "CTRL_Params/i_range_z":    rospy.get_param(f"{temp_str}/i_range_z"),
        }
        self.cf.setParams(GainsDict)

        GainsDict = {
            "CTRL_Params/R_kp_xy":      rospy.get_param(f"{temp_str}/R_kp_xy"),
            "CTRL_Params/R_kd_xy":      rospy.get_param(f"{temp_str}/R_kd_xy"),
            "CTRL_Params/R_ki_xy":      rospy.get_param(f"{temp_str}/R_ki_xy"),
            "CTRL_Params/i_range_xy":   rospy.get_param(f"{temp_str}/i_range_R_xy"),


            "CTRL_Params/R_kp_z":       rospy.get_param(f"{temp_str}/R_kp_z"),
            "CTRL_Params/R_kd_z":       rospy.get_param(f"{temp_str}/R_kd_z"),
            "CTRL_Params/R_ki_z":       rospy.get_param(f"{temp_str}/R_ki_z"),
            "CTRL_Params/i_range_R_z":  rospy.get_param(f"{temp_str}/i_range_R_z"),
        }
        self.cf.setParams(GainsDict)
        
        ## SET SYSTEM GEOMETRY PARAMETERS INERTIA VALUES
        temp_str = f"/SAR_Type/{self.SAR_Type}/System_Params"
        SystemParamDict = {
            "CTRL_Params/Prop_Dist":    rospy.get_param(f"{temp_str}/Prop_Dist"), 
            "CTRL_Params/C_tf":         rospy.get_param(f"{temp_str}/C_tf"),
            "CTRL_Params/f_max":        rospy.get_param(f"{temp_str}/f_max"),
        }
        self.cf.setParams(SystemParamDict)


        ## SET CONTROLLER INERTIA VALUES
        temp_str = f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}"
        InertiaParamDict = {
            "CTRL_Params/CF_mass": rospy.get_param(f"{temp_str}/Mass"),
            "CTRL_Params/Ixx":     rospy.get_param(f"{temp_str}/Ixx"), 
            "CTRL_Params/Iyy":     rospy.get_param(f"{temp_str}/Iyy"),
            "CTRL_Params/Izz":     rospy.get_param(f"{temp_str}/Izz"),
        }
        self.cf.setParams(InertiaParamDict)

        self.SendCmd("Load_Params")


    def safeMode(self,status):
        
        if status == True:
            self.cf.setParam("CTRL_Params/SafeMode",1)
        elif status == False:
            self.cf.setParam("CTRL_Params/SafeMode",0)


if __name__ == "__main__":
    env = SAR_Exp_Interface()
    rospy.spin()
    # env.SendCmd('Stop')
        
        
    