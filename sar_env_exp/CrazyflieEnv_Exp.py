
from crazyflie_env.CrazyflieEnv_Base import CrazyflieEnv_Base


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



class CrazyflieEnv_Exp(CrazyflieEnv_Base):
    def __init__(self):
        print("[STARTING] CrazyflieEnv is starting...")
        CrazyflieEnv_Base.__init__(self)
        # rospy.init_node("Crazyflie_Env_Exp_Node",anonymous=True)
        os.system("roslaunch sar_launch_exp params.launch")

        ## CRAZYSWARM INITIALIZATION
        cf_yaml = f"{crazyswarm_path}/launch/crazyflies.yaml"
        self.cf_swarm = Crazyswarm(crazyflies_yaml=cf_yaml)
        self.cf = self.cf_swarm.allcfs.crazyflies[0]
        self.timeHelper = self.cf_swarm.timeHelper


        ## CRAZYFLIE PARAMETERS
        self.CF_Type = rospy.get_param('/QUAD_SETTINGS/CF_Type')
        self.CF_Config = rospy.get_param('/QUAD_SETTINGS/CF_Config')
        self.modelInitials = rospy.get_param(f"/CF_Type/{self.CF_Type}/Config/{self.CF_Config}/Initials")
        self.modelName = f"crazyflie_{self.CF_Config}"
        self.setParams()

        self.logDir = "/home/bhabas/catkin_ws/src/sar_experiment/"

        # ## PLANE PARAMETERS
        # self.Plane_Model = rospy.get_param('/PLANE_SETTINGS/Plane_Model')
        # self.Plane_Config = rospy.get_param('/PLANE_SETTINGS/Plane_Config')
        # self.Plane_Angle = rospy.get_param(f'/Plane_Config/{self.Plane_Config}/Plane_Angle')
        # self.Plane_Angle_rad = np.deg2rad(self.Plane_Angle)
        # self.Plane_Pos = [
        #     rospy.get_param(f'/Plane_Config/{self.Plane_Config}/Pos_X'),
        #     rospy.get_param(f'/Plane_Config/{self.Plane_Config}/Pos_Y'),
        #     rospy.get_param(f'/Plane_Config/{self.Plane_Config}/Pos_Z'),
        # ]

    def setParams(self):

        os.system("roslaunch sar_launch_exp params.launch")
        self.cf.setParam("stabilizer/controller", 5) # Set firmware controller to GTC


        ## SET EXP SETTINGS
        if rospy.get_param("/QUAD_SETTINGS/Policy_Type") == "PARAM_OPTIM":
            self.cf.setParam("GTC_Params/PolicyType_p",0)

        elif rospy.get_param("/QUAD_SETTINGS/Policy_Type") == "SVL_POLICY":
            self.cf.setParam("GTC_Params/PolicyType_p",1)

        elif rospy.get_param("/QUAD_SETTINGS/Policy_Type") == "DEEP_RL":
            self.cf.setParam("GTC_Params/PolicyType_p",2)

        
        ## SET CONTROLLER GAIN VALUES
        temp_str = f"/CF_Type/{self.CF_Type}/CtrlGains"
        GainsDict = {
            "GTC_Params/P_kp_xy_p":     rospy.get_param(f"{temp_str}/P_kp_xy"),
            "GTC_Params/P_kd_xy_p":     rospy.get_param(f"{temp_str}/P_kd_xy"), 
            "GTC_Params/P_ki_xy_p":     rospy.get_param(f"{temp_str}/P_ki_xy"),
            # "GTC_Params/i_range_xy_p":  rospy.get_param(f"{temp_str}/i_range_xy"),

            "GTC_Params/P_kp_z_p":      rospy.get_param(f"{temp_str}/P_kp_z"),        
            "GTC_Params/P_kd_z_p":      rospy.get_param(f"{temp_str}/P_kd_z"),
            "GTC_Params/P_ki_z_p":      rospy.get_param(f"{temp_str}/P_ki_z"),
            # "GTC_Params/i_range_z_p":   rospy.get_param(f"{temp_str}/i_range_z"),
        }
        self.cf.setParams(GainsDict)

        GainsDict = {
            "GTC_Params/R_kp_xy_p":     rospy.get_param(f"{temp_str}/R_kp_xy"),
            "GTC_Params/R_kd_xy_p":     rospy.get_param(f"{temp_str}/R_kd_xy"),
            "GTC_Params/R_ki_xy_p":     rospy.get_param(f"{temp_str}/R_ki_xy"),
            # "GTC_Params/i_range_xy_p":  rospy.get_param(f"{temp_str}/i_range_R_xy"),


            "GTC_Params/R_kp_z_p":      rospy.get_param(f"{temp_str}/R_kp_z"),
            "GTC_Params/R_kd_z_p":      rospy.get_param(f"{temp_str}/R_kd_z"),
            "GTC_Params/R_ki_z_p":      rospy.get_param(f"{temp_str}/R_ki_z"),
            # "GTC_Params/i_range_R_z_p": rospy.get_param(f"{temp_str}/i_range_R_z"),
        }
        self.cf.setParams(GainsDict)
        
        ## SET SYSTEM GEOMETRY PARAMETERS INERTIA VALUES
        temp_str = f"/CF_Type/{self.CF_Type}/System_Params"
        SystemParamDict = {
            "GTC_Params/Prop_Dist_p":     rospy.get_param(f"{temp_str}/Prop_Dist"), 
            "GTC_Params/C_tf_p":     rospy.get_param(f"{temp_str}/C_tf"),
            "GTC_Params/f_max_p":     rospy.get_param(f"{temp_str}/f_max"),
        }
        self.cf.setParams(SystemParamDict)


        ## SET CONTROLLER INERTIA VALUES
        temp_str = f"/CF_Type/{self.CF_Type}/Config/{self.CF_Config}"
        InertiaParamDict = {
            "GTC_Params/CF_mass_p": rospy.get_param(f"{temp_str}/Mass"),
            "GTC_Params/Ixx_p":     rospy.get_param(f"{temp_str}/Ixx"), 
            "GTC_Params/Iyy_p":     rospy.get_param(f"{temp_str}/Iyy"),
            "GTC_Params/Izz_p":     rospy.get_param(f"{temp_str}/Izz"),
        }
        self.cf.setParams(InertiaParamDict)

        self.SendCmd("Load_Params")

    def safeMode(self,status):
        
        if status == True:
            self.cf.setParam("GTC_Params/SafeModeFlag_p",1)
        elif status == False:
            self.cf.setParam("GTC_Params/SafeModeFlag_p",0)


if __name__ == "__main__":
    env = CrazyflieEnv_Exp()
    env.SendCmd('Stop')
        
        
    