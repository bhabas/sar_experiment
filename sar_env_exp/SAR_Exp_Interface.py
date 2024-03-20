
from sar_env.SAR_Base_Interface import SAR_Base_Interface


## SYSTEM IMPORTS
import numpy as np
import rospy
import time
import sys
import rospkg
import os
import yaml

## LOCAL IMPORTS
crazyswarm_path = rospkg.RosPack().get_path('crazyswarm')
sys.path.insert(1, crazyswarm_path)
from scripts.pycrazyswarm import Crazyswarm


np.set_printoptions(suppress=True)



class SAR_Exp_Interface(SAR_Base_Interface):
    def __init__(self):
        print("[STARTING] SAR_Exp is starting...")
        SAR_Base_Interface.__init__(self,Experiment_Setup=True)
        self.EXP_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env_exp'))
        self.loadBaseParams()
        self.loadExpParams()

        ## CRAZYSWARM INITIALIZATION
        cf_yaml = f"{crazyswarm_path}/launch/crazyflies.yaml"
        self.cf_swarm = Crazyswarm(crazyflies_yaml=cf_yaml)
        self.cf = self.cf_swarm.allcfs.crazyflies[0]
        self.timeHelper = self.cf_swarm.timeHelper


        ## SAR PARAMETERS
        self.Done = False
        self.setParams()
        self.Log_Dir =  f"{self.EXP_PATH}/sar_logging_exp/local_logs"

    def setParams(self):

        self.cf.setParam("stabilizer/controller", 5) # Set firmware controller to GTC

        ## SET SAR TYPE
        if self.SAR_Type == "Crazyflie":
            self.cf.setParam("P1/SAR_Type",1)
        elif self.SAR_Type == "Impulse_Micro":
            self.cf.setParam("P1/SAR_Type",2)
        elif self.SAR_Type == "Source_One_V5":
            self.cf.setParam("P1/SAR_Type",3)
        else:
            self.cf.setParam("P1/SAR_Type",0)


        ## SET EXP SETTINGS
        if rospy.get_param("/SAR_SETTINGS/Policy_Type") == "PARAM_OPTIM":
            self.cf.setParam("P1/PolicyType",0)

        elif rospy.get_param("/SAR_SETTINGS/Policy_Type") == "DEEP_RL_SB3":
            self.cf.setParam("P1/PolicyType",1)

        elif rospy.get_param("/SAR_SETTINGS/Policy_Type") == "DEEP_RL_ONBOARD":
            self.cf.setParam("P1/PolicyType",2)

        ## SET CONTROLLER INERTIA VALUES
        InertiaParamDict = {
            "P1/Mass":               self.Ref_Mass,
            "P1/I_xx":                self.Ref_Ixx, 
            "P1/I_yy":                self.Ref_Iyy,
            "P1/Izz":                self.Ref_Izz,
            "P1/L_eff":              self.L_eff,
        }
        self.cf.setParams(InertiaParamDict)

        SystemParamDict = {
            "P1/Prop_14_x":      self.Prop_Front[0], 
            "P1/Prop_14_y":      self.Prop_Front[1], 
            "P1/Prop_23_x":      self.Prop_Rear[0],
            "P1/Prop_23_y":      self.Prop_Rear[1],
            "P1/C_tf":           self.C_tf,
            "P1/Tust_max":     self.Thrust_max,
            "P1/Fwd_Reach":      self.Forward_Reach,

        }
        self.cf.setParams(SystemParamDict)
        
        ## SET CONTROLLER GAIN VALUES
        temp_str = f"/SAR_Type/{self.SAR_Type}/CtrlGains"
        GainsDict = {
            "P2/P_kp_xy":      rospy.get_param(f"{temp_str}/P_kp_xy"),
            "P2/P_kd_xy":      rospy.get_param(f"{temp_str}/P_kd_xy"), 
            "P2/P_ki_xy":      rospy.get_param(f"{temp_str}/P_ki_xy"),
            "P2/i_range_xy":   rospy.get_param(f"{temp_str}/i_range_xy"),

            "P2/P_kp_z":       rospy.get_param(f"{temp_str}/P_kp_z"),        
            "P2/P_Kd_z":       rospy.get_param(f"{temp_str}/P_kd_z"),
            "P2/P_ki_z":       rospy.get_param(f"{temp_str}/P_ki_z"),
            "P2/i_range_z":    rospy.get_param(f"{temp_str}/i_range_z"),
        }
        self.cf.setParams(GainsDict)

    
        GainsDict2 = {
            "P2/R_kp_xy":      rospy.get_param(f"{temp_str}/R_kp_xy"),
            "P2/R_kd_xy":      rospy.get_param(f"{temp_str}/R_kd_xy"),
            "P2/R_ki_xy":      rospy.get_param(f"{temp_str}/R_ki_xy"),
            "P2/i_range_xy":   rospy.get_param(f"{temp_str}/i_range_R_xy"),


            "P2/R_kpz":       rospy.get_param(f"{temp_str}/R_kp_z"),
            "P2/R_kdz":       rospy.get_param(f"{temp_str}/R_kd_z"),
            "P2/R_ki_z":       rospy.get_param(f"{temp_str}/R_ki_z"),
            "P2/i_range_R_z":  rospy.get_param(f"{temp_str}/i_range_R_z"),
        }
        self.cf.setParams(GainsDict2)
        
        ## SET SYSTEM GEOMETRY PARAMETERS INERTIA VALUES




        self.sendCmd("Plane_Pose",cmd_vals=[self.Plane_Pos_x_init,self.Plane_Pos_y_init,self.Plane_Pos_z_init],cmd_flag=self.Plane_Angle_deg_init)

        self.sendCmd("Load_Params")
        self.sendCmd("Ctrl_Reset")

    def loadExpParams(self):

        ## LOAD BASE PARAMETERS
        param_path = f"{self.EXP_PATH}/sar_config_exp/Exp_Settings.yaml"
                    
        with open(param_path, 'r') as file:
            loaded_parameters = yaml.safe_load(file)

        # Load parameters into the ROS Parameter Server
        for param_name, param_value in loaded_parameters.items():
            rospy.set_param(param_name, param_value)


        ## SAR PARAMETERS
        self.SAR_Type = rospy.get_param('/SAR_SETTINGS/SAR_Type')
        self.SAR_Config = rospy.get_param('/SAR_SETTINGS/SAR_Config')
        self.Policy_Type = rospy.get_param('/SAR_SETTINGS/Policy_Type')

        ## INERTIAL PARAMETERS
        self.Ref_Mass = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Ref_Mass")
        self.Ref_Ixx = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Ref_Ixx")
        self.Ref_Iyy = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Ref_Iyy")
        self.Ref_Izz = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Ref_Izz")

        self.Base_Mass = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Base_Mass")
        self.Base_Ixx = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Base_Ixx")
        self.Base_Iyy = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Base_Iyy")
        self.Base_Izz = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Base_Izz")

        ## GEOMETRIC PARAMETERS
        self.Forward_Reach = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Forward_Reach")
        self.Leg_Length = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Leg_Length")
        self.Leg_Angle = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Leg_Angle")
        self.Prop_Front = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Prop_Front")
        self.Prop_Rear = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Prop_Rear")

        ## EFFECTIVE-GEOEMTRIC PARAMETERS
        self.L_eff = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/L_eff")
        self.Gamma_eff = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Gamma_eff")
        self.Lx_eff = self.L_eff*np.sin(np.radians(self.Gamma_eff))
        self.Lz_eff = self.L_eff*np.cos(np.radians(self.Gamma_eff))
        self.Collision_Radius = max(self.L_eff,self.Forward_Reach)

        ## SYSTEM AND FLIGHT PARAMETERS
        self.TrajAcc_Max = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/TrajAcc_Max")
        self.TrajJerk_Max = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/TrajJerk_Max")
        self.Tau_up = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Tau_up")
        self.Tau_down = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Tau_down")
        self.Thrust_max = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Thrust_max")
        self.C_tf = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/C_tf")
        self.Ang_Acc_max = (9.81*self.Thrust_max*1e-3*self.Prop_Front[0])*2/self.Ref_Iyy
        self.setAngAcc_range([-self.Ang_Acc_max, self.Ang_Acc_max])
        
        self.Beta_Min_deg = -(self.Gamma_eff + np.degrees(np.arctan2(self.Forward_Reach-self.Lx_eff,self.Lz_eff)))
        self.Phi_P_B_impact_Min_deg = -self.Beta_Min_deg - self.Gamma_eff + 90

        ## CAM PARAMETERS
        self.Cam_Config = rospy.get_param('/CAM_SETTINGS/Cam_Config')
        self.Cam_Active = rospy.get_param('/CAM_SETTINGS/Cam_Active')
        

        ## PLANE PARAMETERS
        self.Plane_Type = rospy.get_param('/PLANE_SETTINGS/Plane_Type')
        self.Plane_Config = rospy.get_param('/PLANE_SETTINGS/Plane_Config')
        self.Plane_Pos_x_init = rospy.get_param('/PLANE_SETTINGS/Pos_X_init')
        self.Plane_Pos_y_init = rospy.get_param('/PLANE_SETTINGS/Pos_Y_init')
        self.Plane_Pos_z_init = rospy.get_param('/PLANE_SETTINGS/Pos_Z_init')
        self.Plane_Angle_deg_init = rospy.get_param('/PLANE_SETTINGS/Plane_Angle_init')

    def handle_Load_Params(self):

        print("Reset ROS Parameters\n")
        self.loadBaseParams()
        self.loadExpParams()
        self.sendCmd("Load_Params")
        self.setParams()


if __name__ == "__main__":
    env = SAR_Exp_Interface()
    rospy.spin()
    # env.SendCmd('Stop')
        
        
    