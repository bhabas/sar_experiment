
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
        print("[STARTING] SAR_Exp is starting...")
        SAR_Base_Interface.__init__(self,Experiment_Setup=True)

        ## CRAZYSWARM INITIALIZATION
        cf_yaml = f"{crazyswarm_path}/launch/crazyflies.yaml"
        self.cf_swarm = Crazyswarm(crazyflies_yaml=cf_yaml)
        self.cf = self.cf_swarm.allcfs.crazyflies[0]
        self.timeHelper = self.cf_swarm.timeHelper


        ## SAR PARAMETERS
        self.Done = False
        self.setParams()
        self.Log_Dir = f"/home/{self.Username}/catkin_ws/src/sar_experiment/sar_logging_exp/local_logs"

    def setParams(self):

        os.system("roslaunch sar_launch_exp Load_Params.launch")
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


    def ArmQuad(self,status):
        
        if status == True:
            self.cf.setParam("System_Params/Armed",1)
        elif status == False:
            self.cf.setParam("System_Params/Armed",0)

    def handle_Load_Params(self):

        print("Reset ROS Parameters\n")
        os.system("roslaunch sar_launch_exp Load_Params.launch")
        self.sendCmd("Load_Params")
        self.setParams()


if __name__ == "__main__":
    env = SAR_Exp_Interface()
    rospy.spin()
    # env.SendCmd('Stop')
        
        
    