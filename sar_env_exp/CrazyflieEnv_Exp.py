
from crazyflie_env.CrazyflieEnv_Base import CrazyflieEnv_Base


## SYSTEM IMPORTS
import numpy as np


def is_number(in_value):
    try:
        float(in_value)
        return True
    except ValueError:
        return False

class CrazyflieEnv_Exp(CrazyflieEnv_Base):
    def __init__(self):
        print("[STARTING] CrazyflieEnv is starting...")
        # CrazyflieEnv_Base.__init__(self)
        # rospy.init_node("Crazyflie_Env_Exp_Node")
        # os.system("roslaunch crazyflie_launch_exp params.launch")


if __name__ == "__main__":
    a = CrazyflieEnv_Exp()
        
        
    