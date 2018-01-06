from utils.config import MAX_BOT_OMEGA,MIN_BOT_OMEGA,ROTATION_FACTOR
from utils.math_functions import *

def Get_Omega(kub_id, totalAngle, homePos):

    # print "Entered in Get Omega",radian_2_deg(totalAngle)
    # if abs(totalAngle) < ROTATION_FACTOR*2:
    #     # print totalAngle,ROTATION_FACTOR
    #     return 0.0

    MAX_w = (MAX_BOT_OMEGA+MIN_BOT_OMEGA)/1.2
    # theta_left = float(homePos[kub_id].theta-totalAngle)
    theta_lft = normalize_angle(normalize_angle(homePos[kub_id].theta)-totalAngle)*-1.0
    vw = (theta_lft/2*math.pi)*MAX_w
    # print "totalAngle",radian_2_deg(totalAngle)
    # print "theta_left ",radian_2_deg(theta_lft),theta_lft
    # print "homePos theta ",radian_2_deg(normalize_angle(homePos[kub_id].theta))
    # print "omega ",vw
    if abs(vw)<1*MIN_BOT_OMEGA:
        vw = 1*MIN_BOT_OMEGA*(1 if vw>0 else -1)

    if abs(theta_lft)<ROTATION_FACTOR/2:
        return 0.0

    # print "Omega return",vw
    return vw


    