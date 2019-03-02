from utils.config import MAX_BOT_OMEGA,MIN_BOT_OMEGA,ROTATION_FACTOR
from utils.math_functions import *

def Get_Omega(kub_id, totalAngle, homePos):

    # print "Entered in Get Omega",radian_2_deg(totalAngle)
    # if abs(totalAngle) < ROTATION_FACTOR*2:
    #     # pr0rn 0.0

    # MAX_w = (MAX_BOT_OMEGA+MIN_BOT_OMEGA)/1.2
    MAX_w = MAX_BOT_OMEGA
    # theta_left = float(homePos[kub_id].theta-totalAngle)
    theta_lft = normalize_angle(normalize_angle(homePos[kub_id].theta)-totalAngle)*-1.0
    vw = 3.0*(theta_lft*abs(theta_lft)/(math.pi*math.pi))*MAX_w

    if abs(vw) > MAX_BOT_OMEGA:
        vw = (vw/abs(vw))*MAX_BOT_OMEGA
    #print "totalAngle",radian_2_deg(totalAngle)
    #print("####################################################") 
    #print("####################################################") 
   # print "theta_left ",radian_2_deg(theta_lft),theta_lft, " omega ", vw
    #print "homePos theta ",radian_2_deg(normalize_angle(homePos[kub_id].theta))
    #print("Max Omega ", MAX_w)
    #print "omega ",vw
    #print("####################################################") 
    #print("####################################################") 
    if abs(vw)<1*MIN_BOT_OMEGA:
        vw = 1*MIN_BOT_OMEGA*(1 if vw>0 else -1)

    if abs(theta_lft)<ROTATION_FACTOR/2:
        return 0.0

    # print "Omega return",vw
    return vw


    