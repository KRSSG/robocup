# Includes all the global parameters as in 'ssl_common/conig.h'
GRSIM = True

def SELECT(sim_param, ssl_param):
    if GRSIM:     #Parameters for the simulator
        return sim_param
    return ssl_param


BS_ADDRESS                   = ['127.0.0.1:11211']

# Geometry constants
GOAL_DEPTH                   = SELECT(300, 300)
CENTER_X                     = SELECT(0, 0)
CENTER_Y                     = SELECT(0, 0)
HALF_FIELD_MAXX              = SELECT(6000, 4500)
HALF_FIELD_MAXY              = SELECT(4500, 3000)
OUR_GOAL_MAXY                = SELECT(600,	500)
OUR_GOAL_MINY                = SELECT(-600, -500)
OUR_GOAL_WIDTH               = OUR_GOAL_MAXY - OUR_GOAL_MINY
CENTER_CIRCLE_DIAMETER       = SELECT(1000,1000)
CENTER_CIRCLE_RADIUS		 = SELECT(500,500)
DBOX_WIDTH                   = SELECT(2400,2000)	#Along y       
DBOX_HEIGHT					 = SELECT(1200,1000)	    #Along x      
OUR_DBOX_MAXY				 = SELECT(1200,1000)
OUR_DBOX_MINY				 = SELECT(-1200,-1000)
OUR_DBOX_X 					 =-HALF_FIELD_MAXX + DBOX_HEIGHT
# Old DBOX Constants
DBOX_WIDTH_OLD                   = SELECT(1000,1000)       # Along X direction
DBOX_HEIGHT_OLD					 = SELECT(1250,1250)	   # Along positive y direction
DBOX_SMALLER_LENGTH          = SELECT(500,500)         # smaller length of line connecting the two quadrants along Y-axis
DBOX_LARGER_LENGTH_OLD           = SELECT(1350,2500)       # larger length of line connecting the two quadrants along Y-axis
DBOX_RADIUS                  = SELECT(425,1000)



# Planning Constants
CLEARANCE_PATH_PLANNER     = SELECT(500, 300)                 #mm
MID_FIELD_THRESH           = SELECT(10, 150)                  #mm
BOT_RADIUS                 = SELECT(90, 90)                  #mm
BALL_RADIUS                = SELECT(21.5, 21.5)                 #mm
SAFE_RADIUS                = (BOT_RADIUS * 2)
COLLISION_DIST             = (BOT_RADIUS * 7)
DRIBBLER_BALL_THRESH       = SELECT(300, 110)                 #mm
FREEKICK_RADIUS            = SELECT(25,650) # To set
FREEBALL_RADIUS            = SELECT(30,700) # To set
KICKOFF_RADIUS             = SELECT(18,200) # To set

MOVING_BALL_VELOCITY         = SELECT(40, 30)
MIN_DIST_FROM_TARGET         = SELECT(30.0, 25.0)
BALL_AT_CORNER_THRESH        = SELECT(20,20)


BOT_MAX                      = SELECT(6, 5)
BOT_ANGLE_THRESH			 = SELECT(0.32,0.32)
OUR_GOAL_MAXX				 = SELECT(340,340)
OPP_GOAL_MAXY                = SELECT(600, 600)
OPP_GOAL_MINY                = SELECT(-600, -600)
OPP_GOAL_WIDTH               = OPP_GOAL_MAXY - OPP_GOAL_MINY
DBOX_DEPTH                   = SELECT(10, 10)

#Bot Parameteres configuration 
ROTATION_FACTOR            = SELECT(0.15, 0.15)               
DISTANCE_THRESH            = 1.5*SELECT(200.0,200.0)

RFACTOR                    = SELECT(3,   0.3)
RFACTOR_SMALL              = SELECT(0.6, 0.15)

CLEARANCE_PATH_PLANNER     = SELECT(500, 300)                 #mm
MID_FIELD_THRESH           = SELECT(10, 150)                  #mm
BOT_RADIUS                 = SELECT(110, 150)                  #mm
BALL_RADIUS                = SELECT(21.5, 40)                 #mm
SAFE_RADIUS                = (BOT_RADIUS * 2)
COLLISION_DIST             = (BOT_RADIUS * 7)
DRIBBLER_BALL_THRESH       = SELECT(300, 500)                 #mm
FREEKICK_RADIUS            = SELECT(25,650)
FREEBALL_RADIUS            = SELECT(30,700)
KICKOFF_RADIUS             = SELECT(18,200)


BOT_BALL_THRESH            = SELECT(110, 150)                  #mm
BOT_BALL_THRESH_FOR_PR     = SELECT(105, 200)                  #mm
BOT_POINT_THRESH           = SELECT(105, 147)                   #mm

STRIP_WIDTH_X              = BOT_RADIUS*1.5
STRIP_WIDTH_Y              = BOT_RADIUS*1.5
MAX_FIELD_DIST             = SELECT(1000, 3500)                #mm
MAX_WHEEL_SPEED            = SELECT(2000, 100)                 #mm/s
MAX_BOT_LINEAR_ACC         = SELECT(1000, 100)                 #mm/s/s
MAX_BOT_LINEAR_VEL_CHANGE  = SELECT(10, 3)

FFV 					  = SELECT(0.5,7.0)
FFA 					  = SELECT(1.0,5.0)

#MAX_BOT_ACCELERATION       = SELECT(900, 900) as per sudo  
MAX_BOT_ACCELERATION       = SELECT(1000*FFA, 1000*FFA)  
MAX_BOT_SPEED              = SELECT(1200*FFV, 500.0*FFV)           #mm
MIN_BOT_SPEED              = SELECT(5, 30)                      #mm/s
MAX_BOT_OMEGA              = SELECT(0.5, 3.0)                     #rad/s//2
MIN_BOT_OMEGA              = SELECT(0.15,0.8)                    #rad/s
MAX_BOT_OMEGA_ACC 		   = SELECT(0.1, 0.1)					# rad/s^2
MAX_BACK_DRIBBLE_V_Y       = SELECT(500, 500)                   #mm/s
MAX_FRONT_DRIBBLE_V_Y      = SELECT(1200, 1200)                 #mm/s
MAX_DRIBBLE_V_X            = SELECT(200, 100)                   #mm/s
MAX_DRIBBLE_R              = SELECT(30, 3)                       #rad
MAX_BALL_SPEED             = SELECT(2700, 2700)                #mm/s
DRIBBLER_BALL_ANGLE_RANGE  = SELECT(0.2, 0.10)                  #rad
SATISFIABLE_THETA          = SELECT(0.08, 0.1)                  #rad
SATISFIABLE_THETA_SHARP    = SELECT(0.01, 0.01)                 #rad
SATISFIABLE_THETA_DEG      = SELECT(0.05236, 0.05236)
MAX_BALL_STEAL_DIST        = SELECT(800, 800)

#SSL param. not needed. 
MAX_KICK_SPEED             = SELECT(0,0)

#If the velocity of a bot is below this value, then the bot has effectively zero velocity 
ZERO_VELOCITY_THRESHOLD    = SELECT(10, 10)
ZERO_VELOCITY_THRESHOLD_SQ = (ZERO_VELOCITY_THRESHOLD * ZERO_VELOCITY_THRESHOLD)
LOW_BALL_VELOCITY_THRES    = SELECT(50, 50)
LOW_BALL_VELOCITY_THRES_SQ = (LOW_BALL_VELOCITY_THRES*LOW_BALL_VELOCITY_THRES)

#Parameters useful for camera's data transformation.
OUR_GOAL_Y = 500
OPP_GOAL_Y = 0
NETWORK_DELAY = SELECT(0.05, 0.05)  #Network Delay in miliseconds

#Distance Hysteresis factor or switching of roles
HYSTERESIS = SELECT(20000, 300)

STRATEGY_GUI_MULTICAST_PORT = 10001 
STRATEGY_GUI_MULTICAST_ADDR = "224.5.23.1"

