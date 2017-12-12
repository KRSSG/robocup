#pragma once
#ifndef CONFIG_H
#define CONFIG_H
#include <string>


// Add new parameters by declaring extern variable here and defining it in "config.cpp"

extern const int   BOT_MAX                    ;

extern const int GOAL_DEPTH;
extern const float MOVING_BALL_VELOCITY       ;
extern const float MIN_DIST_FROM_TARGET       ;
extern const int CENTER_X                     ;
extern const int CENTER_Y                     ;
extern const int HALF_FIELD_MAXX              ; //actual 225 (rugged surface at end)
extern const int HALF_FIELD_MAXY              ;

#define OUR_GOAL_X                            -HALF_FIELD_MAXX
#define OPP_GOAL_X                            HALF_FIELD_MAXX

extern const int OUR_GOAL_MAXY                ;
extern const int OUR_GOAL_MINY                ;
extern const int OPP_GOAL_MAXY                ;
extern const int OPP_GOAL_MINY                ;
extern const int OUR_GOAL_WIDTH               ;
extern const int OPP_GOAL_WIDTH               ;
extern const int CENTER_CIRCLE_DIAMETER       ;
extern const int DBOX_WIDTH                   ;       //Along X direction
extern const int DBOX_HEIGHT                  ;       //Along Y direction(half height in each y direction)
extern const int DBOX_DEPTH                   ;
extern const int BALL_AT_CORNER_THRESH        ; 

/* Bot Parameteres configuration */
extern const float ROTATION_FACTOR            ;                //

extern const float RFACTOR                    ;
extern const float RFACTOR_SMALL              ;

extern const int CLEARANCE_PATH_PLANNER       ;               //mm
extern const int MID_FIELD_THRESH             ;                       // mm
extern const float BOT_RADIUS                 ;                       // mm
extern const float BALL_RADIUS                ;                       // mm
extern const float SAFE_RADIUS                ;
extern const float COLLISION_DIST             ;
extern const int DRIBBLER_BALL_THRESH         ;            // mm
extern const int KICKOFF_RADIUS								;            // < 25cm, ~18 just so that whole bot is inside circle.
extern const int FREEKICK_RADIUS							;            // = should be 20cm, actually 25cm
extern const int FREEBALL_RADIUS							;            // = should be 25cm, actually 30cm



extern const int BOT_BALL_THRESH              ;                  // mm
extern const int BOT_BALL_THRESH_FOR_PR       ;                  // mm
extern const int BOT_POINT_THRESH             ;                     // mm

extern const int STRIP_WIDTH_X                ;
extern const int STRIP_WIDTH_Y                ;
extern const int MAX_FIELD_DIST               ;                // mm
extern const float MAX_WHEEL_SPEED            ;                 //mm/s
extern const float MAX_BOT_LINEAR_ACC         ;                // mm/s/s
extern const float MAX_BOT_LINEAR_VEL_CHANGE  ;
extern const float MAX_BOT_SPEED              ;                 // mm
extern const float MIN_BOT_SPEED              ;                     // mm/s
extern const float MAX_BALL_SPEED              ;                     // mm/s
extern const float MAX_BOT_OMEGA              ;                     // rad/s//2
extern const float MIN_BOT_OMEGA              ;                     // rad/s
extern const float MAX_BACK_DRIBBLE_V_Y       ;                  // mm/s
extern const float MAX_FRONT_DRIBBLE_V_Y      ;                // mm/s
extern const float MAX_DRIBBLE_V_X            ;                 // mm/s
extern const float MAX_DRIBBLE_R              ;                      // rad
extern const float DRIBBLER_BALL_ANGLE_RANGE  ;                // rad
extern const float SATISFIABLE_THETA          ;                // rad
extern const float SATISFIABLE_THETA_SHARP    ;              // rad
extern const float MAX_BALL_STEAL_DIST        ;
/// SSL param. not needed. 
extern const int MAX_KICK_SPEED               ;
/* If the velocity of a bot is below this value, then the bot has effectively zero velocity */
extern const float ZERO_VELOCITY_THRESHOLD    ;
extern const float ZERO_VELOCITY_THRESHOLD_SQ ;
extern const float LOW_BALL_VELOCITY_THRES    ;
extern const float LOW_BALL_VELOCITY_THRES_SQ ;
// Parameters useful for camera's data transformation.
extern const int OUR_GOAL_Y ;
extern const int OPP_GOAL_Y ;
extern const float NETWORK_DELAY ;  // Network Delay in miliseconds
//Distance Hysteresis factor for switching of roles
extern const int HYSTERESIS ;

extern const short STRATEGY_GUI_MULTICAST_PORT ;
extern const std::string STRATEGY_GUI_MULTICAST_ADDR ;

// namespace Simulator
// {
//   // THESE ARE THE RATES FOR UPDATE ETC
//   static const int VIEW_REFRESH_RATE = 30;
//   static const int UPDATE_RATE       = 30;
//   static const int SEND_RATE         = 60;
  
//   // THESE ARE THE NETWORK PROPERTIES
//   static const int   VISION_PORT  = 10002;
//   static const int   COMMAND_PORT = 20012;
//   const char ADDRESS[]             = "224.5.23.2";
//   const char INTER_FACE[]          = "";
//   static const int BALL_AREA    = 200;
//   static const int BALL_PIXEL_X = 30;
//   static const int BALL_PIXEL_Y = 30;
//   static const int BOT_PIXEL_X  = 60;
//   static const int BOT_PIXEL_Y  = 60;
// }


#endif  // INCL_CONFIG_H 
