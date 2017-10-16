# beliefstate message

bool               isteamyellow
uint32             frame_number 
float64            t_capture     
float64            t_sent   
geometry_msgs/Pose2D  ballPos       
geometry_msgs/Point32 ballVel  
geometry_msgs/Pose2D[] awayPos 
geometry_msgs/Pose2D[] homePos
geometry_msgs/Pose2D[] awayVel 
geometry_msgs/Pose2D[] homeVel
bool ballDetected
bool[] homeDetected
bool[] awayDetected
uint8   our_bot_closest_to_ball
uint8   opp_bot_closest_to_ball
uint8   our_goalie			#returns 10 for no goalie
uint8   opp_goalie			#returns 10 for no goalie
uint8   opp_bot_marking_our_attacker
bool    ball_at_corners
bool    ball_in_our_half
bool    ball_in_our_possession

# add other stuff eg. field geometry, referee signals, bot/ball velocities and acc, score count, predicates, etc.
# node will have to subscribe to vision, refbox 