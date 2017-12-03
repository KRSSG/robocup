import sys
import rospy

sys.path.insert(0,'../')
sys.path.append('../../..')
from krssg_ssl_msgs.msg import BeliefState

from tactic_factory import *

state = BeliefState()

#tacticTest = TPosition(1, state)



def bs_callback(data):
		state.isteamyellow                 = data.isteamyellow
		state.frame_number                 = data.frame_number
		state.t_capture                    = data.t_capture
		state.ballPos                      = data.ballPos
		state.ballVel                      = data.ballVel
		state.awayPos                      = data.awayPos
		state.homePos                      = data.homePos
		state.awayVel                      = data.awayVel
		state.homeVel                      = data.homeVel
		state.ballDetected                 = data.ballDetected
		state.homeDetected                 = data.homeDetected
		state.awayDetected                 = data.awayDetected
		state.our_bot_closest_to_ball      = data.our_bot_closest_to_ball
		state.opp_bot_closest_to_ball      = data.opp_bot_closest_to_ball
		state.our_goalie                   = data.our_goalie
		state.opp_goalie                   = data.opp_goalie
		state.opp_bot_marking_our_attacker = data.opp_bot_marking_our_attacker
		state.ball_at_corners              = data.ball_at_corners
		state.ball_in_our_half             = data.ball_in_our_half
		state.ball_in_our_possession       = data.ball_in_our_possession

		detected = 0

		tacticTest = TPosition(1, state)
		tacticTest.execute(state, pub)
		print("HEY! SHUBHAM I AM IN CALLBACK")




def node():
	
	print("HEY! SHUBHAM")
	rospy.Subscriber('/belief_state', BeliefState, bs_callback)

if __name__ == '__main__':
	print("HEY! SHUBHAM")

	node()

	