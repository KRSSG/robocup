#ifndef TDEFEND_ARC_HPP
#define TDEFEND_ARC_HPP
#include "tactic.h"
#include "skills/skillSet.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include "ssl_common/config.h"
#include <sys/time.h>
#include <unistd.h>
#include "tactic_factory.h"
#include <ssl_common/geometry.hpp>

namespace Strategy {

	class TDefendARC : public Tactic {
	public:

		TDefendARC(int botID);
		 ~TDefendARC();

		 virtual bool isCompleted(const BeliefState& state,const Tactic::Param& tParam) const;
		 virtual bool isActiveTactic(void) const;
		 virtual int chooseBestBot(const BeliefState& state, std::list<int>& freeBots, const Param& tParam, int prevID = -1) const;
		 virtual gr_Robot_Command execute(const BeliefState& state, const Param& tParam);
		 virtual Tactic::Param paramFromJSON(string json);
		 virtual string paramToJSON(Tactic::Param p);

		 //some of the local functions exclusive to DefendARC tactic

		 /*
		 returns the botID of the primary threat i.e, the bot which is likely to recieve to the pass
		 else if the ball's velocity is less than certain threshold function returns -1 
		 */
		 int primary_threat(const BeliefState& state) const;

		/*
		secondary threat detection
		*/
		void secondary_threat(const BeliefState& state, std::vector<int> &vec, int pri_threat) const;

		/*
		returns the intersection point of a line and circle
		*/
		void inter_circle_and_line(Vector2D<float> P1, Vector2D<float> P2, Vector2D<float> C, float R, Vector2D<float>& P) const;

		/*
		this function returns true if the direction of ball's velocity vector lies within the 
		open angle of the goal
		*/
		bool ball_velocity_direction(const BeliefState& state) const;

		private:
			enum InternalState {
				POSITION,
				GO_TO_BALL,
				KICK
			}iState;
	};//defendARC class

	REGISTER_TACTIC(TDefendARC)

}//namespace Strategy

#endif