#ifndef T_GOALIE_HPP
#define T_GOALIE_HPP
#include "tactic.h"
#include "skills/skillSet.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include "ssl_common/config.h"
#include <sys/time.h>
#include <unistd.h>
#include "tactic_factory.h"
#include <ssl_common/geometry.hpp>

namespace Strategy{

	class TGoalie: public Tactic
	{
	public:
		TGoalie(int botID);
		~TGoalie();
	
		virtual bool isCompleted(const BeliefState &bs,const Tactic::Param& tParam) const ;

		virtual bool isActiveTactic(void) const;

		virtual int chooseBestBot(const BeliefState &bs, std::list<int>& freeBots, const Param& tParam, int prevID = -1) const;

		virtual gr_Robot_Command execute(const BeliefState &state, const Param& tParam);

		virtual Tactic::Param paramFromJSON(string json);
		virtual string paramToJSON(Tactic::Param p);	
	};

	REGISTER_TACTIC(TGoalie)

}
#endif