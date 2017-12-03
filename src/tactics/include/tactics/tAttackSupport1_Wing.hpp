#ifndef T_ATTACKSUPPORT1_WING_HPP
#define T_ATTACKSUPPORT1_WING_HPP
#include "tactic.h"
#include "skills/skillSet.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include "ssl_common/config.h"
#include <sys/time.h>
#include <unistd.h>
#include "tactic_factory.h"
#include <ssl_common/geometry.hpp>

namespace Strategy{

	class TAttackSupport1_Wing: public Tactic
	{
	public:
		TAttackSupport1_Wing(int botID);
		~TAttackSupport1_Wing();
	
		virtual bool isCompleted(const BeliefState &bs,const Tactic::Param& tParam) const ;

		virtual bool isActiveTactic(void) const;

		virtual bool isApplicable(const BeliefState &bs, const Param& tParam);

		virtual int chooseBestBot(const BeliefState &bs, std::list<int>& freeBots, const Param& tParam, int prevID = -1) const;

		virtual gr_Robot_Command execute(const BeliefState &state, const Param& tParam);

		virtual Tactic::Param paramFromJSON(string json);
		virtual string paramToJSON(Tactic::Param p);

	};

	REGISTER_TACTIC(TAttackSupport1_Wing)

}
#endif