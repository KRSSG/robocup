#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>

namespace Strategy
{
  gr_Robot_Command SkillSet::dribble(const SParam &param, const BeliefState &state, int botID)
  {
//    Logger::toStdOut("%f\n", state->homeAngle[botID]);
  	 return getRobotCommandMessage(botID, 0, 0, 0, 0, true);
  } // dribble
}
