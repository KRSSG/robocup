#include "skillSet.h"
#include <ssl_common/grSimComm.h>
namespace Strategy
{
  gr_Robot_Command SkillSet::stop(const SParam& param, const BeliefState &state, int botID)
  {
    return getRobotCommandMessage(botID, 0, 0, 0, 0, false);
  } // stop
}
