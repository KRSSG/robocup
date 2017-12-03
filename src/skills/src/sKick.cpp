#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>
#include <navigation/planners.h>
#include <navigation/controllers/waypoint.h>
#include <cstdio>
#include <vector>
namespace Strategy
{
  gr_Robot_Command SkillSet::kick(const SParam &param, const BeliefState &state, int botID)
  {
    return getRobotCommandMessage(botID, 0, 0, 0, param.KickP.power, false);
  } // kick
}
