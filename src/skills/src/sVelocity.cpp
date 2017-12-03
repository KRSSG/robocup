#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>

namespace Strategy
{
  gr_Robot_Command SkillSet::velocity(const SParam &param, const BeliefState &state, int botID)
  {
//    Logger::toStdOut("%f\n", state->homeAngle[botID]);
  	 return getRobotCommandMessage(botID, param.VelocityP.v_x, param.VelocityP.v_y, param.VelocityP.v_t, 0, false);
  } // velocity
}


