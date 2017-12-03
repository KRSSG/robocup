#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>

namespace Strategy
{
  gr_Robot_Command SkillSet::spin(const SParam &param, const BeliefState &state, int botID)
  {
//    Logger::toStdOut("%f\n", state->homeAngle[botID]);
  	 return getRobotCommandMessage(botID, 0, 0, param.SpinP.radPerSec, 0, false);
  } // spin
}
