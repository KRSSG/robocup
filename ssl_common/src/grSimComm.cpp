#include <cstdio>
#include <ssl_common/config.h>
#include "grSimComm.h"


namespace Strategy
{
  

  gr_Robot_Command getRobotCommandMessage(int   botID,
                                          float v_x,
                                          float v_y,
                                          float v_t,
                                          float kickPower,
                                          bool  dribble)
  {
	//Logger::toStdOut("Bot: %d, %f, %f, %f, %d\n", botID, v_x, v_y, v_t, kickPower, (int)dribble);
    gr_Robot_Command command;
    command.id = botID;
    command.wheelsspeed = 0;
    command.veltangent = (v_y/1000.0f); // Unit of length in Strategy used is mm but it is m in grSim
    command.velnormal = (-v_x/1000.0f); // Unit of length in Strategy used is mm but it is m in grSim
    command.velangular = v_t;
    command.kickspeedx = kickPower;
    command.kickspeedz = 0;  // chip kicker disabled
    command.spinner = dribble;
    

    return command;
  }
} // namespace HAL
