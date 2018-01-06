// Utilily functions for creating gr_Robot_Command message.

#ifndef SIM3D_COMM_H
#define SIM3D_COMM_H

#include <list>
#include <krssg_ssl_msgs/gr_Robot_Command.h>

namespace Strategy {
  using krssg_ssl_msgs::gr_Robot_Command;
  gr_Robot_Command getRobotCommandMessage(int   botID,
                                          float v_x,
                                          float v_y,
                                          float v_t,
                                          float kickPower,
                                          bool  dribble);
}

#endif // SIM3D_COMM_H
