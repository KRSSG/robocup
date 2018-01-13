// waypoint controller, that takes nextWP and nextNWP to generate control commands for the SSL robot.

#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <krssg_ssl_msgs/gr_Robot_Command.h>
#include "krssg_ssl_msgs/BeliefState.h"
#include <ssl_common/geometry.hpp>
using krssg_ssl_msgs::gr_Robot_Command;
using krssg_ssl_msgs::BeliefState;
namespace Navigation {
  gr_Robot_Command waypointCommand( int botID, 
                                    const BeliefState &state, 
                                    Vector2D<int> nextWP, 
                                    Vector2D<int> nextNWP, 
                                    float finalSlope, 
                                    bool align);
}
#endif