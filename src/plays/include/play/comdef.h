/* This file contains common definitions and settings for Team Id Code
 * and their color codes and the number of Bots
 */

#ifndef COMDEF_H
#define COMDEF_H

#ifdef WIN32
# define WIN32_LEAN_AND_MEAN
# include <winsock2.h>
#endif // WIN32

//**************** Common definitions ******************//
#define NOT_USED(x)      ((void)(x))

#define MINIMUM(a,b) ({typeof(a) _a = (a); typeof(b) _b = (b); (_a < _b) ? _a : _b;})
#define MAXIMUM(a,b) ({typeof(a) _a = (a); typeof(b) _b = (b); (_a > _b) ? _a : _b;})
#define ABSOLUTE(a)  ({typeof(a) _a = (a); (_a < 0) ? -_a : _a;})

namespace Simulator
{
  // Team color codes
  enum TeamColor
  {
    BLUE_TEAM  = 0,
    YELLOW_TEAM = 1
  };
}

namespace Strategy
{
  // Team ID codes
  enum TeamID
  {
    HOME_TEAM = 0,
    AWAY_TEAM = 1
  };

  // Home team properties
  class HomeTeam
  {
  public:
    static Simulator::TeamColor COLOR;
    static const int                  SIZE  = 6;
  };

  // Away team properties
  class AwayTeam
  {
  public:
    static Simulator::TeamColor COLOR;
    static const int                  SIZE  = 6;
  };
} // namespace Strategy

namespace Simulator
{
  // Blue team properties
  class BlueTeam
  {
  public:
    static Strategy::TeamID ID;
    static const int              SIZE = 6;
  };

  // Yellow team properties
  class YellowTeam
  {
  public:
    static Strategy::TeamID ID;
    static const int              SIZE = 6;
  };
} // namespace Simulator

#endif // COMDEF_H
