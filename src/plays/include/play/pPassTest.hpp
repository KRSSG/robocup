#ifndef P_PASS_TEST_HPP
#define P_PASS_TEST_HPP

#include <utility>
#include "play.hpp"
#include "krssg_ssl_msgs/BeliefState.h"
#include "tactics/tactic.h"
#include <ssl_common/config.h>
#include <ssl_common/geometry.hpp>
#include <math.h>
#define GAP 1000  //SELECT(1000,100)
#define u MAX_BOT_SPEED
#define v MAX_BOT_SPEED
#define w MAX_BALL_SPEED

namespace Strategy
{
  class PPassTest : public Play
  {
  public:
    PPassTest(const krssg_ssl_msgs::BeliefState& state) ;
    bool assignTactics();
    Vector2D<int> destPassPoint;
    inline ~PPassTest()
    { }

    bool applicable(void) const;
    
    inline Result done(void) const
    {
      // TODO make it more sophisticated and also use the timeout info to determine if the play has terminated
      // printf("Done condition not finalised\n");
      return NOT_TERMINATED;
    }
    void updateParam();
  private:
    int findMarker(int);
  }; // class PTestPlay
} // namespace Strategy

#endif // PTEST_PLAY_HPP
