#ifndef PNONE_HPP
#define PNONE_HPP

#include <utility>
#include "play.hpp"
#include "krssg_ssl_msgs/BeliefState.h"
#include "tactics/tactic.h"

namespace Strategy
{
  class PNone : public Play
  {
  public:
    inline PNone(const krssg_ssl_msgs::BeliefState& state) 
      : Play(state)
    {
      name = "None";
    }

    inline ~PNone()
    { }

    inline bool applicable(void) const
    {
      return false; // Should always be false as this the None play
    }

    inline Result done(void) const
    {
      return COMPLETED;
    }
    void updateParam()
    {
      
    }
  }; // class PNone
} // namespace Strategy

#endif // PNONE_HPP
