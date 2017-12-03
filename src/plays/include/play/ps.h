// Play Selector class definitions

#ifndef PS_H
#define PS_H

#include "playBook.h"
#include "play.hpp"

namespace Strategy
{
  class BeliefState;
  class PExec;
  
  class PS : protected PlayBook
  {
    friend class PExec;

  protected:
    PlayID       playID;
    float        pProb[PlayBook::MAX_PLAYS];
    bool         appl[PlayBook::MAX_PLAYS];

    void         select(void);
    virtual void updateWeights(Play::Result termResult) const = 0;
    
  public:
    PS(const krssg_ssl_msgs::BeliefState& state);
    ~PS();
  }; // class PS
} // namespace Strategy

#endif // PS_H
