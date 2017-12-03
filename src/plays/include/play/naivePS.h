#pragma once

#ifndef NAIVE_PS_H
#define NAIVE_PS_H

#include "play.hpp"
#include "ps.h"
#include "krssg_ssl_msgs/BeliefState.h"

namespace Strategy
{
  class BeliefState;

  class NaivePS : public PS
  {
  public:
    NaivePS(const krssg_ssl_msgs::BeliefState& state);
    ~NaivePS();

    void updateWeights(Play::Result termResult) const;
  }; // class NaivePS
} // namespace Strategy

#endif // NAIVE_PS_H
