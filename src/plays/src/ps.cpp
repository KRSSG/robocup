/* This file contains the PS class(Play Selector) that implements the algorithm
 * to select the play to be implemented in each frame. It uses the probabilistic model 
 * to select the most appropriate play among the list of applicable plays.
 */

#include <cstdlib>
#include "ps.h"
#include "playBook.h"
#include "play.hpp"

using namespace std;

namespace Strategy
{
  PS::PS(const krssg_ssl_msgs::BeliefState& state) : 
    PlayBook(state),
    playID(PlayBook::None)
  {
    for (int playID=0; playID<PlayBook::MAX_PLAYS; ++playID)
    {
      appl[playID] = false;
      pProb[playID] = 0.0f;
    }
 }

  PS::~PS()
  { }

  void PS::select(void)
  {
    // Find the applicable plays
    for (int pID=0; pID<PlayBook::MAX_PLAYS; ++pID)
    {
      appl[pID] = playList[pID]->applicable();
    }

    float cumWeight = 0.0f;
    for (int pID=0; pID<PlayBook::MAX_PLAYS; ++pID)
    {
      if (appl[pID])
      {
        cumWeight += playList[pID]->weight;
      }
    }

    for (int pID=0; pID<PlayBook::MAX_PLAYS; ++pID)
    {
      if (appl[pID])
      {
        pProb[pID] = playList[pID]->weight/cumWeight;
      }
      else
      {
        pProb[pID] = 0.0f;
      }
    }

    float cumProb = 0.0f;
    float randVal = (float)rand()/RAND_MAX + 0.000001f;
    if (randVal >= 1.0f)
    {
      randVal = 0.999999f;
    }
    
    //what is this thing doing ??

    playID = PlayBook::None;
    for (int pID=0; pID<PlayBook::MAX_PLAYS; ++pID)
    {
      cumProb += pProb[pID];
      if (cumProb >= randVal && appl[pID])
      {
        playID = (PlayID)pID;
        break;
      }
    }
     //playID=(PlayID)(3);
    assert(playID != PlayBook::None); // No play selected

    playList[playID]->startTimer();
  } // select
} // namespace Strategy
