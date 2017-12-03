/* This file updates the weights of the different plays depending upon their outcome:
 * SUCCESS,COMPLETED,FAILURE OR ABORTED.
 */

#include <cmath>
#include "comdef.h"
#include "expPS.h"



namespace Strategy
{
  ExpPS::ExpPS(const krssg_ssl_msgs::BeliefState& state) : 
    PS(state)
  { }

  ExpPS::~ExpPS()
  { }

  void ExpPS::updateWeights(Play::Result termResult) const
  {
    assert(termResult != Play::NOT_TERMINATED); // Play must terminate before updating weights

    static bool init = true;
    float       wcap[PlayBook::MAX_PLAYS];
    float       norm[PlayBook::MAX_PLAYS];
    float       mult[PlayBook::MAX_PLAYS];

    /* Initializing all the multiplication factors to default value of 1.0 */
    for (int pID=0; pID<PlayBook::MAX_PLAYS; ++pID)
    {
      mult[pID] = 1.0f;
    }

    /* Updating multplication factor of the just terminated play */
    switch (termResult)
    {
    case Play::ABORTED:
      mult[playID] = 10.0f/11.0f;
      break;

    case Play::COMPLETED:
      mult[playID] = 11.0f/10.0f;
      break;

    case Play::FAILURE:
      mult[playID] = 2.0f/3.0f;
      break;

    case Play::SUCCESS:
      mult[playID] = 3.0f/2.0f;
      break;

    default:
      break;
      // TODO Add handling of other return values. Update here after done.
      //Logger::abort("Unhandled play done return value");
    }

    if (init == false)
    {
      // Computing wcap values
      for (int pID=0; pID<PlayBook::MAX_PLAYS; ++pID)
      {
        wcap[pID] = playList[pID]->weight*pow((float)mult[pID], (float)1.0/pProb[pID]);
      }

      // Computing normalization factors
      float num = 0, den = 0;
      for (int pID=0; pID<PlayBook::MAX_PLAYS; ++pID)
      {
        if (appl[pID])
        {
          num += playList[pID]->weight;
          den += wcap[pID];
        }
      }
      for (int pID=0; pID<PlayBook::MAX_PLAYS; ++pID)
      {
        if (appl[pID] == false)
        {
          norm[pID] = 1.0f;
        }
        else
        {
          norm[pID] = num/den;
        }
      }

      // Normalizing the play weights
      for (int pID=0; pID<PlayBook::MAX_PLAYS; ++pID)
      {
        playList[pID]->weight = wcap[pID]*norm[pID];
      }
    }

    init = false;
  }
} // namespace Strategy
