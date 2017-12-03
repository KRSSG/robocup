#include "comdef.h"
#include "naivePS.h"



namespace Strategy
{
  NaivePS::NaivePS(const krssg_ssl_msgs::BeliefState& state) 
    : PS(state)
  { } // NaivePS

  NaivePS::~NaivePS()
  { } // ~NaivePS

  void NaivePS::updateWeights(Play::Result termResult) const
  {
    assert(termResult != Play::NOT_TERMINATED); // Play must terminate before updating weights

    float mult;

    switch (termResult)
    {
    case Play::ABORTED:
      mult = 10.0f/11.0f;
      break;

    case Play::COMPLETED:
      mult = 11.0f/10.0f;
      break;

    case Play::FAILURE:
      mult = 2.0f/3.0f;
      break;

    case Play::SUCCESS:
      mult = 3.0f/2.0f;
      break;

    default:break;
     // Logger::abort("Unhandled play done return value");
    }
    
    // TODO Do the post-processing here
    playList[playID]->weight *= mult;
  }
} // namespace Strategy
