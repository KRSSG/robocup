#include <list>
#include "tactics/tStop.hpp"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
#include <stdio.h>
#include <ssl_common/geometry.hpp>
namespace Strategy
{

    TStop::TStop(int botID) :
      Tactic(botID)
    {

    } // TStop

    TStop::~TStop()
    { } // ~TStop
    bool TStop::isCompleted(const BeliefState &bs,const Tactic::Param& tParam) const {
      // should add relevant logic here
      return true;
    }
    bool TStop::isActiveTactic(void) const
    {
      return false;
    }

    int TStop::chooseBestBot(const BeliefState &state, std::list<int>& freeBots, const Param& tParam, int prevID) const
    {
      int minv   = *(freeBots.begin());
      int mindis = 1000000000;
      // return the first bot
      return minv;
    } // chooseBestBot

    gr_Robot_Command TStop::execute(const BeliefState &state, const Param& tParam)
    {
      Strategy::SkillSet::SkillID sID = SkillSet::Stop;
      SkillSet::SParam sParam;

      // Execute the selected skill
      return Strategy::SkillSet::instance()->executeSkill(sID, sParam, state, botID);

    }
    Tactic::Param TStop::paramFromJSON(string json) {
      using namespace rapidjson;
      Tactic::Param tParam;
      return tParam;
    }

    string TStop::paramToJSON(Tactic::Param tParam) {
      return string("");
    }
} // namespace Strategy
