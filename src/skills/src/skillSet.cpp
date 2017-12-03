// This file contains all the skillSet functons
#include <cassert>
#include "skillSet.h"
// #include "krssg_ssl_msgs/geometry.hpp"
// #include <assert.h>
using namespace std;

namespace Strategy
{
  SkillSet* SkillSet::m_pInstance = NULL;

  SkillSet *SkillSet::instance() {
    if (!m_pInstance) {
      m_pInstance = new SkillSet;
    }
    assert(m_pInstance != NULL);
    return m_pInstance;
  }
  // SkillSet will be a singleton object.
  SkillSet::SkillSet()
  {

    // Resetting the skill list
    for (int sID = 0; sID < MAX_SKILLS; ++sID)
    {
      skillList[sID] = NULL;
    }

    // Mapping the skill IDs to their corresponding function pointers

    skillList[Spin] = &SkillSet::spin;    
    skillList[Kick] = &SkillSet::kick;
    skillList[Stop] = &SkillSet::stop;
    skillList[Dribble] = &SkillSet::dribble;
    skillList[Velocity] = &SkillSet::velocity;
    skillList[GoToBall] = &SkillSet::goToBall;
    skillList[GoToPoint] = &SkillSet::goToPoint;
    skillList[DefendPoint] = &SkillSet::defendPoint;
    skillList[DribbleToPoint] = &SkillSet::dribbleToPoint;
    skillList[ReceiveBall] = &SkillSet::receiveBall;
    skillList[GoalKeeping] = &SkillSet::goalKeeping;
    skillList[KickToPoint] = &SkillSet::kickToPoint;
    skillList[TurnToPoint] = &SkillSet::turnToPoint;
    skillList[TurnToAngle] = &SkillSet::turnToAngle;
    skillList[DribbleTurn] = &SkillSet::dribbleTurn;
    // Initialization check
    for (int sID = 0; sID < MAX_SKILLS; ++sID)
    {
      assert(skillList[sID] != NULL); // Skill enum not mapped to the corresponding skill function
    }

  } // SkillSet  

} // namespace Strategy
