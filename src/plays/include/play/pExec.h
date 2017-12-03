#ifndef PEXEC_H
#define PEXEC_H

#include "cs.hpp"
#include "naivePS.h"
#include "expPS.h"
#include "play.hpp"
#include "ps.h"
#include "../../../ssl_robot/include/ssl_robot/robot.h"
#include <tactics/tactic_factory.h>
#include "krssg_ssl_msgs/BeliefState.h"
#include <ssl_common/config.h>
#include "ros/ros.h"

// Forward Declarations
namespace Strategy
{
  //class BeliefState;
  //class Robot;
}

namespace Strategy
{

        //private PS_CLASS ----------- this was used by default
  class PExec : public NaivePS
                
  {
  private:
    Play::Result playResult;
    Robot*       robot[HomeTeam::SIZE];
    unsigned int roleBotMapping[HomeTeam::SIZE];
    //Tactic*    tactic[HomeTeam::SIZE];

  public:
    PExec(krssg_ssl_msgs::BeliefState* state,ros::NodeHandle&);

    ~PExec();

  private:
    // Stores the index to the tactics in all role that is to be executed by the team
    unsigned int currTacticIdx[HomeTeam::SIZE];
   // Current tactic in execution by each bot
    std::pair<string, Tactic::Param> currTactic[HomeTeam::SIZE];
    auto_ptr<Tactic> selTactic;

    void assignRoles(krssg_ssl_msgs::BeliefState &bs);

    bool canTransit(void);

    bool tryTransit(void);

    bool transit(krssg_ssl_msgs::BeliefState &bs);

  public:
    Robot** selectPlay(krssg_ssl_msgs::BeliefState &bs);

    Robot** executePlay(krssg_ssl_msgs::BeliefState &bs);

    void evaluatePlay(void);

    bool playTerminated(krssg_ssl_msgs::BeliefState &bs);

    bool playCompleted(krssg_ssl_msgs::BeliefState &bs);
  }; // class PExec
} // namespace Strategy

#endif // PEXEC_H