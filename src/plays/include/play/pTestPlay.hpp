#ifndef PTEST_PLAY_HPP
#define PTEST_PLAY_HPP

#include <fstream>
#include <utility>
#include "play.hpp"
#include "krssg_ssl_msgs/BeliefState.h"
#include "tactics/tactic.h"
#include <ssl_common/config.h>
#include <ssl_common/geometry.hpp>

#define GAP 1000  //SELECT(1000,100)

namespace Strategy
{
  class PTestPlay : public Play
  {
  public:
    inline PTestPlay(const krssg_ssl_msgs::BeliefState& state) 
      : Play(state)
    {
      name = "TestPlay";

      assert(HomeTeam::SIZE == 6); // TestPlay is applicable for a team of 3 robots only
      //printf("in Set Position \n");
      PositionPlay = PLAYTYPE_YES;
      AttackPlay   = PLAYTYPE_NO;
      Tactic::Param param;
      //for (int botIDx=0; botIDx<HomeTeam::SIZE; ++botIDx) 
    //  roleList[0].push_back(std::make_pair("TKickToGoal", param));  
      /*param.PositionP.x=HALF_FIELD_MAXX;
    //  roleList[0].push_back(std::make_pair("TStop",param));  
       // param.DribbleTurnPassP.x= 0;
       // param.DribbleTurnPassP.y= HALF_FIELD_MAXY;
      param.PositionP.y=0;
      param.PositionP.finalSlope=0;
      param.PositionP.align = false;*/
      // param.PositionP.finalSlope= Vector2D<int>::angle(state.homePos[1];
      // roleList[0].push_back(std::make_pair("TDribbleTurnPass", param));
       roleList[0].push_back(std::make_pair("TGoalie",param));
      // roleList[0].push_back(std::make_pair("TStop", param));


      /*param.PositionP.x=-CENTER_X/2;
      param.PositionP.y=CENTER_Y/2;
      param.PositionP.finalSlope=0;
      param.PositionP.align = false;*/
     // roleList[0].push_back(std::make_pair("TStop", param));
     // roleList[1].push_back(std::make_pair("TKickToGoal", param));
      
      /*param.PositionP.x= CENTER_X;
      param.PositionP.y= CENTER_Y + 2*GAP;
      param.PositionP.finalSlope=-PI/2;*/
     // roleList[1].push_back(std::make_pair("TPosition", param));

       /*param.DribbleTurnPassP.x= HALF_FIELD_MAXX;
       param.DribbleTurnPassP.y= 0;*/

      roleList[1].push_back(std::make_pair("TKickToGoal", param));
     

      /*param.PositionP.x= CENTER_X;
      param.PositionP.y= CENTER_Y - 2*GAP;
      param.PositionP.finalSlope= PI/2;*/
      roleList[2].push_back(std::make_pair("TKickToGoal", param));
      

      /*param.PositionP.x= CENTER_X - GAP;
      param.PositionP.y= CENTER_Y + GAP/2;
      param.PositionP.finalSlope= -PI/4;*/
      roleList[3].push_back(std::make_pair("TStop", param));
      

      /*param.PositionP.x= CENTER_X - GAP;
      param.PositionP.y= CENTER_Y - GAP/2;
      param.PositionP.finalSlope= PI/4;*/
      roleList[4].push_back(std::make_pair("TStop", param));
      

 /*     param.PositionP.x= CENTER_X + GAP;
      param.PositionP.y= CENTER_Y + GAP/2;
      param.PositionP.finalSlope= PI/4;*/
      roleList[5].push_back(std::make_pair("TStop", param));
      

      computeMaxTacticTransits();
    }

    inline ~PTestPlay()
    { }

    inline bool applicable(void) const
    {
      // printf("Set position is applicable\n");
      // TODO make it more sophisticated
      return true;
    }

    inline Result done(void) const
    {
      // TODO make it more sophisticated and also use the timeout info to determine if the play has terminated
      // printf("Done condition not finalised\n");
      return NOT_TERMINATED;
    }

    void updateParam()
    {
      Tactic::Param param;

     for(int i=0;i<HomeTeam::SIZE;i++)
        roleList[i].clear();
           // param.PositionP.x=state.ballPos.x;
      // param.PositionP.y=state.ballPos.y;
      // param.PositionP.finalSlope=state.homePos[1].theta;
      // param.DribbleTurnPassP.x= -HALF_FIELD_MAXX;
       // param.DribbleTurnPassP.y=0;
      roleList[0].push_back(std::make_pair("TGoalie",param));
      // roleList[0].push_back(std::make_pair("TStop", param));


      /*param.PositionP.x=-CENTER_X/2;
      param.PositionP.y=CENTER_Y/2;
      param.PositionP.finalSlope=0;
      param.PositionP.align = false;*/
     // roleList[0].push_back(std::make_pair("TStop", param));
     // roleList[1].push_back(std::make_pair("TKickToGoal", param));
      
      /*param.PositionP.x= CENTER_X;
      param.PositionP.y= CENTER_Y + 2*GAP;
      param.PositionP.finalSlope=-PI/2;*/
     // roleList[1].push_back(std::make_pair("TPosition", param));
      /*param.DribbleTurnPassP.x= HALF_FIELD_MAXX;
      param.DribbleTurnPassP.y= 0;*/
      roleList[1].push_back(std::make_pair("TKickToGoal", param));
     

      /*param.PositionP.x= CENTER_X;
      param.PositionP.y= CENTER_Y - 2*GAP;
      param.PositionP.finalSlope= PI/2;*/
      roleList[2].push_back(std::make_pair("TKickToGoal", param));
      

      /*param.PositionP.x= CENTER_X - GAP;
      param.PositionP.y= CENTER_Y + GAP/2;
      param.PositionP.finalSlope= -PI/4;*/
      roleList[3].push_back(std::make_pair("TStop", param));
      

      /*param.PositionP.x= CENTER_X - GAP;
      param.PositionP.y= CENTER_Y - GAP/2;
      param.PositionP.finalSlope= PI/4;*/
      roleList[4].push_back(std::make_pair("TStop", param));
      

 /*     param.PositionP.x= CENTER_X + GAP;
      param.PositionP.y= CENTER_Y + GAP/2;
      param.PositionP.finalSlope= PI/4;*/
      roleList[5].push_back(std::make_pair("TStop", param));

    }
  }; // class PTestPlay
} // namespace Strategy

#endif // PTEST_PLAY_HPP
