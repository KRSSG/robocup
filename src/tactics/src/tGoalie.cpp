#include <list>
#include "tactics/tGoalie.hpp"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include <krssg_ssl_msgs/BeliefState.h>
#include "rapidjson/stringbuffer.h"
#include <ssl_common/config.h>
#include <iostream>
#include <stdio.h>
#include <ssl_common/geometry.hpp>

#define KICK_RANGE_THRESH 3 * MAX_DRIBBLE_R
#define THRES (0.8f)
#define THETA_THRESH 0.005
#define SHIFT (HALF_FIELD_MAXX/9.0)


namespace Strategy{

	TGoalie::TGoalie(int botID):
		Tactic(botID){

		}
	
	TGoalie::~TGoalie(){

	}

	bool TGoalie::isCompleted(const BeliefState &bs,const Tactic::Param& tParam) const{
		//add logic
		return false;
	}
	bool TGoalie::isActiveTactic(void)const{
		return true;
	}

	int TGoalie::chooseBestBot(const BeliefState &state, std::list<int>& freeBots, const Param& tParam, int prevID) const{
      return 1;
	  int minv   = *(freeBots.begin());
      int mindis = 1000000;
      Vector2D<int> tGoToPoint(-HALF_FIELD_MAXX, 0);
      for (std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
      {
        // TODO make the bot choosing process more sophisticated, the logic below returns the 1st available bot
        Vector2D<int> homePos(state.homePos[*it].x, state.homePos[*it].y);
        float dis_from_point = sqrt((homePos - tGoToPoint).absSq());
        if(*it == prevID)
          dis_from_point -= HYSTERESIS;
        if(dis_from_point < mindis)
        {
          mindis = dis_from_point;
          minv = *it;
        }
      }
      //printf("%d assigned Position\n", minv);

      return minv;
	}

	gr_Robot_Command TGoalie::execute(const BeliefState &state, const Tactic::Param& tParam){



		Vector2D<int> ballAim, goalieTarget;
		Vector2D<int> botpos (state.homePos[botID].x, state.homePos[botID].y);
		Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
		Vector2D<int> ballVel(state.ballVel.x , state.ballVel.y);

		Strategy::SkillSet::SkillID sID;
		SkillSet::SParam sParam;
		
		
		
		float dist = Vector2D<int>::dist(ballPos, botpos);
		cout << "Velocity ::DSFGHFHGHJGHJGHJ :::::   " <<  sqrt(ballVel.absSq()) << endl;
		cout << " Max bot speed :: " << 0.02*MAX_BOT_SPEED << endl; 
		if(sqrt(ballVel.absSq()) < 0.02*MAX_BOT_SPEED && abs(state.ballPos.y) < OUR_GOAL_MAXY && state.ballPos.x  > (HALF_FIELD_MAXX /*- (2 * DBOX_WIDTH)*/ - (HALF_FIELD_MAXX/5.0) )){	//Invert sign on Side Change
			if(dist >= DRIBBLER_BALL_THRESH){
				sID = SkillSet::GoToPoint;
				sParam.GoToPointP.x = state.ballPos.x;// - SHIFT/2;//state.ballPos.x;													
				sParam.GoToPointP.y = state.ballPos.y;													
			 	sParam.GoToPointP.finalVelocity = 0;													
			 	sParam.GoToPointP.finalslope = atan2( (state.ballPos.y - state.homePos[botID].y) , (state.ballPos.x - state.homePos[botID].x));									
					
				return SkillSet::instance()->executeSkill(sID, sParam, state, botID);	 
			}
			else{
				sID = SkillSet::Kick;
				sParam.KickP.power = 7.0f;							
				return SkillSet::instance()->executeSkill(sID, sParam, state, botID);    	
			}

		}
		
		float default_x = HALF_FIELD_MAXX + SHIFT;														//Invert Signs on Side Change

		if(state.ballPos.x > (HALF_FIELD_MAXX - SHIFT) && state.ballPos.x < HALF_FIELD_MAXX )												//Invert Signs on Side Change
			goalieTarget.x = HALF_FIELD_MAXX - SHIFT;													//Invert Signs on Side Change
		else
			goalieTarget.x = default_x;
		

		float striker = -1 , striker_dist = 99999999999;


		for(int oppID=0;oppID<4;++oppID){
			if (oppID == botID)
				continue;

			Vector2D<int> oppPos (state.homePos[oppID].x, state.homePos[oppID].y);	

			float kick_range_test = sqrt((oppPos - ballPos).absSq());

			if(kick_range_test < KICK_RANGE_THRESH && kick_range_test < striker_dist){
				striker = oppID;
				striker_dist = kick_range_test;
			}
		}

		if(striker != -1){
			goalieTarget.y = ( ((state.ballPos.y - state.homePos[striker].y) / (state.ballPos.x - state.homePos[striker].x)) * (goalieTarget.x - state.ballPos.x) ) + state.ballPos.y;
		}

		else{
			if(state.ballVel.x == 0){
				goalieTarget.y = state.ballPos.y;
			}
			else{
				if(state.ballVel.x > 0)																																							//invert sign	
					goalieTarget.y 	= (( state.ballVel.y / state.ballVel.x ) * ( goalieTarget.x - state.ballPos.x ) ) + state.ballPos.y;
				else
					goalieTarget.y 	= 0;
			}
		}
		
		if(goalieTarget.y< OUR_GOAL_MINY/1.2){
			goalieTarget.y = OUR_GOAL_MINY/1.2;
		}
		else if(goalieTarget.y > OUR_GOAL_MAXY/1.2){
			goalieTarget.y = OUR_GOAL_MAXY/1.2;
		}

		sID = SkillSet::GoToPoint;
		sParam.GoToPointP.x = goalieTarget.x;													
		sParam.GoToPointP.y = goalieTarget.y;													
	 	sParam.GoToPointP.finalVelocity = 0;													
	 	// sParam.GoToPointP.finalslope = 0;									
	 	sParam.GoToPointP.finalslope = atan2( (state.ballPos.y - state.homePos[botID].y) , (state.ballPos.x - state.homePos[botID].x));									
	 	cout<<"in goalieeeee ------------------- moving to : "<<sParam.GoToPointP.x<<","<<sParam.GoToPointP.y<<endl;
	 	return SkillSet::instance()->executeSkill(sID, sParam, state, botID);							


	}
	
	

 	 Tactic::Param TGoalie::paramFromJSON(string json) {
	      using namespace rapidjson;
	      Tactic::Param tParam;
	      
	      return tParam;
	}

    	string TGoalie::paramToJSON(Tactic::Param tParam) {
	      return string("");
	}
	    
}
