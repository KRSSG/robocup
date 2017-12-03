#include <list>
#include "tactics/tMark.hpp"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <ssl_common/config.h>
#include <iostream>
#include <stdio.h>
#include <ssl_common/geometry.hpp>

#include <fstream>

#define KICK_RANGE_THRESH MAX_DRIBBLE_R
#define THRES (0.8f)
#define THETA_THRESH 0.005
#define TURNING_THRESH 10

namespace Strategy{

	TMark::TMark(int botID):
		Tactic(botID){

		}
	
	TMark::~TMark(){

	}

	bool TMark::isCompleted(const BeliefState &bs,const Tactic::Param& tParam) const{
		//add logic
		return false;
	}
	bool TMark::isActiveTactic(void)const{
		return false;
	}

	int TMark::chooseBestBot(const BeliefState &state, std::list<int>& freeBots, const Param& tParam, int prevID) const{
		std::vector<int> away_bots_on_our_goalie_side;
		Vector2D<float> bot;
		int best_bot = -1;
		float min_dis = 999999.9f;

		for(int i = 0; i < 6; ++i){
			if(state.awayPos[i].x < 0){
				away_bots_on_our_goalie_side.push_back(i);
			}
		}

		for(std::list<int>::const_iterator itr = freeBots.begin(); itr != freeBots.end(); ++itr){
			for(int i = 0; i < away_bots_on_our_goalie_side.size(); ++i){

				Vector2D<int> awayBotPos(state.awayPos[away_bots_on_our_goalie_side[i]].x,state.awayPos[away_bots_on_our_goalie_side[i]].y);
				Vector2D<int> homeBotPos(state.homePos[*itr].x, state.homePos[*itr].y);
				if(Vector2D<int>::dist(awayBotPos,homeBotPos) < min_dis){
					best_bot = *itr;
					//min_dis = Vector2D<float>::dist(away_bot, homeBotPos);
				}
			}
		}
		assert(best_bot != -1);
		return best_bot;
	}

	gr_Robot_Command TMark::execute(const BeliefState &state, const Tactic::Param& tParam){




		Vector2D<int> ballAim, marked, guardPos;
		Vector2D<int> botpos (state.homePos[botID].x, state.homePos[botID].y);
		Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
		Vector2D<int> markedPos (state.homePos[tParam.MarkBotP.awayBotID].x, state.homePos[tParam.MarkBotP.awayBotID].y);
		Vector2D<int> ballVel(state.ballVel.x , state.ballVel.y);
		
		Strategy::SkillSet::SkillID sID;
		SkillSet::SParam sParam;
	
		
	
		float passer = -1 , passer_dist = 99999999999;


		for(int oppID=0;oppID<4;++oppID){
			Vector2D<int> oppPos (state.homePos[oppID].x, state.homePos[oppID].y);
			float kick_range_test = (oppPos - ballPos).absSq();

			if(kick_range_test < KICK_RANGE_THRESH && kick_range_test < passer_dist){
				passer = oppID;
				passer_dist = kick_range_test;
			}
		}

		if(passer != -1){
			if( abs(ballPos.x - markedPos.x) > abs(ballPos.y - markedPos.y)){
				if(ballPos.x > markedPos.x)
					guardPos.x = markedPos.x + DBOX_WIDTH/4;
				else
					guardPos.x = markedPos.x - DBOX_WIDTH/4;
				
				guardPos.y = ( ((state.ballPos.y - state.awayPos[passer].y) / (state.ballPos.x - state.awayPos[passer].x)) * (guardPos.x - state.ballPos.x) ) + state.ballPos.y;
			}
			else{
				if(ballPos.y > markedPos.y)
					guardPos.y = markedPos.y + DBOX_WIDTH/4;
				else
					guardPos.y = markedPos.y - DBOX_WIDTH/4;
				
				guardPos.x = ( ((state.ballPos.x - state.awayPos[passer].x) / (state.ballPos.y - state.awayPos[passer].y)) * (guardPos.y - state.ballPos.y) ) + state.ballPos.x;
			}
		}

		else{
			if( abs(ballPos.x - markedPos.x) > abs(ballPos.y - markedPos.y)){
				if(ballPos.x > markedPos.x)
					guardPos.x = markedPos.x + DBOX_WIDTH/4;
				else
					guardPos.x = markedPos.x - DBOX_WIDTH/4;
				
				guardPos.y = ( ((markedPos.y - state.ballPos.y) / (markedPos.x - state.ballPos.x)) * (guardPos.x - state.ballPos.x) ) + state.ballPos.y;
			}
			else{
				if(ballPos.y > markedPos.y)
					guardPos.y = markedPos.y + DBOX_WIDTH/4;
				else
					guardPos.y = markedPos.y - DBOX_WIDTH/4;
				
				guardPos.x = ( ((markedPos.x - state.ballPos.x) / (markedPos.y - state.ballPos.y)) * (guardPos.y - state.ballPos.y) ) + state.ballPos.x;
			}
		}
		

		sID = SkillSet::GoToPoint;
		sParam.GoToPointP.x = guardPos.x;

		sParam.GoToPointP.y = guardPos.y;
	 	
	 	sParam.GoToPointP.finalVelocity = 0;	
	 	
	 	float angleToTurn = normalizeAngle(Vector2D<int>::angle(ballPos , guardPos));									
	 	sParam.GoToPointP.finalslope = angleToTurn ;
	 	
	 																	

	 	return SkillSet::instance()->executeSkill(sID, sParam, state, botID);							


	}
	


 	 Tactic::Param TMark::paramFromJSON(string json) {
	      using namespace rapidjson;
	      Tactic::Param tParam;
	      Document d;
	      d.Parse(json.c_str());
	      tParam.MarkBotP.awayBotID = d["id"].GetInt();
	      return tParam;
	}

    	string TMark::paramToJSON(Tactic::Param tParam) {
	      using namespace rapidjson;
	      StringBuffer buffer;
	      Writer <StringBuffer> w(buffer);
	      w.StartObject();
	      w.String("id");
	      w.Int(tParam.MarkBotP.awayBotID);
	      w.EndObject();
	      return buffer.GetString();
	}
	    
}
