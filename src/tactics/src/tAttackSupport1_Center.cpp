#include <list>
#include "tactics/tAttackSupport1_Center.hpp"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <ssl_common/config.h>
#include <iostream>
#include <stdio.h>
#include <ssl_common/geometry.hpp>

#include <fstream>

#define MARGIN_PASS_AHEAD 0
#define SIDE_THRESH 2*DBOX_WIDTH

namespace Strategy{

	TAttackSupport1_Center::TAttackSupport1_Center(int botID):
		Tactic(botID){

		}
	
	TAttackSupport1_Center::~TAttackSupport1_Center(){

	}

	bool TAttackSupport1_Center::isCompleted(const BeliefState &bs,const Tactic::Param& tParam) const{
	    Vector2D<int> botPos(bs.homePos[botID].x, bs.homePos[botID].y);
	    Vector2D<int> ballPos(bs.ballPos.x, bs.ballPos.y);
	    float ballDist = Vector2D<int>::dist(botPos, ballPos);
	    fstream f;
	    f.open("/home/gunjan/catkin_ws/src/play/receive.txt",fstream::out|fstream::app);
	    // f<<"here"<<endl;
	    if(ballDist<1.2*DRIBBLER_BALL_THRESH) f<<"completed"<<endl;
	    f.close();
	    if(ballDist<1.2*DRIBBLER_BALL_THRESH) return true;
	    return false;
	}

	bool TAttackSupport1_Center::isActiveTactic(void)const{
		return false;
	}

	bool TAttackSupport1_Center::isApplicable(const BeliefState &bs, const Param& tParam){

	}

	int TAttackSupport1_Center::chooseBestBot(const BeliefState &state, std::list<int>& freeBots, const Param& tParam, int prevID) const{

		int attacker_ID = tParam.AttackSupportP.id;

		Vector2D<int> attacker(state.homePos[attacker_ID].x , state.homePos[attacker_ID].y);
		Vector2D<int> ballPos(state.ballPos.x , state.ballPos.y);
		int top_half=0,defended_side;

		for(int i=0; i<6; ++i){

			if(state.awayPos[i].x > 0){
				if(state.awayPos[i].y > 0)
					top_half++;
				else if(state.awayPos[i].y < 0)
					top_half--;
			}

		}

		if(top_half > 0)
			defended_side = 1;
		else if(top_half < 0)
			defended_side = -1;
		else
			defended_side = 0;

		int side =  defended_side;
		int on_side_bots = 0, free_bots = 0;

		float target_x , target_y;

		if(state.ballPos.x < (HALF_FIELD_MAXX / 3)){
			target_x = 0.6 * HALF_FIELD_MAXX ;

			target_y = -DBOX_HEIGHT * side;
		}

		else{
			target_x = 0.6 * HALF_FIELD_MAXX + ((0.4 * HALF_FIELD_MAXX) * (state.ballPos.x/HALF_FIELD_MAXX)) ;

			target_y = -DBOX_HEIGHT * side;

			if( fabs(target_y) < DBOX_HEIGHT)
				target_y = DBOX_HEIGHT * side;
			else if( fabs(target_y) > 0.6* HALF_FIELD_MAXY)
				target_y = target_y = 0.6 * HALF_FIELD_MAXY * side;
		}

		Vector2D<int> target(target_x , target_y);

		int test_bot_gen = *(freeBots.begin());
		int test_bot_on_side = *(freeBots.begin());
		float min_distance_from_target = 2 * HALF_FIELD_MAXX;
		float min_distance_from_target_on_side = 2 * HALF_FIELD_MAXX;


		for (std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it){
			Vector2D<int> testPos(state.homePos[*it].x , state.homePos[*it].y);

			if( sqrt((ballPos - testPos).absSq()) > DRIBBLER_BALL_THRESH){

				float distance_from_target = sqrt((testPos - target).absSq());

				if(distance_from_target < min_distance_from_target){
					test_bot_gen = *it;
					min_distance_from_target = distance_from_target;
				}

				if((state.homePos[*it].y * side) > 0){
					++on_side_bots;

					if(distance_from_target < min_distance_from_target_on_side){
						test_bot_on_side = *it;
						min_distance_from_target_on_side = distance_from_target;
					}

				}
			}
			++free_bots;
		}

		if( free_bots - on_side_bots >= 2 || (min_distance_from_target_on_side - min_distance_from_target) <= SIDE_THRESH ){
			return test_bot_gen;
		}

		return test_bot_on_side;
	}



	


	gr_Robot_Command TAttackSupport1_Center::execute(const BeliefState &state, const Tactic::Param& tParam){

		

		Strategy::SkillSet::SkillID sID;
		SkillSet::SParam sParam;

		int attacker_ID = tParam.AttackSupportP.id; 

		Vector2D<int> attacker(state.homePos[attacker_ID].x , state.homePos[attacker_ID].y);
		Vector2D<int> botPos(state.homePos[botID].x , state.homePos[botID].y);
		Vector2D<int> ballPos(state.ballPos.x , state.ballPos.y);


		float target_x , target_y;

		// -----------------------------------------------------------Calculating the heavily defended side---------------------------------------------------

		int top_half=0,defended_side, side;

		for(int i=0; i<6; ++i){

			if(state.awayPos[i].x > 0){
				if(state.awayPos[i].y > 0)
					top_half++;
				else if(state.awayPos[i].y < 0)
					top_half--;
			}

		}

		if(top_half > 0)
			defended_side = 1;
		else if(top_half < 0)
			defended_side = -1;
		else
			defended_side = 0;

		//----------------------------------------------------------------------------------------------------------------------------------------------------------

		
		// ----------------------------------------------------------------Region Definition------------------------------------------------------------------------------

		int region = 1;

		if( ballPos.x <= HALF_FIELD_MAXX/2  && fabs(ballPos.y) <= HALF_FIELD_MAXY/2 ) {
			region = 1;
		}
		if( ballPos.x <= 0) {
			region = 1;
		}
		if( ballPos.x >= HALF_FIELD_MAXX/2 && fabs(ballPos.y) <= HALF_FIELD_MAXY/2 ) {
			region = 2;
		}
		if( ballPos.x >= HALF_FIELD_MAXX/2 && fabs(ballPos.y) >= HALF_FIELD_MAXY/2 ) {
			region = 3;
		}

		//---------------------------------------------------------------------------For Region 1--------------------------------------------------------------------------------------

		if(region = 1){

			int side =  defended_side;


			if(state.ballPos.x < 0){
				target_x = 0.6 * HALF_FIELD_MAXX ;

				target_y = DBOX_HEIGHT * side;
			}

			else{
				target_x = 0.6 * HALF_FIELD_MAXX + ((0.4 * HALF_FIELD_MAXX) * (state.ballPos.x/HALF_FIELD_MAXX)) ;

				target_y = DBOX_HEIGHT * side;

				if( fabs(target_y) < DBOX_HEIGHT)
					target_y = DBOX_HEIGHT * side;
				else if( fabs(target_y) > 0.6* HALF_FIELD_MAXY)
					target_y = target_y = 0.6 * HALF_FIELD_MAXY * side;
			}

			Vector2D<int> target( target_x , target_y);
			float theta = normalizeAngle(Vector2D<int>::angle(ballPos , target)) , our_distance_from_line = 999999, opp_distance_from_line = 999999;
			
			for( int id=0; id<6; id++){
				Vector2D <int> opp_test (state.awayPos[id].x , state.awayPos[id].y);
				Vector2D <int> home_test (state.homePos[id].x , state.homePos[id].y);

				opp_distance_from_line = fabs(((target_y - state.ballPos.y) * (state.awayPos[id].x)) - ((target_x - state.ballPos.x) * (state.awayPos[id].y)) + (target_x* state.ballPos.y) - (target_y * state.ballPos.x)) / sqrt((botPos - ballPos).absSq());
				if( (ballPos - opp_test).absSq() > (ballPos - target).absSq() || (botPos - opp_test).absSq() > (ballPos - target).absSq() )
					opp_distance_from_line = 999999;
				our_distance_from_line = fabs(((target_y - state.ballPos.y) * (state.homePos[id].x)) - ((target_x - state.ballPos.x) * (state.homePos[id].y)) + (target_x* state.ballPos.y) - (target_y * state.ballPos.x)) / sqrt((botPos - ballPos).absSq());
				if( (ballPos - home_test).absSq() > (ballPos - target).absSq() || (botPos - home_test).absSq() > (ballPos - target).absSq() || id == attacker_ID || id == botID)
					our_distance_from_line = 999999;
				if ( opp_distance_from_line < 150 || our_distance_from_line < 100){

					//target_x -= 2 * BOT_RADIUS;
					target_y -= 5 * BOT_RADIUS * side;

					sID = SkillSet::GoToPoint;
					sParam.GoToPointP.x = target_x;
					sParam.GoToPointP.y = target_y;
					sParam.GoToPointP.finalVelocity = 0;

					Vector2D<int> final_target( target_x , target_y);
					theta = normalizeAngle(Vector2D<int>::angle(ballPos , final_target));

					sParam.GoToPointP.finalslope = theta ;

					return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
				}
			}

			if(fabs(ballPos.x) > (HALF_FIELD_MAXX - (1.5 * DBOX_WIDTH) ))
				target_x = HALF_FIELD_MAXX - (1.5 * DBOX_WIDTH);
			else
				target_x = HALF_FIELD_MAXX - DBOX_WIDTH / 1.5;


			sID = SkillSet::GoToPoint;

			sParam.GoToPointP.x = target_x;

			sParam.GoToPointP.y = target_y;

			sParam.GoToPointP.finalVelocity = 0;

			Vector2D<int> final_target( target_x , target_y);
			theta = normalizeAngle(Vector2D<int>::angle(ballPos , final_target));
			sParam.GoToPointP.finalslope = theta ;

			return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
		}

		//-----------------------------------------------------------------------------------------------------------------------------------------------------------------


		//-------------------------------------------------------------------------For Region 2-----------------------------------------------------------------------

		if (region = 2)
		{				
			if(ballPos.y > 0)
				side = -1;
			else
				side = 1;

			target_y = side * DBOX_HEIGHT/1.5;
			target_x = ballPos.x + (HALF_FIELD_MAXX - ballPos.x)/ 2.5 ; 


			Vector2D<int> target( target_x , target_y);
			float theta = normalizeAngle(Vector2D<int>::angle(ballPos , target)) , our_distance_from_line = 999999, opp_distance_from_line = 999999;
			
			for( int id=0; id<6; id++){
				Vector2D <int> opp_test (state.awayPos[id].x , state.awayPos[id].y);
				Vector2D <int> home_test (state.homePos[id].x , state.homePos[id].y);

				opp_distance_from_line = fabs(((target_y - state.ballPos.y) * (state.awayPos[id].x)) - ((target_x - state.ballPos.x) * (state.awayPos[id].y)) + (target_x* state.ballPos.y) - (target_y * state.ballPos.x)) / sqrt((botPos - ballPos).absSq());
				if( (ballPos - opp_test).absSq() > (ballPos - target).absSq() || (botPos - opp_test).absSq() > (ballPos - target).absSq() )
					opp_distance_from_line = 999999;
				our_distance_from_line = fabs(((target_y - state.ballPos.y) * (state.homePos[id].x)) - ((target_x - state.ballPos.x) * (state.homePos[id].y)) + (target_x* state.ballPos.y) - (target_y * state.ballPos.x)) / sqrt((botPos - ballPos).absSq());
				if( (ballPos - home_test).absSq() > (ballPos - target).absSq() || (botPos - home_test).absSq() > (ballPos - target).absSq() || id == attacker_ID || id == botID)
					our_distance_from_line = 999999;
				if ( opp_distance_from_line < 150 || our_distance_from_line < 100){

					target_x -= 5 * BOT_RADIUS;
					target_y += 3 * BOT_RADIUS * side;

					if(fabs(ballPos.x) > (HALF_FIELD_MAXX - (1.5 * DBOX_WIDTH) ))
						target_x = HALF_FIELD_MAXX - (1.5 * DBOX_WIDTH);
					else
						target_x = HALF_FIELD_MAXX - DBOX_WIDTH / 1.5;

					sID = SkillSet::GoToPoint;
					sParam.GoToPointP.x = target_x;
					sParam.GoToPointP.y = target_y;
					sParam.GoToPointP.finalVelocity = 0;

					Vector2D<int> final_target( target_x , target_y);
					theta = normalizeAngle(Vector2D<int>::angle(ballPos , final_target));

					sParam.GoToPointP.finalslope = theta ;

					return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
				}
			}

			if(fabs(ballPos.x) > (HALF_FIELD_MAXX - (1.5 * DBOX_WIDTH) ))
				target_x = HALF_FIELD_MAXX - (1.5 * DBOX_WIDTH);
			else
				target_x = HALF_FIELD_MAXX - DBOX_WIDTH / 1.5;


			sID = SkillSet::GoToPoint;

			sParam.GoToPointP.x = target_x;

			sParam.GoToPointP.y = target_y;

			sParam.GoToPointP.finalVelocity = 0;

			Vector2D<int> final_target( target_x , target_y);
			theta = normalizeAngle(Vector2D<int>::angle(ballPos , final_target));
			sParam.GoToPointP.finalslope = theta ;

			return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
		}

		//-----------------------------------------------------------------------------------------------------------------------------------------------------------------


		//-------------------------------------------------------------------------For Region 3-----------------------------------------------------------------------

		if (region = 3)
		{
			if(ballPos.y > 0)
				side = 1;
			else
				side = -1;

			if(fabs(ballPos.y) > DBOX_HEIGHT/1.3){

				target_x = HALF_FIELD_MAXX - (2 * DBOX_WIDTH);

				target_y = 0;
			}

			else{
				target_x = HALF_FIELD_MAXX - DBOX_WIDTH;

				target_y = DBOX_HEIGHT * side;
			}


			Vector2D<int> target( target_x , target_y);
			float theta = normalizeAngle(Vector2D<int>::angle(ballPos , target)) , our_distance_from_line = 999999, opp_distance_from_line = 999999;
			
			for( int id=0; id<6; id++){
				Vector2D <int> opp_test (state.awayPos[id].x , state.awayPos[id].y);
				Vector2D <int> home_test (state.homePos[id].x , state.homePos[id].y);

				opp_distance_from_line = fabs(((target_y - state.ballPos.y) * (state.awayPos[id].x)) - ((target_x - state.ballPos.x) * (state.awayPos[id].y)) + (target_x* state.ballPos.y) - (target_y * state.ballPos.x)) / sqrt((botPos - ballPos).absSq());
				if( (ballPos - opp_test).absSq() > (ballPos - target).absSq() || (botPos - opp_test).absSq() > (ballPos - target).absSq() )
					opp_distance_from_line = 999999;
				our_distance_from_line = fabs(((target_y - state.ballPos.y) * (state.homePos[id].x)) - ((target_x - state.ballPos.x) * (state.homePos[id].y)) + (target_x* state.ballPos.y) - (target_y * state.ballPos.x)) / sqrt((botPos - ballPos).absSq());
				if( (ballPos - home_test).absSq() > (ballPos - target).absSq() || (botPos - home_test).absSq() > (ballPos - target).absSq() || id == attacker_ID || id == botID)
					our_distance_from_line = 999999;
				if ( opp_distance_from_line < 140 || our_distance_from_line < 100){

					target_x += 4 * BOT_RADIUS;
					target_y -= 4 * BOT_RADIUS * side;

					if(target_x >= HALF_FIELD_MAXX)
						target_x -= 8 * BOT_RADIUS;
				
					if(fabs(ballPos.x) > (HALF_FIELD_MAXX - (1.5 * DBOX_WIDTH) ))
						target_x = HALF_FIELD_MAXX - (1.5 * DBOX_WIDTH);
					else
						target_x = HALF_FIELD_MAXX - DBOX_WIDTH / 1.5;


					sID = SkillSet::GoToPoint;
					sParam.GoToPointP.x = target_x;
					sParam.GoToPointP.y = target_y;
					sParam.GoToPointP.finalVelocity = 0;

					Vector2D<int> final_target( target_x , target_y);
					float angleToTurn = normalizeAngle(Vector2D<int>::angle(ballPos , final_target));

					sParam.GoToPointP.finalslope = angleToTurn ;

					return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
				}
			}

			if(fabs(ballPos.x) > (HALF_FIELD_MAXX - (1.5 * DBOX_WIDTH) ))
				target_x = HALF_FIELD_MAXX - (1.5 * DBOX_WIDTH);
			else
				target_x = HALF_FIELD_MAXX - DBOX_WIDTH / 1.5;


			sID = SkillSet::GoToPoint;

			if(target_x > (HALF_FIELD_MAXX - DBOX_WIDTH/3))
				target_x = (HALF_FIELD_MAXX - DBOX_WIDTH/3);

			sParam.GoToPointP.x = target_x;

			sParam.GoToPointP.y = target_y;

			sParam.GoToPointP.finalVelocity = 0;

			Vector2D<int> final_target( target_x , target_y);
			theta = normalizeAngle(Vector2D<int>::angle(ballPos , final_target));
			sParam.GoToPointP.finalslope = theta ;

			return SkillSet::instance()->executeSkill(sID, sParam, state, botID);

		}
		//-----------------------------------------------------------------------------------------------------------------------------------------------------------------

	}

	

 	Tactic::Param TAttackSupport1_Center::paramFromJSON(string json) {
	      using namespace rapidjson;
	      Tactic::Param tParam;
	      Document d;
	      d.Parse(json.c_str());
	      tParam.AttackSupportP.id = d["id"].GetInt();
	      return tParam;
	}

    	string TAttackSupport1_Center::paramToJSON(Tactic::Param tParam) {
	      using namespace rapidjson;
	      StringBuffer buffer;
	      Writer <StringBuffer> w(buffer);
	      w.StartObject();
	      w.String("id");
	      w.Int(tParam.AttackSupportP.id);
	      w.EndObject();
	      return buffer.GetString();
	}
}