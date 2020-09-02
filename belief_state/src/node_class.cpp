#include "ros/ros.h"
#include "../../ssl_common/include/ssl_common/config.h"
#include <krssg_ssl_msgs/BeliefState.h>
#include <krssg_ssl_msgs/SSL_DetectionFrame.h>
#include <krssg_ssl_msgs/SSL_DetectionRobot.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include "../../ssl_common/include/ssl_common/geometry.hpp"

#include <bits/stdc++.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <queue>
#include <deque>
#include <fstream>
#include <Eigen/Dense>
#include "kalman_filter.h"

//const int BALL_AT_CORNER_THRESH	= 20; 
const int HALF_FIELD_MAXX		= 6000; 
const int HALF_FIELD_MAXY		= 4500;
const float MAX_DRIBBLE_R		= 3;
const int DBOX_WIDTH			= 2400;
const int DBOX_HEIGHT			= 1200;
const int MAX_QUEUE_SZ			= 5;
short int isteamyellow = 0;

using namespace std;
using geometry_msgs::Pose2D;
using geometry_msgs::Point32;

ros::Publisher pub;

void print(Pose2D& p){
	cout<<"x: "<<p.x<<" y: "<<p.y<<endl;
}

void print(vector<Pose2D> p){
	for(int i = 0; i < p.size(); i++)
	{
		cout<<"i: \t";
		print(p[i]);
	}
}

void print(vector<bool> p){
	for(int i=0;i<p.size();i++)
	{
		cout<<"i: \t"<<p[i]<<endl;
	}
}

class BeliefState {

	//Time
	ros::Time time;

	//Technical
	bool isteamyellow;
	int frame_number;
	double t_capture, t_sent;

	//Positions
	Pose2D ballPos;
	vector<Pose2D> awayPos, homePos;

	//Velocities
	Pose2D ballVel;
	vector<Pose2D> awayVel, homeVel;

	//Detections
	bool ballDetected;
	vector<bool> awayDetected, homeDetected;

	//Predictions
	short int 	//our_bot_closest_to_ball,
				//opp_bot_closest_to_ball,
				our_goalie,
				opp_goalie;
				//opp_bot_marking_our_attacker;

	//bool ball_in_our_half; //ball_at_corners, ball_in_our_possession;

public:
	BeliefState();
	BeliefState(const krssg_ssl_msgs::SSL_DetectionFrame::ConstPtr& , list<BeliefState> &, KalmanFilter *home[], KalmanFilter *away[], KalmanFilter *ball);
	BeliefState(const BeliefState &);
	void copy(const BeliefState&);
	void initialise();
	void print();
	krssg_ssl_msgs::BeliefState getBeliefStateMsg();

};

bool isValidMsg(const krssg_ssl_msgs::SSL_DetectionFrame::ConstPtr& vmsg){
	// if(vmsg->robots_yellow.size() == 0 && vmsg->robots_blue.size()==0)
	// 	return false;
	// if(vmsg->balls.size() == 0)
	// 	return false;
	return true;
}

BeliefState::BeliefState(){
	this->initialise();
}

BeliefState::BeliefState(const BeliefState &msg){
	this->copy(msg);
}

void BeliefState::copy(const BeliefState& bs){
	this->isteamyellow = bs.isteamyellow;
	this->frame_number = bs.frame_number;
	this->t_capture = bs.t_capture;
	this->time = ros::Time::now();
	this->t_sent = bs.t_sent;
	this->ballPos = bs.ballPos;
	this->awayPos = bs.awayPos;
	this->homePos = bs.homePos;
	this->ballVel = bs.ballVel;
	this->awayVel = bs.awayVel;
	this->homeVel = bs.homeVel;
	this->ballDetected = bs.ballDetected;
	this->homeDetected = bs.homeDetected;
	this->awayDetected = bs.awayDetected;
	//this->our_bot_closest_to_ball = bs.our_bot_closest_to_ball;
	//this->opp_bot_closest_to_ball = bs.opp_bot_closest_to_ball;
	this->our_goalie = bs.our_goalie;
	this->opp_goalie = bs.opp_goalie;
	//this->opp_bot_marking_our_attacker = bs.opp_bot_marking_our_attacker;
	//this->ball_at_corners = bs.ball_at_corners;
	//this->ball_in_our_half = bs.ball_in_our_half;
	//this->ball_in_our_possession = bs.ball_in_our_possession;
}

float distFn(krssg_ssl_msgs::SSL_DetectionRobot p, float x, float y){
	return sqrt((p.x - x)*(p.x - x) + (p.y - y)*(p.y - y));
}

BeliefState::BeliefState(const krssg_ssl_msgs::SSL_DetectionFrame::ConstPtr& vmsg, list<BeliefState> &prev_msgQ, KalmanFilter *home[], KalmanFilter *away[], KalmanFilter *ball){
	if(!isValidMsg(vmsg)){
		if(prev_msgQ.size())
			copy(prev_msgQ.back());
		else{
			this->initialise();
		}
	}
	else{
		if(prev_msgQ.size())
			copy(prev_msgQ.back());
		else this->initialise();
		this->time = ros::Time::now();
		this->frame_number = vmsg->frame_number;
		// this->t_capture = vmsg->t_capture;
		// this->t_sent = vmsg->t_sent;
		this->isteamyellow = ::isteamyellow;

		vector<krssg_ssl_msgs::SSL_DetectionRobot> homePos, awayPos;
		if(::isteamyellow){
			homePos = vmsg->robots_yellow;
			awayPos = vmsg->robots_blue;
		}
		else{
			homePos = vmsg->robots_blue;
			awayPos = vmsg->robots_yellow;
		}

		// Savitzky-Golay Filter
		double FILTER[MAX_QUEUE_SZ] = {0.086, -0.143, -0.086, 0.257, 0.886};
		//double FILTER[MAX_QUEUE_SZ] = {0, -0, -0, 0, 1};

		if(vmsg->balls.size()){
			this->ballDetected = 1;
			/*this->ballPos.x = vmsg->balls[0].x;
			this->ballPos.y = vmsg->balls[0].y;

			//FILTERING BALLPOS
			if(prev_msgQ.size() == MAX_QUEUE_SZ){
				this->ballPos.x *= FILTER[MAX_QUEUE_SZ - 1];
				this->ballPos.y *= FILTER[MAX_QUEUE_SZ - 1];
				int count = -1;
				for(list<BeliefState>::iterator it = prev_msgQ.begin(); it != prev_msgQ.end(); it++){
					if(it != prev_msgQ.begin()&&count!=4){
						this->ballPos.x += it->ballPos.x*FILTER[count];
						this->ballPos.y += it->ballPos.y*FILTER[count];
					}
					count ++;
				}
			}*/
			Eigen::VectorXd x(4);
			if(!ball->intialized()){
				Eigen::MatrixXd P(4, 4);
				x << vmsg->balls[0].x, 0, vmsg->balls[0].y, 0;
				P.setIdentity();
				P = P*200;
				ball->init(x, P);
			}
			else{
				Eigen::VectorXd z(2);
				z << vmsg->balls[0].x, vmsg->balls[0].y;
				ball->predict();
				x = ball->update(z);
			}
			// cout << ball.x_size << "\n" << ball.z_size << "\n" << ball.F <<
			// "\n" << ball.Q << "\n" << ball.H << "\n" << ball.R << "\n" << ball.x
			// << "\n" << ball.x_prior << "\n" << ball.x_post << "\n" << ball.P <<
			// "\n" << ball.P_prior << "\n" << ball.P_post << "\n";
			this->ballPos.x = x(0);
			this->ballVel.x = x(1);
			this->ballPos.y = x(2);
			this->ballVel.y = x(3);
			// if(this->ballPos.x <= 0)
			// 	this->ball_in_our_half = true;
			// else this->ball_in_our_half = false;
			/*if(	fabs(this->ballPos.x) > (HALF_FIELD_MAXX - BALL_AT_CORNER_THRESH) && 
				fabs(this->ballPos.y) > (HALF_FIELD_MAXY - BALL_AT_CORNER_THRESH)	)
				this->ball_at_corners = true;
			else this->ball_at_corners = false;*/

		}
		else this->ballDetected = 0;
		
		float tempGoalie=7, tempDist=9999999, distance_from_ball = 99999999;
		for(int i=0;i<homePos.size();i++){
			int bot_id = homePos[i].robot_id;
    		this->homeDetected[bot_id] = 1;
    		this->homePos[bot_id].theta = homePos[i].orientation;
    		Eigen::VectorXd x(4);
    		if(!home[bot_id]->intialized()){
    			Eigen::MatrixXd P(4, 4);
    			x << homePos[i].x, 0, homePos[i].y, 0;
    			P.setIdentity();
    			P = P*200;
    			home[bot_id]->init(x, P);
    		}
    		else{
    			Eigen::VectorXd z(2);
    			z << homePos[i].x, homePos[i].y;
    			home[bot_id]->predict();
    			x = home[bot_id]->update(z);
    		}

    		this->homePos[bot_id].x = x(0);
    		this->homeVel[bot_id].x = x(1);
    		this->homePos[bot_id].y = x(2);
    		this->homeVel[bot_id].y = x(3);
    		/*this->homePos[bot_id].x = homePos[i].x;
    		this->homePos[bot_id].y = homePos[i].y;

			//HOMEPOS FILTERING
			if(prev_msgQ.size() == MAX_QUEUE_SZ){
			this->homePos[i].x *= FILTER[MAX_QUEUE_SZ - 1];
			this->homePos[i].y *= FILTER[MAX_QUEUE_SZ - 1];
			int count = -1;
				for(list<BeliefState>::iterator it = prev_msgQ.begin(); it != prev_msgQ.end(); it++){
					if(it != prev_msgQ.begin()){
						this->homePos[i].x += it->homePos[i].x*FILTER[count];
						this->homePos[i].y += it->homePos[i].y*FILTER[count];
					}
					count ++;
				}
			}*/

			// float dist = sqrt(pow((homePos[i].x - this->ballPos.x),2) + pow((homePos[i].y - this->ballPos.y) , 2));
   // 			if(dist < distance_from_ball){
   //    			distance_from_ball = dist;
   //   	 		this->our_bot_closest_to_ball = i;
	      		//if(distance_from_ball < MAX_DRIBBLE_R) this->ball_in_our_possession = true;
    	}
    		

    			/* #######################################  */

    			// Goalie id in referee msg

    			// float thisDist = distFn(homePos[i],(-HALF_FIELD_MAXX + DBOX_WIDTH),DBOX_HEIGHT) +
    			// 		distFn(homePos[i],(-HALF_FIELD_MAXX + DBOX_WIDTH),-DBOX_HEIGHT);
	    		// if(thisDist < tempDist){
	    		// 	tempDist = thisDist;
	    		// 	tempGoalie = bot_id;
	    		// }

    			/* #######################################  */
    	
	
		// goalie id in refereee msg
    	// this->our_goalie = tempGoalie;

    	// tempGoalie = 7;
    	// distance_from_ball = 9999999;
    	// tempDist = 9999999;

    	for(int i=0;i<awayPos.size();i++){
    		int bot_id = awayPos[i].robot_id;
    		this->awayDetected[bot_id] = 1;
    		this->awayPos[bot_id].theta = awayPos[i].orientation;
    		Eigen::VectorXd x(4);
    		if(!away[bot_id]->intialized()){
    			Eigen::MatrixXd P(4, 4);
    			x << awayPos[i].x, 0, awayPos[i].y, 0;
    			P.setIdentity();
    			P = P*200;
    			away[bot_id]->init(x, P);
    		}
    		else{
    			Eigen::VectorXd z(2);
    			z << awayPos[i].x, awayPos[i].y;
    			away[bot_id]->predict();
    			x = away[bot_id]->update(z);
    		}

    		this->awayPos[bot_id].x = x(0);
    		this->awayVel[bot_id].x = x(1);
    		this->awayPos[bot_id].y = x(2);
    		this->awayVel[bot_id].y = x(3);
    		/*int bot_id = awayPos[i].robot_id;
    		this->awayDetected[bot_id] = 1;
    		this->awayPos[bot_id].x = awayPos[i].x;
    		this->awayPos[bot_id].y = awayPos[i].y;
    		this->awayPos[bot_id].theta = awayPos[i].orientation;

    		//AWAYPOS FILTERING
    		if(prev_msgQ.size() == MAX_QUEUE_SZ){
				this->awayPos[i].x *= FILTER[MAX_QUEUE_SZ - 1];
				this->awayPos[i].y *= FILTER[MAX_QUEUE_SZ - 1];
				int count = -1;
				for(list<BeliefState>::iterator it = prev_msgQ.begin(); it != prev_msgQ.end(); it++){
					if(it != prev_msgQ.begin()){
						this->awayPos[i].x += it->awayPos[i].x*FILTER[count];
						this->awayPos[i].y += it->awayPos[i].y*FILTER[count];
					}
					count ++;
				}
			}*/

    //		float dist = sqrt(((awayPos[i].x - this->ballPos.x)*(awayPos[i].x - this->ballPos.x)) + ((awayPos[i].y - this->ballPos.y)*(awayPos[i].y - this->ballPos.y)));

   //  		if(dist < distance_from_ball){
   //    			distance_from_ball = dist;
   //    			this->opp_bot_closest_to_ball = bot_id;
			// }

			/*
			###############################
			Goalie id referee msg
   			float thisDist = distFn(awayPos[i],(HALF_FIELD_MAXX - DBOX_WIDTH),DBOX_HEIGHT) + distFn(awayPos[i],(HALF_FIELD_MAXX - DBOX_WIDTH),-DBOX_HEIGHT);
    		if(thisDist < tempDist){
    			tempDist = thisDist;
    			tempGoalie = bot_id;
    		}
    		*/
    	}

	    // this->opp_goalie = tempGoalie;

	    //VELOCITY CALCULATIONS

    	/*
		
		TODO: improve velocity calculation
    	BeliefState prevState;
    	if(prev_msgQ.size())
    		prevState = prev_msgQ.front();
    	assert(prevState.homePos.size() == 6);
    	assert(prevState.awayPos.size() == 6);
    	this->ballVel.x = (this->ballPos.x - prevState.ballPos.x)/(this->time - prevState.time).toSec();
    	this->ballVel.y = (this->ballPos.y - prevState.ballPos.y)/(this->time - prevState.time).toSec();
    	if(prev_msgQ.size() == MAX_QUEUE_SZ){
			this->ballVel.x *= FILTER[MAX_QUEUE_SZ - 1];
			this->ballVel.y *= FILTER[MAX_QUEUE_SZ - 1];
			int count = -1;
			for(list<BeliefState>::iterator it = prev_msgQ.begin(); it != prev_msgQ.end(); it++){
				if(it != prev_msgQ.begin()){
					this->ballVel.x += it->ballVel.x*FILTER[count];
					this->ballVel.y += it->ballVel.y*FILTER[count];
				}
				count ++;
			}
		}
    	for(int i=0;i<this->homePos.size();i++)
    	{
    		this->homeVel[i].x = (this->homePos[i].x - prevState.homePos[i].x)/(this->time - prevState.time).toSec();
    		this->homeVel[i].y = (this->homePos[i].y - prevState.homePos[i].y)/(this->time - prevState.time).toSec();
    	}
		for(int i=0;i<this->awayPos.size();i++)
    	{
    		this->awayVel[i].x = (this->awayPos[i].x - prevState.awayPos[i].x)/(this->time - prevState.time).toSec();
    		this->awayVel[i].y = (this->awayPos[i].y - prevState.awayPos[i].y)/(this->time - prevState.time).toSec();
    	}
    	*/

    	//UPDATING LIST
		if(prev_msgQ.size() == MAX_QUEUE_SZ){
			prev_msgQ.pop_front();
			prev_msgQ.push_back(*this);
			assert(prev_msgQ.size()==MAX_QUEUE_SZ);
		}
		else
			prev_msgQ.push_back(*this);

	}
}

void BeliefState::initialise(){

	this->time = ros::Time::now();
	this->isteamyellow = ::isteamyellow;
	this->frame_number = 0;
	this->t_capture = 0.0;
	this->t_sent = 0.0;

	this->ballPos = Pose2D();
	this->awayPos = vector<Pose2D>(6,Pose2D());
	this->homePos = vector<Pose2D>(6,Pose2D());

	this->ballVel = Pose2D();
	this->awayVel = vector<Pose2D>(6,Pose2D());
	this->homeVel = vector<Pose2D>(6,Pose2D());

	this->ballDetected = false;
	this->homeDetected = vector<bool>(6,0);
	this->awayDetected = vector<bool>(6,0);
	for( int i = 0; i < 6; i++ )
	{
		this->awayPos[i].x = this->awayPos[i].y = 3000;
		this->homePos[i].x = this->homePos[i].y = 3500;
	}

	//this->our_bot_closest_to_ball = this->opp_bot_closest_to_ball = 
	//this->opp_bot_marking_our_attacker = -1;

	//this->ball_at_corners = false;
	//this->ball_in_our_half = false;
	//this->ball_in_our_possession = false;
}

void BeliefState::print(){
	system("clear");
	cout<<"time: "<<this->time<<endl;
	cout<<"isteamyellow: "<<isteamyellow<<endl;
	cout<<"frame_number: "<<frame_number<<endl;
	cout<<"t_capture: "<<t_capture<<" t_sent: "<<t_sent<<endl;
	cout<<"ballPos:"<<endl;
	::print(ballPos);
	cout<<"awayPos:"<<endl;
	::print(awayPos);
	cout<<"homePos:"<<endl;
	::print(homePos);
	cout<<"ballVel:"<<endl;
	::print(ballVel);
	cout<<"awayVel:"<<endl;
	::print(awayVel);
	cout<<"homeVel:"<<endl;
	::print(homeVel);
	cout<<"ballDetected: "<<ballDetected;
	cout<<"homeDetected:"<<endl;
	::print(homeDetected);
	cout<<"awayDetected:"<<endl;
	::print(awayDetected);
	cout<<//"our_bot_closest_to_ball: "<<our_bot_closest_to_ball<<
	//" opp_bot_closest_to_ball: "<<opp_bot_closest_to_ball<<
	" \n our_goalie: "<<our_goalie<<" opp_goalie: "<<opp_goalie<<endl;
	//" opp_bot_marking_our_attacker: "<<opp_bot_marking_our_attacker<<
	//"\n ball_at_corners: "<<ball_at_corners<<
	//"\n ball_in_our_half: "<<ball_in_our_half<<endl;
	//"\n ball_in_our_possession: "<<ball_in_our_possession<<endl;
}

krssg_ssl_msgs::BeliefState BeliefState::getBeliefStateMsg(){
	krssg_ssl_msgs::BeliefState msg;
	msg.isteamyellow = this->isteamyellow;
	msg.frame_number = this->frame_number;
	msg.t_capture = this->t_capture;
	msg.ballPos.x = this->ballPos.x;
	msg.ballPos.y = this->ballPos.y;	
	msg.awayPos = this->awayPos;
	msg.homePos = this->homePos;
	msg.ballVel.x = this->ballVel.x;
	msg.ballVel.y = this->ballVel.y;
	msg.awayVel = this->awayVel;
	msg.homeVel = this->homeVel;
	msg.ballDetected = this->ballDetected;
	for(int i=0;i<this->homeDetected.size();i++)
		msg.homeDetected.push_back(this->homeDetected[i]);
	for(int i=0;i<this->awayDetected.size();i++)
		msg.awayDetected.push_back(this->awayDetected[i]);
	//msg.our_bot_closest_to_ball = this->our_bot_closest_to_ball;
	//msg.opp_bot_closest_to_ball = this->opp_bot_closest_to_ball;
	msg.our_goalie = this->our_goalie;
	msg.opp_goalie = this->opp_goalie;
	//msg.opp_bot_marking_our_attacker = this->opp_bot_marking_our_attacker;
	//msg.ball_at_corners = this->ball_at_corners;
	//msg.ball_in_our_half = this->ball_in_our_half;
	//msg.ball_in_our_possession = this->ball_in_our_possession;
	return msg;
}

KalmanFilter ball;
KalmanFilter *home[6], *away[6];

void Callback(const krssg_ssl_msgs::SSL_DetectionFrame::ConstPtr& vmsg){
	static list<BeliefState> prev_msgQ;
	// cout << vmsg->balls[0].x << "," << vmsg->balls[0].y << endl;
	BeliefState bs(vmsg,prev_msgQ, home, away, &ball);
	// bs.print();
	pub.publish(bs.getBeliefStateMsg());
}


int main(int argc, char *argv[])
{
	ros::init(argc,argv,"beliefstate_node");
	Eigen::MatrixXd F(4, 4), H(2, 4), Q(4, 4), R(2, 2);
	double dt = 0.020;
	F << 1, dt, 0, 0, 0, 1, 0, 0, 0, 0, 1, dt, 0, 0, 0, 1;
	H << 1, 0, 0, 0, 0, 0, 1, 0;
	Q << pow(dt, 4)/4, pow(dt, 3)/2, 0, 0, pow(dt, 3)/2, pow(dt, 2), 0, 0, 0, 0, pow(dt, 4)/4, pow(dt, 3)/2, 0, 0, pow(dt, 3)/2, pow(dt, 2);
	Q = Q*1.5*1.5;
	R.setIdentity();
	R*=0.1*0.1;
	for(int i=0; i<6; i++){
		home[i] = (KalmanFilter*)malloc(sizeof(KalmanFilter));
		home[i]->init(4, 2);
		home[i]->init(F, H, Q, R, dt);
		away[i] = (KalmanFilter*)malloc(sizeof(KalmanFilter));
		away[i]->init(4, 2);
		away[i]->init(F, H, Q, R, dt);
	}
	ball.init(4, 2);
	ball.init(F, H, Q, R, dt);
	if(argc > 1){
		::isteamyellow = atof(argv[1]);
	}
	ros::NodeHandle n;
	ros::Subscriber vision_sub = n.subscribe("/vision", 10000, Callback);
	::pub = n.advertise<krssg_ssl_msgs::BeliefState>("/belief_state", 10000);
	ros::spin();
	return 0;
}