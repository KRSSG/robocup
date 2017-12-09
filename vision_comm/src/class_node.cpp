#include "class_node.hpp"

#include <climits>
#include <string>

bool is_valid(const krssg_ssl_msgs::SSL_DetectionFrame::ConstPtr& vmsg) {
   if(vmsg->robots_yellow.size() == 0 && vmsg->robots_blue.size() == 0)
      return false;
   if(vmsg->balls.size() == 0)
      return false;
   return true;
}

BeliefState::BeliefState() {}

BeliefState::BeliefState(bool isteamyellow) {
   this->prev_msg = NULL;
   this->isteamyellow = isteamyellow;
   this->initialise();
}

BeliefState::BeliefState(const BeliefState &msg) {
   this->copy(msg);
}

krssg_ssl_msgs::BeliefState BeliefState::get_beliefstate_msg() {
   krssg_ssl_msgs::BeliefState msg;

   msg.stamp = this->stamp;
   msg.isteamyellow = this->isteamyellow;
   msg.frame_number = this->frame_number;
   msg.t_capture = this->t_capture;
   msg.t_sent = this->t_sent;

   msg.ballPos.x = this->ballPos.x;
   msg.ballPos.y = this->ballPos.y; 
   msg.awayPos = this->awayPos;
   msg.homePos = this->homePos;

   msg.ballVel.x = this->ballVel.x;
   msg.ballVel.y = this->ballVel.y;
   msg.awayVel = this->awayVel;
   msg.homeVel = this->homeVel;

   msg.ballDetected = this->ballDetected;
   for(int i=0;i<msg.homeDetected.size();i++)
      msg.homeDetected[i] = this->homeDetected[i];
   for(int i=0;i<msg.awayDetected.size();i++)
      msg.awayDetected[i] = this->awayDetected[i];

   return msg;
}

void BeliefState::update_frame(const krssg_ssl_msgs::SSL_DetectionFrame *vmsg){
   BeliefState prev_state;

   if(this->prev_msg == NULL) {
      this->initialise();
   } else {
      prev_state = *this->prev_msg;
      this->stamp = ros::Time::now();
      this->frame_number = vmsg->frame_number;
      this->t_capture = vmsg->t_capture;
      this->t_sent = vmsg->t_sent;
      this->isteamyellow = isteamyellow;

      vector<krssg_ssl_msgs::SSL_DetectionRobot> homePos, awayPos;
      if(isteamyellow) {
         homePos = vmsg->robots_yellow;
         awayPos = vmsg->robots_blue;
      } else {
         homePos = vmsg->robots_blue;
         awayPos = vmsg->robots_yellow;
      }

      if(vmsg->balls.size()) {
         this->ballDetected = 1;
         this->ballPos.x = vmsg->balls[0].x;
         this->ballPos.y = vmsg->balls[0].y;
      } else {
         this->ballDetected = 0;
      }
      
      for(int i=0;i<homePos.size();i++) {
         int bot_id = homePos[i].robot_id;
         this->homeDetected[bot_id] = 1;
         this->homePos[bot_id].x = homePos[i].x;
         this->homePos[bot_id].y = homePos[i].y;
         this->homePos[bot_id].theta = homePos[i].orientation;
      }

      for(int i=0;i<awayPos.size();i++){
         int bot_id = awayPos[i].robot_id;
         this->awayDetected[bot_id] = 1;
         this->awayPos[bot_id].x = awayPos[i].x;
         this->awayPos[bot_id].y = awayPos[i].y;
         this->awayPos[bot_id].theta = awayPos[i].orientation;
      }
   }

   if(this->prev_msg == NULL) {
      prev_msg = this;
      return;
   }

   assert(prev_state.homePos.size() == 6);
   assert(prev_state.awayPos.size() == 6);

   this->ballVel.x = (this->ballPos.x - prev_state.ballPos.x)/(this->stamp - prev_state.stamp).toSec();
   this->ballVel.y = (this->ballPos.y - prev_state.ballPos.y)/(this->stamp - prev_state.stamp).toSec();

   for(int i=0;i<this->homePos.size();i++) {
      this->homeVel[i].x = (this->homePos[i].x - prev_state.homePos[i].x)/(this->stamp - prev_state.stamp).toSec();
      this->homeVel[i].y = (this->homePos[i].y - prev_state.homePos[i].y)/(this->stamp - prev_state.stamp).toSec();
   }

   for(int i=0;i<this->awayPos.size();i++) {
      this->awayVel[i].x = (this->awayPos[i].x - prev_state.awayPos[i].x)/(this->stamp - prev_state.stamp).toSec();
      this->awayVel[i].y = (this->awayPos[i].y - prev_state.awayPos[i].y)/(this->stamp - prev_state.stamp).toSec();
   }
}

void BeliefState::initialise() {
   this->stamp = ros::Time::now();
   this->isteamyellow = isteamyellow;
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
}

void BeliefState::copy(const BeliefState& bs){
   this->stamp = bs.stamp;
   this->isteamyellow = bs.isteamyellow;
   this->frame_number = bs.frame_number;
   this->t_capture = bs.t_capture;
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
}

void print(Pose2D& p) {
   cout<<"x: "<<p.x<<" y: "<<p.y<<endl;
}

void print(vector<Pose2D> p) {
   for(int i = 0; i < p.size(); i++) {
      cout<<"i: \t";
      print(p[i]);
   }
}

void print(vector<bool> p) {
   for(int i=0;i<p.size();i++) {
      cout<<"i: \t"<<p[i]<<endl;
   }
}

void BeliefState::print(string str) {
   system("clear");
   cout<<"Data recieved from : " << str << endl 
       <<"Time: "<<this->stamp<<endl
       <<"isteamyellow: "<<this->isteamyellow<<endl
       <<"frame_number: "<<frame_number<<endl
       <<"t_capture: "<<t_capture<<endl
       <<"t_sent: "<<t_sent<<endl
       <<"ballPos:"<<endl;
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
}
