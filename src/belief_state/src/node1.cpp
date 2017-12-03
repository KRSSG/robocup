#include "ros/ros.h"
#include "../../ssl_common/include/ssl_common/config.h"
#include <krssg_ssl_msgs/BeliefState.h>
#include <krssg_ssl_msgs/SSL_DetectionFrame.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <queue>
#include <deque>
#include <fstream>
#include "../../ssl_common/include/ssl_common/geometry.hpp"

const int BALL_AT_CORNER_THRESH                        = 20; 
const int HALF_FIELD_MAXX                              = 3000; 
const int HALF_FIELD_MAXY                              = 2000;
const float MAX_DRIBBLE_R                              = 3;
const int DBOX_WIDTH                                   = 600;
const int DBOX_HEIGHT                                  = 600;
using namespace std;

bool is_team_yellow;
ros::Subscriber vision_sub;
ros::Publisher pub;
queue<pair<geometry_msgs::Pose2D, ros::Time> > velQ;


  void PRINT(const krssg_ssl_msgs::BeliefState bs)
  {
    cout<<"isteamyellow: "<<bs.isteamyellow<<" frame_number: "<<bs.frame_number<<" t_capture: "<<bs.t_capture<<" t_sent: "<<bs.t_sent<<endl;
    cout<<"ballPos.x: "<<bs.ballPos.x<<" bs.ballPos.y "<<bs.ballPos.y<<" bs.ballVel.x: "<<bs.ballVel.x<<" bs.ballVel.y: "<<bs.ballVel.y<<endl;
    for(int i=0;i<6;i++)
    {
      cout<<"awayPos "<<i<<" : "<<bs.awayPos[i].x<<" "<<bs.awayPos[i].y<<endl;
    }
    for(int i=0;i<6;i++)
    {
      cout<<"homePos "<<i<<" : "<<bs.homePos[i].x<<" "<<bs.homePos[i].y<<endl;
    }
    cout<<"ballDetected: "<<bs.ballDetected<<endl;
    for(int i=0;i<6;i++)
    {
      cout<<"homeDetected "<<i<<" : "<<bs.homeDetected[i];
    }
    for(int i=0;i<6;i++)
    {
      cout<<"awayDetected "<<i<<" : "<<bs.awayDetected[i];
    }
    cout<<"our_bot_closest_to_ball: "<<bs.our_bot_closest_to_ball<<" opp_bot_closest_to_ball: "<<bs.opp_bot_closest_to_ball<<" our_goalie: "<<bs.our_goalie<<endl;
    cout<<"opp_goalie: "<<bs.opp_goalie<<" opp_bot_marking_our_attacker: "<<bs.opp_bot_marking_our_attacker<<" ball_at_corners: "<<bs.ball_at_corners<<endl;
    cout<<"ball_in_our_half: "<<bs.ball_in_our_half<<" ball_in_our_possession: "<<bs.ball_in_our_possession<<endl;
  }

krssg_ssl_msgs::BeliefState set_message(krssg_ssl_msgs::BeliefState msg)
{
  krssg_ssl_msgs::BeliefState copy;

  copy.isteamyellow=msg.isteamyellow;
  copy.frame_number=msg.frame_number ;
  copy.t_capture=msg.t_capture  ;   
  copy.t_sent=msg.t_sent   ;
  copy.ballPos=msg.ballPos  ;     
  copy.ballVel=msg.ballVel  ;
  copy.awayPos=msg.awayPos ;
  copy.homePos=msg.homePos;
  copy.ballDetected=msg.ballDetected;
  copy.homeDetected=msg.homeDetected;
  copy.awayDetected=msg.awayDetected;
  copy.our_bot_closest_to_ball=msg.our_bot_closest_to_ball;
  copy.opp_bot_closest_to_ball=msg.opp_bot_closest_to_ball;
  copy.our_goalie=msg.our_goalie;      
  copy.opp_goalie=msg.opp_goalie;   
  copy.opp_bot_marking_our_attacker=msg.opp_bot_marking_our_attacker;
  copy.ball_at_corners=msg.ball_at_corners;
  copy.ball_in_our_half=msg.ball_in_our_half;
  copy.ball_in_our_possession=msg.ball_in_our_possession;

  return copy;
}

krssg_ssl_msgs::BeliefState filtering(const krssg_ssl_msgs::BeliefState vmsg){

  using namespace krssg_ssl_msgs;
  using geometry_msgs::Pose2D;
  using geometry_msgs::Point32;

  krssg_ssl_msgs::BeliefState filterd_msg;
  filterd_msg.awayDetected = vector<uint8_t>(6, 0);
  filterd_msg.homeDetected = vector<uint8_t>(6, 0);
  filterd_msg.awayPos = vector<Pose2D>(6, Pose2D());
  filterd_msg.homePos = vector<Pose2D>(6, Pose2D());

  filterd_msg=set_message(vmsg);


  static int count = 0, t_count=0;
  static deque<float> vec_homepos_x[6], vec_awaypos_x[6], vec_ballpos_x;
  static deque<float> vec_homepos_y[6], vec_awaypos_y[6], vec_ballpos_y; 

  deque<float> temp_vec_homepos_x[6], temp_vec_awaypos_x[6], temp_vec_ballpos_x;
  deque<float> temp_vec_homepos_y[6], temp_vec_awaypos_y[6], temp_vec_ballpos_y;

  // double arr[5] = {0.086, -0.143, -0.086, 0.257, 0.886};
  double arr[5] = {0.886, 0.257, -0.086, -0.143, 0.086};
  if(count < 5){
    ++count;


    for (int i = 0; i < 6; ++i)
    {
      vec_homepos_x[i].push_back(vmsg.homePos[i].x);
      vec_awaypos_x[i].push_back(vmsg.awayPos[i].x);

      vec_homepos_y[i].push_back(vmsg.homePos[i].y);
      vec_awaypos_y[i].push_back(vmsg.awayPos[i].y);      
    }
    vec_ballpos_x.push_back(vmsg.ballPos.x);
    vec_ballpos_y.push_back(vmsg.ballPos.y);

    for (int i = 0; i < count; ++i)
    {
      for (int j = 0; j < 6; ++j)
      {
        temp_vec_homepos_x[j][i] = vec_homepos_x[j][i] * arr[i];
        temp_vec_awaypos_x[j][i] = vec_awaypos_x[j][i] * arr[i];

        temp_vec_homepos_y[j][i] = vec_homepos_y[j][i] * arr[i];
        temp_vec_awaypos_y[j][i] = vec_awaypos_y[j][i] * arr[i];
      } 

      temp_vec_ballpos_x[i] = vec_ballpos_x[i] * arr[i];
      temp_vec_ballpos_y[i] = vec_ballpos_y[i] * arr[i];
    }

    // average
    float sum_homepos_x[6], sum_awaypos_x[6], sum_ballpos_x=0;
    float sum_homepos_y[6], sum_awaypos_y[6], sum_ballpos_y=0;

    for(int i=0;i<6;i++)
      sum_homepos_x[i]=sum_homepos_y[i]=sum_awaypos_x[i]=sum_awaypos_y[i]=0;

      for (int j = 0; j < 6; ++j)
      {
        for (int i = 0; i < count; ++i)
        {
          sum_homepos_x[j] += temp_vec_homepos_x[j][i];
          sum_awaypos_x[j] += temp_vec_awaypos_x[j][i];

          sum_homepos_y[j] += temp_vec_homepos_y[j][i];
          sum_awaypos_y[j] += temp_vec_awaypos_y[j][i];
        }      
   
      }
     
      for(int i=0;i<count;i++){
        sum_ballpos_x += temp_vec_ballpos_x[i];
        sum_ballpos_y += temp_vec_ballpos_y[i]; 
      }

      // replace with filtered msg
       // replace with filtered msg
    for (int i = 0; i < 6; ++i)
    {
    filterd_msg.homePos[i].x = sum_homepos_x[i];
    filterd_msg.awayPos[i].x = sum_awaypos_x[i];
      // cout<<sum_homepos_x[i]<<" "<<sum_homepos_y[i]<<endl;
    filterd_msg.homePos[i].y = sum_homepos_y[i];
     filterd_msg.awayPos[i].y = sum_awaypos_y[i];     
    }
      filterd_msg.ballPos.x = sum_ballpos_x;
      filterd_msg.ballPos.y = sum_ballpos_y;

    }    
  else{

    for (int i = 0; i < 6; ++i)
    {
      vec_homepos_x[i].push_back(vmsg.homePos[i].x);
      vec_awaypos_x[i].push_back(vmsg.awayPos[i].x);

      vec_homepos_y[i].push_back(vmsg.homePos[i].y);
      vec_awaypos_y[i].push_back(vmsg.awayPos[i].y);      
    }
    vec_ballpos_x.push_back(vmsg.ballPos.x);
    vec_ballpos_y.push_back(vmsg.ballPos.y);


    for (int i = 0; i < 6; ++i)
    {
      vec_homepos_x[i].pop_front();
      vec_awaypos_x[i].pop_front();

      vec_homepos_y[i].pop_front();
      vec_awaypos_y[i].pop_front();      
    }
    vec_ballpos_x.pop_front();
    vec_ballpos_y.pop_front();

    for (int i = 0; i < 5; ++i)
    {
      for (int j = 0; j < 6; ++j)
      {
        temp_vec_homepos_x[j][i] = vec_homepos_x[j][i] * arr[i];
        temp_vec_awaypos_x[j][i] = vec_awaypos_x[j][i] * arr[i];

        temp_vec_homepos_y[j][i] = vec_homepos_y[j][i] * arr[i];
        temp_vec_awaypos_y[j][i] = vec_awaypos_y[j][i] * arr[i];        
      } 

      temp_vec_ballpos_x[i] = vec_ballpos_x[i] * arr[i];
      temp_vec_ballpos_y[i] = vec_ballpos_y[i] * arr[i];
    }

    // average
    float sum_homepos_x[6]={0}, sum_awaypos_x[6]={0}, sum_ballpos_x=0;
    float sum_homepos_y[6]={0}, sum_awaypos_y[6]={0}, sum_ballpos_y=0;

    for (int j = 0; j < 6; ++j)
    {
      for (int i = 0; i < 5; ++i)
      {
        sum_homepos_x[j] += temp_vec_homepos_x[j][i];
        sum_awaypos_x[j] += temp_vec_awaypos_x[j][i];

        sum_homepos_y[j] += temp_vec_homepos_y[j][i];
        sum_awaypos_y[j] += temp_vec_awaypos_y[j][i];        
      }      
 
    }
    for(int i=0;i<5;i++){
        sum_ballpos_x += temp_vec_ballpos_x[i];
        sum_ballpos_y += temp_vec_ballpos_y[i];      
    }

    // replace with filtered msg
    for (int i = 0; i < 6; ++i)
    {
    filterd_msg.homePos[i].x = sum_homepos_x[i];
    filterd_msg.awayPos[i].x = sum_awaypos_x[i];
      // cout<<sum_homepos_x[i]<<" "<<sum_homepos_y[i]<<endl;
    filterd_msg.homePos[i].y = sum_homepos_y[i];
     filterd_msg.awayPos[i].y = sum_awaypos_y[i];     
    }
      filterd_msg.ballPos.x = sum_ballpos_x;
      filterd_msg.ballPos.y = sum_ballpos_y;
  }
  return filterd_msg;
}


const int Q_SIZE = 3;
void Callback(const krssg_ssl_msgs::SSL_DetectionFrame::ConstPtr& vmsg) 
{
  static krssg_ssl_msgs::SSL_DetectionFrame vmsg_temp; 
  if((vmsg->robots_yellow.size()==0 && vmsg->robots_blue.size()==0))
  {
    cout<<"bad frame bot!!!!!!!!!!!!!!!!!!! "<<endl;
    vmsg_temp.frame_number=vmsg->frame_number+1; 
  }
  else
  {
    cout<<"proceeding"<<endl;
    vmsg_temp.frame_number=vmsg->frame_number; 
    vmsg_temp.t_capture=vmsg->t_capture;     
    vmsg_temp.t_sent=vmsg->t_sent;        
    vmsg_temp.camera_id=vmsg->camera_id;          
    vmsg_temp.robots_yellow=vmsg->robots_yellow; 
    vmsg_temp.robots_blue=vmsg->robots_blue;   
    if(vmsg->balls.size()==0)
    {
      cout<<"bad frame ball!!!!!!!!!!!!!!!!!!! "<<endl;
    }
    else
    {
      vmsg_temp.balls=vmsg->balls;    
    }
  }
  

  //printf("got a vmsg1!\n");
  using namespace krssg_ssl_msgs;
  using geometry_msgs::Pose2D;
  using geometry_msgs::Point32;
  krssg_ssl_msgs::BeliefState msg;
  msg.frame_number = vmsg_temp.frame_number;
  msg.t_capture = vmsg_temp.t_capture;
  msg.t_sent = vmsg_temp.t_sent;
  msg.isteamyellow = is_team_yellow;
  
  vector<SSL_DetectionRobot> homePos, awayPos;
  if (is_team_yellow) {
    homePos = vmsg_temp.robots_yellow;
    awayPos = vmsg_temp.robots_blue;
  } else {
    homePos = vmsg_temp.robots_blue;
    awayPos = vmsg_temp.robots_yellow;
  }




  if (vmsg_temp.balls.size() > 0) {
    assert(velQ.size() != 0);
    Pose2D oldPos = velQ.front().first;
    ros::Time oldTime = velQ.front().second;
    velQ.pop();
    ros::Time curTime = ros::Time::now();
    msg.ballDetected = true;
    msg.ballPos.x = vmsg_temp.balls[0].x;
    msg.ballPos.y = vmsg_temp.balls[0].y;

  //printf("got a vmsg2!\n");
    if(vmsg_temp.balls[0].x <= 0 )
        msg.ball_in_our_half = true;
    else
        msg.ball_in_our_half = false;

    if(fabs(vmsg_temp.balls[0].x) > (HALF_FIELD_MAXX - BALL_AT_CORNER_THRESH) && fabs(vmsg_temp.balls[0].y) > (HALF_FIELD_MAXY - BALL_AT_CORNER_THRESH))
        msg.ball_at_corners = true;
    else
        msg.ball_at_corners = false;

    msg.ballVel.x = (msg.ballPos.x - oldPos.x)/(curTime-oldTime).toSec();
    msg.ballVel.y = (msg.ballPos.y - oldPos.y)/(curTime-oldTime).toSec();
    velQ.push(make_pair(msg.ballPos, curTime));
  } else {
    msg.ballDetected = 0;
  }
  
  //printf("got a vmsg3!\n");
  // assuming 6 robots per side!
  msg.awayDetected = vector<uint8_t>(6, 0);
  msg.homeDetected = vector<uint8_t>(6, 0);
  // cout<<"away Detected: "<<msg.awayDetected.size()<<", home detected: "<<msg.homeDetected.size()<<endl;
  msg.awayPos = vector<Pose2D>(6, Pose2D());
  msg.homePos = vector<Pose2D>(6, Pose2D());

  //printf("got a vmsg4!\n");
  float distance_from_ball = 999999,dist,temp=10;
  msg.ball_in_our_possession = false;

  for (int i = 0; i < homePos.size(); ++i)
  {
    int bot_id = homePos[i].robot_id;
    msg.homeDetected[bot_id] = 1;
    msg.homePos[bot_id].x = homePos[i].x;
    msg.homePos[bot_id].y = homePos[i].y;

    dist = sqrt(pow((homePos[i].x - msg.ballPos.x),2) + pow((homePos[i].y - msg.ballPos.y) , 2));
    if(dist < distance_from_ball){
      distance_from_ball = dist;
      msg.our_bot_closest_to_ball = i;

      if(distance_from_ball < MAX_DRIBBLE_R){
        msg.ball_in_our_possession = true;
      }
    }

    msg.homePos[bot_id].theta = homePos[i].orientation;

    if(homePos[i].x < (-HALF_FIELD_MAXX + DBOX_WIDTH) && fabs(homePos[i].y) < DBOX_HEIGHT){
        
        if(temp = 10)
            temp = i;
        else if(fabs(homePos[temp].orientation) > 1.54 && fabs(homePos[i].orientation) < 1.54)
            temp = i;
    }
  }
  msg.our_goalie = temp;
  // temp = 10;
  temp = 10;
  distance_from_ball = 999999;
  // printf("got a vmsg41! %d\n",awayPos.size());
 // cout<<awayPos.size()<<endl;
// cout<<(msg.ballPos.x)<<" "<<(msg.ballPos.y)<<endl;
  for (int i = 0; i < awayPos.size(); ++i)
  {
    
    //printf("%d \n",awayPos[i].robot_id);
    //cout<<sizeof(awayPos[i].robot_id)<<endl;
    int bot_id = awayPos[i].robot_id;
 //   cout<<"Ghus gaya0"<<endl;
    //cout<<"got a vmsg481"<<endl;
    //fflush(stdout);
    msg.awayDetected[bot_id] = 1;
 //   cout<<"Ghus gaya1"<<endl;
    msg.awayPos[bot_id].x = awayPos[i].x;
//    cout<<"Ghus gaya2"<<endl;
    msg.awayPos[bot_id].y = awayPos[i].y;
//    cout<<bot_id<<endl;
//    cout<<"Ghus gaya3"<<endl;
    
 //   dist = sqrt(pow((awayPos[i].x - vmsg_temp.balls[0].x),2) + pow((awayPos[i].y - vmsg_temp.balls[0].y) , 2));
    dist = sqrt(((awayPos[bot_id].x - msg.ballPos.x)*(awayPos[bot_id].x - msg.ballPos.x)) + ((awayPos[bot_id].y - msg.ballPos.y)*(awayPos[bot_id].y - msg.ballPos.y)));
 //   cout<<"Ghus gaya4"<<endl;
    
    if(dist < distance_from_ball){
      distance_from_ball = dist;
      msg.opp_bot_closest_to_ball = i;
    }
//    <<"P 5\n"<<endl;
    msg.awayPos[bot_id].theta = awayPos[i].orientation;

    if(awayPos[i].x > (HALF_FIELD_MAXX - DBOX_WIDTH) && fabs(awayPos[i].y) < DBOX_HEIGHT){
        
        if(temp = 10)
            temp = i;
        else if(fabs(awayPos[temp].orientation) > 1.54 && fabs(awayPos[i].orientation) < 1.54)
            temp = i;
    }

  }
  msg.opp_goalie = temp;
  // cout<<msg.homePos[1].x<<" "<<msg.homePos[1].y<<endl;
  // PRINT(msg);

  // filtering
  krssg_ssl_msgs::BeliefState filtered_msg;
  filtered_msg = filtering(msg);
  // cout<<"msg ------------------"<<endl;
  // PRINT(msg);
  // cout<<"filtered msg ------------------"<<endl;
  PRINT(filtered_msg);
  pub.publish(filtered_msg);


  float angle = Vector2D<int>::angle(Vector2D<int>(filtered_msg.ballPos.x, filtered_msg.ballPos.y), Vector2D<int>(filtered_msg.homePos[1].x, filtered_msg.homePos[1].y));
  cout<<"angle ::::::::::::####################"<<angle<<endl;
  cout<<"homePos.theta ::::::::::::####################"<<filtered_msg.homePos[1].theta<<endl;
  
 // cout<<"P 6\n"<<endl;
}


int main(int argc, char **argv)
{
  // if no argument is passed, assumed our team is blue
  // else if argument 0 = our team blue, 1 = our team yellow
  ros::init(argc, argv, "beliefstate_node");
  is_team_yellow = 0;
  if (argc > 1) {
    is_team_yellow = atof(argv[1]);
  }
  ros::NodeHandle n;
  for (int i = 0; i < Q_SIZE; ++i)
  {
    velQ.push(make_pair(geometry_msgs::Pose2D(), ros::Time::now()));
  }
  vision_sub = n.subscribe("/vision", 10000, Callback);
  pub = n.advertise<krssg_ssl_msgs::BeliefState>("/belief_state", 10000);
  ros::spin();

  return 0;
}