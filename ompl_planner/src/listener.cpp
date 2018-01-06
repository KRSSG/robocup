/**
 * @file listener.cpp
 * @brief Communication Module for planner.
 * 
 * Node Name :- /listener
 * Suscribed to :- /beliefstate, /gui_params
 * Publish to :- /path_planner_ompl
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include "krssg_ssl_msgs/planner_path.h"
#include "krssg_ssl_msgs/point_2d.h"
#include "krssg_ssl_msgs/point_SF.h"
#include "krssg_ssl_msgs/path_plan.h"
#include <iostream>
#include "MotionPlanner.h"
#include <bits/stdc++.h>
#include <time.h>
using namespace std;
int count_=0;
bool replanCondition;
struct timeval t;
long long int currT;

// std::vector<krssg_ssl_msgs::point_2d> path_points;
std::vector<krssg_ssl_msgs::point_2d> v;
// krssg_ssl_msgs::planner_path points;
// krssg_ssl_msgs::point_2d point_, initial_p, final_p;
ros::Publisher pub;
krssg_ssl_msgs::point_SF gui_msgs;

std::vector<krssg_ssl_msgs::point_2d> awayVel;
std::vector<krssg_ssl_msgs::point_2d> homeVel;

krssg_ssl_msgs::point_2d ballPos;
/**
 * @brief      Callback for gui msgs
 *
 * @param[in]  msg   msg from /gui_params
 * 
 * Step Size
 * Max_Iteration
 * Bias_Param
 */
void Callback_gui(const krssg_ssl_msgs::point_SF::ConstPtr& msg)
{
  // TODO
  // Link Gui with planner completely
  gui_msgs.step_size = msg->step_size;
  gui_msgs.max_iteration = msg->max_iteration;
  gui_msgs.bias_param = msg->bias_param;
}

/**
 * @brief      BeliefState Callback
 *
 * @param[in]  msg   msg from /belief_state
 * 
 * Subscribe to BeliefState, Get start and end points and plan 
 * path accordingly
 *  
 * @see Planning
 * @see Planning::planSimple()
 * @see Planning::plan()
 * @see Planning::recordSolution()
 */
void Callback(const krssg_ssl_msgs::BeliefState::ConstPtr& msg)
{
  // cout<<"in beliefstate Callback function \n\n";
  krssg_ssl_msgs::point_2d p;
  count_++;
  krssg_ssl_msgs::point_2d vel;
  v.clear();

  ballPos.x = msg->ballPos.x*BS_TO_OMPL;
  ballPos.y = msg->ballPos.y*BS_TO_OMPL;
  
  for(int i=0;i<msg->homePos.size();i++){
    p.x = msg->homePos[i].x*BS_TO_OMPL;
    p.y = msg->homePos[i].y*BS_TO_OMPL;
  	v.push_back(p);
    // vel.x = msg->homeVel[i].x;
    // vel.y = msg->homeVel[i].y;
    // homeVel.push_back(vel);
  }

  for(int i=0;i<msg->awayPos.size();i++){
    p.x = msg->awayPos[i].x*BS_TO_OMPL;
    p.y = msg->awayPos[i].y*BS_TO_OMPL;
  	v.push_back(p);
    // vel.x = msg->awayVel[i].x;
    // vel.y = msg->awayVel[i].y;
    // awayVel.push_back(vel);
  }
}

float distance_(float x1, float y1, float x2, float y2)
{
  float dx = x1 - x2;
  float dy = y1 - y2;

  return sqrt(dx*dx + dy*dy);
}

bool path(krssg_ssl_msgs::path_plan::Request &req,
          krssg_ssl_msgs::path_plan::Response &res){
  krssg_ssl_msgs::point_2d start;
  krssg_ssl_msgs::point_2d target;
  krssg_ssl_msgs::point_2d point;
  krssg_ssl_msgs::planner_path points;
  start.x = req.start.x;
  start.y = req.start.y;
  target.x = req.target.x;
  target.y = req.target.y;
  

  ROS_INFO("Start (%f %f)  target (%f %f) ",start.x,start.y,target.x,target.y);
  ROS_INFO("Distance = %f, threshold = %f",distance_(start.x, start.y, target.x, target.y)*BS_TO_OMPL, radius);
  ROS_INFO("Planning");


  bool avoid_ball = req.avoid_ball;
  ROS_INFO("avoid_ball %d",avoid_ball);
  if (avoid_ball)
    v.push_back(ballPos);


  Planning planning(v,v.size(),gui_msgs);
  planning.planSimple();
  planning.plan(start.x*BS_TO_OMPL,start.y*BS_TO_OMPL,
                target.x*BS_TO_OMPL,target.y*BS_TO_OMPL);
  std::vector<krssg_ssl_msgs::point_2d> path_points;
  path_points = planning.recordSolution();
  for (int i = 0; i < path_points.size(); ++i)
  {
    point.x = path_points[i].x*OMPL_TO_BS;
    point.y = path_points[i].y*OMPL_TO_BS;
    res.path.push_back(point);
    points.point_array.push_back(point);
  }
  pub.publish(points);
  ROS_INFO("Sending Response");
  ROS_INFO("Publishing Points on publisher");
  return true;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/belief_state", 1000, Callback);
  ros::Subscriber sub1 = n.subscribe("/gui_params", 1000, Callback_gui);
  ros::ServiceServer service = n.advertiseService("planner", path);
  pub = n.advertise<krssg_ssl_msgs::planner_path>("/path_planner_ompl", 1000);
  ros::spin();
  return 0;
}
