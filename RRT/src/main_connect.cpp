#include <bits/stdc++.h>
#include "../include/RRTConnect_implementation.hpp"
#include "ros/ros.h"
#include "krssg_ssl_msgs/planner_path.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include "krssg_ssl_msgs/point_2d.h"
#include "krssg_ssl_msgs/point_SF.h"

using namespace std;
using namespace rrt;

RRT<int> test;
Utils::Point<int> start;
Utils::Point<int> finish;
ros::Publisher pub;
int p;

bool check(Utils::Point<int> cur)
{
	if(abs(cur.x)<3300 && abs(cur.y)<2200)
		return true;
	return false;
}


void send_points()
{
	krssg_ssl_msgs::planner_path points;
	 
	krssg_ssl_msgs::point_2d point;
    vector<Utils::Point<int> > path=test.getPointsOnPath();
	

	// for(int i=0;i<path.size();i++)
	// 	cout<<"("<<path[i].x<<","<<path[i].y<<")";
	  int count = 0;

	
	  cout<<" path.size() = "<<path.size()<<endl;
	  cout<<"generated path start = "<<path[0].x<<","<<path[0].y<<"  Finish = "<<path[path.size()-1].x<<","<<path[path.size()-1].y<<endl;
	  for(int i=0;i<path.size();i++)
	  {
	  	point.x=path[i].x;
	  	point.y=path[i].y;
	  	points.point_array.push_back(point);
	  	// cout<<path[i].x<<","<<path[i].y<<endl;
	  }	  

	 

	  pub.publish(points); cout<<"data published "<<p++<<endl;
}

void set_space(const krssg_ssl_msgs::BeliefState::ConstPtr& msg)
{
	test.ObstaclePoints.clear();

	Utils::Point<int> p;
	for(int i=1;i<msg->homePos.size();i++){
  	p.x=fabs(msg->homePos[i].x+3300)/8.0;
  	p.y=fabs(msg->homePos[i].y+2200)/8.0;
  	//cout<<p.x<<" "<<p.y<<endl;
  	test.ObstaclePoints.push_back(p);
  }

  for(int i=0;i<msg->awayPos.size();i++){
  	p.x=fabs(msg->awayPos[i].x+3300)/8.0;
  	p.y=fabs(msg->awayPos[i].y+2200)/8.0;
  	test.ObstaclePoints.push_back(p);
  }
}

void callback(const krssg_ssl_msgs::point_SF::ConstPtr& msg)
{
	cout<<" got step_size="<<msg->step_size<<" max_iteration="<<msg->max_iteration<<endl;
	start.x=msg->s_x;
	start.y=msg->s_y;
	finish.x=msg->f_x;
	finish.y=msg->f_y;
	cout<<"going to create path start = "<<start.x<<","<<start.y<<" end = "<<finish.x<<","<<finish.y<<endl;

	test.setStepLength(msg->step_size);
	test.setBiasParameter(msg->bias_param);
	test.setMaxIterations(msg->max_iteration);
	test.setEndPoints(start,finish);

	test.plan();
	cout<<"Hey!!!!!";
	send_points();
}

int main(int argc, char **argv)
{
	srand(time(NULL));
	ros::init(argc, argv, "mkPath_cpp");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/gui_params", 1000, callback);
	ros::Subscriber sub1 = n.subscribe("/belief_state", 1000, set_space);
    pub = n.advertise<krssg_ssl_msgs::planner_path>("path_planner", 1000);

	Utils::Point<int> start,finish,origin;
	start.x=-10;
	start.y=10;
	finish.x=1000;
	finish.y=730;
	origin.x=0;
	origin.y=0;

	test.setEndPoints(start,finish);
	test.setCheckPointFunction(*(check));
	test.setStepLength(200);
	test.setHalfDimensions(3000.0,2000.0);
	test.setBiasParameter(100);
	test.setOrigin(origin);
	test.setMaxIterations(10000);
	test.setObstacleRadius(50);
	test.plan();
	cout<<"#################################################"<<endl;
	vector<Utils::Point<int> > path=test.getPointsOnPath();
	for(int i=0;i<path.size();i++)
		cout<<path[i].x<<","<<path[i].y<<endl;
	ros::spin();
}	
