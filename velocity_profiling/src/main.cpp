#include <bits/stdc++.h>
#include "functions.h"
using namespace std;

const int MAX_VEL = 50, BOT_ID = 2;
ros::Publisher pub, pub_to_gui, pubreplan;
krssg_ssl_msgs::planner_path path_points_pub;
krssg_ssl_msgs::point_2d point_;
krssg_ssl_msgs::gr_Commands final_msgs;
krssg_ssl_msgs::gr_Robot_Command command_msgs;
double VEL_ANGLE = 0;
#define MAJOR_AXIS_FACTOR 10
#define MINOR_AXIS_FACTOR 2
#define PI 3.141592653589793
#define radius 100
int count = 1;
double current_speed = 0;
int flagreplan = 0;
void send_stop()
{
	cout<<"sending stop\n";
	krssg_ssl_msgs::pid_message msg;
	msg.velX = 0.0;
	msg.velY = 0.0;
	msg.errorX = 0;
	msg.errorY = 0;
	msg.id = BOT_ID;
	msg.flag = flagreplan;
	current_speed = 0;
	pub.publish(msg);
	return ;
}

bool InEllipse(int my_id, int opp_id, double alpha)
{
  double a = MAJOR_AXIS_FACTOR*radius/2.0, b = MINOR_AXIS_FACTOR*radius/2.0;
  double x0 = home_pos[my_id].x + a*cos(alpha), y0 = home_pos[my_id].y + a*sin(alpha);
  double xp = away_pos[opp_id].x, yp = away_pos[opp_id].y;
  double v1 = (cos(alpha)*(xp-x0) + sin(alpha)*(yp-y0))/a;
  double v2 = (sin(alpha)*(xp-x0) - cos(alpha)*(yp-y0))/b;
  // cout<<"center = "<<x0<<","<<y0<<" point = "<<xp<<","<<yp<<endl;
  v1 = v1*v1;
  v2 = v2*v2;
  double value = v1  + v2;
  // cout<<"value = "<<value<<"\n alpha = "<<alpha*180/PI<<endl;
  if(value<=1)
    return 1;
  else
    return 0;
}

int cnt = 5, cnt1 = 5;
bool FLAG = false;

bool InEllipse1(int my_id, int opp_id, double alpha)
{
  double a = MAJOR_AXIS_FACTOR*radius/2.0, b = MINOR_AXIS_FACTOR*radius/2.0;
  double x0 = home_pos[my_id].x + a*cos(alpha), y0 = home_pos[my_id].y + a*sin(alpha);
  double xp = home_pos[opp_id].x, yp = home_pos[opp_id].y;
  double v1 = (cos(alpha)*(xp-x0) + sin(alpha)*(yp-y0))/a;
  double v2 = (sin(alpha)*(xp-x0) - cos(alpha)*(yp-y0))/b;
  // cout<<"center = "<<x0<<","<<y0<<" point = "<<xp<<","<<yp<<endl;
  v1 = v1*v1;
  v2 = v2*v2;
  double value = v1  + v2;
  // cout<<"value = "<<value<<"\n alpha = "<<alpha*180/PI<<endl;
  if(value<=1)
    return 1;
  else
    return 0;
}

bool ShouldReplan()
{
	if(current_speed<1)
	{
		FLAG = false;
		return false;
	}

	for(int i=0;i<away_pos.size();i++)
		if(InEllipse(BOT_ID, i, VEL_ANGLE))
			return true;
	for(int i=1;i<home_pos.size();i++)
		if(InEllipse1(BOT_ID, i, VEL_ANGLE))
			return true;	
	return false;	
}
void send_vel(double speed, double motion_angle, int index)
{
	krssg_ssl_msgs::pid_message msg;
	current_speed = speed;
	speed/=1000 ;
	cout<<"motion_angle = "<<motion_angle<<endl;
	double bot_angle = home_pos_theta[BOT_ID];
	double vX = speed*cos(motion_angle);
	double vY = speed*sin(motion_angle);
	msg.velX = vX;
	msg.velY = vY;
	msg.flag = flagreplan;
	// msg.velX = 0.0;
	// msg.velY = 0.5;
	msg.errorX = path_points[index].x - home_pos[BOT_ID].x;
	msg.errorY = path_points[index].y - home_pos[BOT_ID].y;
	double error_mag = sqrt(msg.errorX*msg.errorX + msg.errorY*msg.errorY);
	if(error_mag>320)
	{
		cout<<"error_mag = "<<error_mag<<endl;

		FLAG = true;
	}
	msg.id = BOT_ID;
	msg.botAngle = bot_angle;

	pub.publish(msg);
	VEL_ANGLE = motion_angle;
	
	return;
}

int flag = 0;
void Callback_BS(const krssg_ssl_msgs::BeliefState::ConstPtr& msg)
{
	cout<<"speed = "<<current_speed<<endl;
	if(PATH_RECEIVED==false)
	{
		cout<<"Path PATH_RECEIVED = False\n";
		return;
	}

	for(int i=0;i<6;i++)
	{
		home_pos_theta[i] = msg->homePos[i].theta;
		home_pos[i].x = msg->homePos[i].x;
		home_pos[i].y = msg->homePos[i].y;
		away_pos[i].x = msg->awayPos[i].x;
		away_pos[i].y = msg->awayPos[i].y;
	}
	float currX = home_pos[BOT_ID].x;
  float currY = home_pos[BOT_ID].y;
  float endX = msg->ballPos.x;
  float endY = msg->ballPos.y;
  float a = currX - endX;
  float b = currY - endY;
  float dist__ = sqrt(a*a + b*b);
  if(dist__<220)
  {
    cout<<"Reached!!___________";
    send_stop();
    return;
  }
	curr_time = ros::Time::now().toSec();
	double t = curr_time - start_time;

	flagreplan = 0;
	if(t>ExpectedTraverseTime)
	{	flag = 0;
		flagreplan = 1;
		if(FLAG)
			cout<<"------------Reason FLAG\n";
		else if(t>ExpectedTraverseTime)
			cout<<" Time Out, So REPLANNING! \n\n";
		else
			cout<<"ShouldReplan returned true! \n\n\n\n";
		cnt--;
		cout<<"Replanning! \n";
		send_stop();
		krssg_ssl_msgs::replan msg;
		msg.x = true;
		pubreplan.publish(msg);
		FLAG = false;
		return;
	}

	if(trapezoid(t, distance_traversed, out_speed))
	{
		int index = GetExpectedPositionIndex(distance_traversed);
		if(index == -1)
		{
			cout<<"Deviated!!\n\n";
			send_stop();
			return;
		}
		send_vel(out_speed, vel_angle[index],index);
	}
	else
	{
		cout<<"Motion Not Possible!! \n\n";
		return;
	}
}

void Callback(const krssg_ssl_msgs::planner_path::ConstPtr& msg)
{
	if(flag==1)
		return;
	flag = 1;
	cout<<"in path_planner Callback function. \n";
	PATH_RECEIVED = true;
	path_points.clear();
	vel_angle.clear();
	double dely = msg->point_array[2].y - msg->point_array[0].y;
	double delx = msg->point_array[2].x - msg->point_array[0].x;
	VEL_ANGLE = atan2(dely, delx);
	int size_ = msg->point_array.size();

	cout<<endl;
	for(int i=0;i<size_;i++)
	{
		point p;
		p.x = msg->point_array[i].x;
		p.y = msg->point_array[i].y;
		path_points.push_back(p);
		// cout<<"("<<p.x<<","<<p.y<<"),";
		if(i<size_-1 && i>0)
		{
			double dx = msg->point_array[i+1].x - msg->point_array[i-1].x;
			double dy = msg->point_array[i+1].y - msg->point_array[i-1].y;
			vel_angle.push_back(atan2(dy, dx));
			// cout<<"1 "<<dy/dx;
		}
		else if(i==0)
		{
			double dx = msg->point_array[i+1].x - msg->point_array[i].x;
			double dy = msg->point_array[i+1].y - msg->point_array[i].y;
			vel_angle.push_back(atan2(dy, dx));
			// cout<<"2 "<<dy/dx;
			// cout<<"vel_angle[0] = "<<vel_angle[0]<<endl;
		}
		else
		{
			double dx = msg->point_array[i].x - msg->point_array[i-1].x;
			double dy = msg->point_array[i].y - msg->point_array[i-1].y;
			vel_angle.push_back(atan2(dy, dx));
			// cout<<"3 "<<dy/dx;
		}
		// cout<<" "<<vel_angle[vel_angle.size() - 1]<<endl;
	}

	// cout<<endl;
	start_point.x = msg->point_array[0].x ;
	start_point.y = msg->point_array[0].y;
	goal_point.x = msg->point_array[size_-1].x;
	goal_point.y = msg->point_array[size_-1].y;

	path_length = GetPathLength();
	cout<<"----------------"<<path_length<<endl;
	ExpectedTraverseTime = getTime(path_length, path_length, MAX_SPEED, MAX_ACC, START_SPEED, FINAL_SPEED);
	cout<<"ExpectedTraverseTime = "<<ExpectedTraverseTime<<"\n";
	
	cout<<endl;
	start_time = ros::Time::now().toSec();
	curr_time =  ros::Time::now().toSec();
	cout<<"setting start_time = "<<start_time<<endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vel_profiling");

  	ros::NodeHandle n;
  	flagreplan = 1;
	ros::Subscriber sub = n.subscribe("/path_planner_ompl", 1000, Callback);
	pub = n.advertise<krssg_ssl_msgs::pid_message>("/pid", 1000);
	pubreplan = n.advertise<krssg_ssl_msgs::replan>("/replan",1000);
	ros::Subscriber sub1 = n.subscribe("/belief_state", 1000, Callback_BS);
	ros::spin();
	return 0;
}