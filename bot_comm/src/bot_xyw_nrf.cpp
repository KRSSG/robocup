#include <stdio.h>
#include "serial.h"
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "krssg_ssl_msgs/gr_BallReplacement.h"
#include "krssg_ssl_msgs/gr_Robot_Command.h"
#include "krssg_ssl_msgs/gr_RobotReplacement.h"
#include "krssg_ssl_msgs/gr_Packet.h"
#include "krssg_ssl_msgs/gr_Commands.h"
#include "krssg_ssl_msgs/gr_Replacement.h"
#include <krssg_ssl_msgs/sslDebug_Data.h>

#define RATIO 40    
#define FACTOR_T RATIO
#define FACTOR_N RATIO
#define FACTOR_W 90

/*
    0 = 00
    1 = 01
    2 = 10
    3 = 11
    <DRIBBLER><KICKER>
*/
#define _DRIBBLER   1<<0
#define _KICKER     1<<1

using namespace std;

const int TEAM_ID = 127;  //yellow
const double Radius=0.087; //bot radius
const double radius=0.025; //wheel radius
const double PI=3.141592653589793;
double max_vel_wheel=5000.0; //this is in rpm
double theta[4]={30.0,150.0,225.0,315.0};

HAL::Serial serial;
unsigned static char buf[32];
static int k=0;
void vel_convert(const double* vel_xyw, double* vel_wheel)
{
    double vx=vel_xyw[0];
    double vy=vel_xyw[1];
    double vw=vel_xyw[2];
    vw*=-1;

    vel_wheel[0] =   (( (Radius * vw) - (vx * sin(theta[0])) + (vy * cos(theta[0]))) )/radius;
    vel_wheel[1] =   (( (Radius * vw) - (vx * sin(theta[1])) + (vy * cos(theta[1]))) )/radius;
    vel_wheel[2] =   (( (Radius * vw) - (vx * sin(theta[2])) + (vy * cos(theta[2]))) )/radius;
    vel_wheel[3] =   (( (Radius * vw) - (vx * sin(theta[3])) + (vy * cos(theta[3]))) )/radius;
    
    for (int i = 0; i < 4; i++)
        vel_wheel[i]=vel_wheel[i]/PI;
    
    for (int i = 0; i < 4; i++)
        if(vel_wheel[i]>0)
            vel_wheel[i] = 126 + ((vel_wheel[i]-max_vel_wheel)*(126.0))/max_vel_wheel;
        else
            vel_wheel[i] = 256 - ((vel_wheel[i]-0)*(129.0-256.0))/max_vel_wheel;

}

void CallBack(const krssg_ssl_msgs::gr_Commands::ConstPtr& msg)
{
    cout << "Receiving callback data for bot: " << msg->robot_commands.id << "\n";
    double vel_xyw[3];
    vel_xyw[0] = (int)(msg->robot_commands.velnormal * FACTOR_T);
    vel_xyw[1] = -1*(int)(msg->robot_commands.veltangent * FACTOR_N);
    vel_xyw[2] =(int)(msg->robot_commands.velangular * FACTOR_W);
    // printf("%d %d %d %d\n",msg->robot_commands.id, vel_xyw[0], vel_xyw[1], vel_xyw[2]);
    buf[0] = TEAM_ID;
    double vel_wheel[4];
    vel_convert(vel_xyw, vel_wheel);

    int _start = 1+msg->robot_commands.id*5, c=0;
    for(int i=_start;i<_start + 4;i++, c++)
        buf[i] = (int)vel_wheel[c];
    cout<<msg->robot_commands.kickspeedx<<" is kicjkssdhfvsdgjivgsdjkfg\n";
    if(msg->robot_commands.kickspeedx > 6)
    {
        
        buf[_start+4] = 3;
    }
    else
    buf[_start + 4] = 0;

    for(int i=0;i<32;i++){
        cout<<(int)buf[i]<<" ";
    }
    cout<<endl;
    cout<<"$$$$$$$$$$$$$$$$$$$$$$$$"<<endl;
    cout<<serial.Write(buf,32)<<endl;
    cout<<"Here"<<endl;
}

int main(int argc, char *argv[])
{
    //Converting angles to radians
    for (int i = 0; i < 4; ++i)
        theta[i]=theta[i]*PI/180;
    if(!serial.Open("/dev/ttyUSB0", 230400)) {
        printf("Could not open the fucking parts.\n");
        exit(0);
    }
    //ros init
    ros::init(argc, argv, "bot_comm_nrf");
    ros::NodeHandle n;
    ros::Subscriber subCmd = n.subscribe("/grsim_data", 1000, CallBack);
	
    

	ros::spin();
    return 0;
}
