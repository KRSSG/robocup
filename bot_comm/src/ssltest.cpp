#include <stdio.h>
#include "serial.h"
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <fstream>
#include <time.h>
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
#define _DRIBBLER   1<<1
#define _KICKER     1<<0

using namespace std;

const int TEAM_ID = 127;  //blue
const double Radius=0.087; //bot radius
const double radius=0.025; //wheel radius
const double PI=3.141592653589793;
double max_vel_wheel=5000.0; //this is in rpm
double theta[4]={30.0,150.0,225.0,315.0};

HAL::Serial serial;

void vel_convert(const double* vel_xyw, double* vel_wheel)
{
    double vx=vel_xyw[0];
    double vy=vel_xyw[1];
    double vw=vel_xyw[2];
    vw*=-1;
    int i;
    vel_wheel[0] =   (( (Radius * vw) - (vx * sin(theta[0])) + (vy * cos(theta[0]))) )/radius;
    vel_wheel[1] =   (( (Radius * vw) - (vx * sin(theta[1])) + (vy * cos(theta[1]))) )/radius;
    vel_wheel[2] =   (( (Radius * vw) - (vx * sin(theta[2])) + (vy * cos(theta[2]))) )/radius;
    vel_wheel[3] =   (( (Radius * vw) - (vx * sin(theta[3])) + (vy * cos(theta[3]))) )/radius;

    for (i = 0; i < 4; i++)
        vel_wheel[i]=vel_wheel[i]/PI;
    
    for (i = 0; i < 4; i++){        
        if(vel_wheel[i]>0)
            vel_wheel[i] = 126 + ((vel_wheel[i]-max_vel_wheel)*(126.0))/max_vel_wheel;
        else
            vel_wheel[i] = 256 - ((vel_wheel[i]-0)*(129.0-256.0))/max_vel_wheel;
    }
}

void CallBack(int ID, double* vel_wheel)
{
    unsigned static char buf[32];
    buf[0] = TEAM_ID;
    int _start = 1+ID*5, c=0;
    for(int i=_start;i<_start + 4;i++, c++)
        buf[i] = (int)vel_wheel[c];
    buf[_start + 4] = 0;
    for(int i=0;i<32;i++){
        cout<<(int)buf[i]<<" ";
    }
    cout<<endl;
    serial.Write(buf,32);
}

int main(int argc, char *argv[])
{
    //Converting angles to radians
    for (int i = 0; i < 4; ++i)
        theta[i]=theta[i]*PI/180;

    if(!serial.Open("/dev/ttyUSB0", 230400)) {

        if(!serial.Open("/dev/ttyUSB1", 230400)) {
            printf("Could not open the fucking parts 1 .\n");
            exit(0);
        }
        printf("Could not open the fucking parts 0 .\n");
        exit(0);
    }

    double vel_wheel[4];
   /* for(int i=0;i<4;i++)
        cin>>vel_wheel[i];
    while(1){
        CallBack(0, vel_wheel);
    }*/
    int c = 0;
    double d1[4] = {0,0,0,0}, d2[4] = {20, 20, 20, 20}, d3[4] = {229, 0, 27, 0};
/*
    while(c<4009){
        CallBack(1, d1);
        c++;
    }
    c = 0;
    while(c < 4000){
        CallBack(1, d2);
        c++;
    }
    c = 0;
    while(c < 4000) {
        CallBack(1, d3);
        c++;
    }
    while(1){
        CallBack(1, d1);
    }*/

    double v3d[3] = {0, 0, 0},drib;
    double wheel4d[4];
    using namespace std;
     std::cin>>v3d[0]>>v3d[1]>>v3d[2]>>drib;

     vel_convert(v3d, wheel4d);
    
    unsigned static char buf[32];
    buf[0] = TEAM_ID;

    for(int i=1;i<5;i++)
        buf[i] = (int)wheel4d[i-1];

    buf[5] = drib;


    cout<<endl;
    for(int i=0;i<32;i++){
        cout<<(int)buf[i]<<" ";
    }

    while(1) {
        serial.Write(buf,32);
        for(int i=0;i<32;i++){
            cout<<(int)buf[i]<<" ";
        }
        system("sleep 0.1s");
        cout<<endl;
    }
    return 0;
}
