#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/geometry.hpp>
#include <ssl_common/config.h>
#include <fstream>
/*#include "ros/ros.h"
#include "std_msgs/String.h"
#include "grSim_Packet.pb.h"
#include "grSim_Replacement.pb.h"
#include "grSim_Commands.pb.h"
#include "sslDebug_Data.pb.h"
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <string.h>
#include <QtGui/QMainWindow>
#include <QtGui/QDialog>
#include <QtNetwork>*/


#define FRICTION_COEFF (0.02)
#define ACCN_GRAVITY (9.80665)
#define ANGLE_THRES (30.0)

/*QUdpSocket udpsocket;
QHostAddress _addr = QHostAddress("127.0.0.1");
quint16 _port = 20011;*/

namespace Strategy{
	gr_Robot_Command SkillSet::dribbleTurn(const SParam &param, const BeliefState &state, int botID)
	{
		Vector2D<int> point(param.DribbleTurnP.x, param.DribbleTurnP.y);
		Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
    	Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
    	float radius = param.DribbleTurnP.turn_radius;
    	float finalSlope = Vector2D<int>::angle(point, ballPos);
    	float turnAngleLeft = normalizeAngle((finalSlope - state.homePos[botID].theta)); // Angle left to turn
    //	float omega = turnAngleLeft * param.DribbleTurnP.max_omega / (2 * PI);
        float omega;
        if(turnAngleLeft>=-ANGLE_THRES*PI/180&&turnAngleLeft<=ANGLE_THRES*PI/180)
            omega = ((turnAngleLeft)*180/ (ANGLE_THRES*PI)) * param.DribbleTurnP.max_omega;
        else
            omega=param.DribbleTurnP.max_omega*(turnAngleLeft<0?-1:1);
    	float phi = normalizeAngle(atan2(omega*omega*radius,FRICTION_COEFF*ACCN_GRAVITY));//*(omega<0?(-1):1));
     //   std::fstream f;
     //   f.open("/home/animesh/Documents/robocup/a.txt",std::ios::out|std::ios::app);
     //   f<<"Phi: "<<phi*180/(PI)<<" Angle left: "<<turnAngleLeft*180/(PI)<<"\n";
     //   f.close();
    	float sphi=sin(phi),cphi=cos(phi),v_x,v_y;
        if(omega>=0)
        {
    	   v_x=-omega*radius*cphi;
           v_y=omega*radius*sphi;
        }
        else {
            v_x=-omega*radius*cphi;
            v_y=-omega*radius*sphi;
        }


        //DEBUG CIRCLE
     /*   GOOGLE_PROTOBUF_VERIFY_VERSION;
        grSim_Packet packet;
        packet.mutable_debuginfo()->set_id(ros::this_node::getName());
        Debug_Circle *circle = packet.mutable_debuginfo()->add_circle();
        Debug_Line *line = packet.mutable_debuginfo()->add_line();
        circle->set_x(state.homePos[botID].x+radius*cos(state.homePos[botID].theta-phi+PI/2));
        circle->set_y(state.homePos[botID].y+radius*sin(state.homePos[botID].theta-phi+PI/2));
        circle->set_radius(radius);
        circle->set_color(0);
        QByteArray dgram;
        dgram.resize(packet.ByteSize());
        packet.SerializeToArray(dgram.data(), dgram.size());
        udpsocket.writeDatagram(dgram, _addr, _port);*/

    	return getRobotCommandMessage(botID, v_x, v_y, omega, 0, true);
	}
}