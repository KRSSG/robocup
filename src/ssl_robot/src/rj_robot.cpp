#include <iostream>
#include <cstring>
#include <ctime>
#include "krssg_ssl_msgs/BeliefState.h"
#include "tactics/tactic_factory.h"
#include <stdio.h>

#include "ros/ros.h"
#include "rj_robot.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>
using namespace std;


namespace Strategy {

  RJRobot::RJRobot(int botID, ros::NodeHandle &n): botID(botID), 
                                              gotTacticPacket(false), 
                                              n(n),
                                              commandPub(n.advertise<krssg_ssl_msgs::gr_Commands>("/grsim_data", 1000)),
                                              debugPub(n.advertise<krssg_ssl_msgs::sslDebug_Data>("/grsim_debug_data", 1000)),
                                              planner(Planning::PlannerForCommandType(Planning::MotionCommand::PathTarget)),
                                              controller(new MotionControl()),
                                              path(nullptr)
  {
    // lets make the default tactic = TPosition, pos = (0,0)
    curTactic = TacticFactory::instance()->Create("TPosition", botID);
    // tParam should already be 0,0,0 no need to set again.
    using namespace Planning;
    std::shared_ptr<Configuration> config =
          Configuration::FromRegisteredConfigurables();
  }

  RJRobot::~RJRobot() {}

  void RJRobot::beliefStateCallback(const krssg_ssl_msgs::BeliefState::ConstPtr &bs) {
    using namespace krssg_ssl_msgs;
    // printf("got beliefState\n");
    using namespace Planning;
    using Geometry2d::Point;
    static MotionInstant prevInstant = MotionInstant();
    // printf("bot %d: (%f, %f)\n", botID, bs->homePos[botID].x, bs->homePos[botID].y);
    
    Geometry2d::ShapeSet obstacles;
    for (int i = 0; i < 6; i++) {
      if (i!=botID) {
        Point robot_pos(bs->homePos[i].x/1000., bs->homePos[i].y/1000.);
        obstacles.add(std::shared_ptr<Geometry2d::Shape>(new Geometry2d::Circle(robot_pos, BOT_RADIUS/1000.*2)));
      }
    }
    for (int i = 0; i < 6; i++) {
      Point robot_pos(bs->awayPos[i].x/1000., bs->awayPos[i].y/1000.);
      obstacles.add(std::shared_ptr<Geometry2d::Shape>(new Geometry2d::Circle(robot_pos, BOT_RADIUS/1000.*2)));
    }
    Point ball_pos(bs->ballPos.x/1000., bs->ballPos.y/1000.);
    obstacles.add(std::shared_ptr<Geometry2d::Shape>(new Geometry2d::Circle(ball_pos, (BALL_RADIUS/1000.)*0.9)));
    // printf("obstacles size = \n");
    Point robot_pos(bs->homePos[botID].x/1000., bs->homePos[botID].y/1000.);
    // destination point behind ball
    Point goal_pos(HALF_FIELD_MAXX/1000.0, 0);
    Point diff_scaled = (ball_pos - goal_pos);
    Point dpoint = ball_pos + (diff_scaled)/diff_scaled.mag()*BOT_RADIUS/1000.*2;
    
    MotionInstant mi(dpoint,{0,0});
    std::unique_ptr<MotionCommand> mc(new PathTargetCommand(mi));
    prevInstant = MotionInstant(robot_pos, prevInstant.vel);
    path = planner->run(prevInstant, mc.get(), MotionConstraints(), &obstacles, std::move(path));
    std::unique_ptr<RotationCommand> rc(new FacePointCommand(goal_pos));
    // printf("rc pos = %f, %f\n", rc->targetPos.x, rc->targetPos.y);
    float robot_angle = bs->homePos[botID].theta;
    MotionConstraints motionConstraints;
    RotationConstraints rotationConstraints;
    MotionWrapper w = controller->run(path.get(), rc.get(), mc.get(), robot_pos, robot_angle, motionConstraints, rotationConstraints);
    prevInstant = MotionInstant(robot_pos, w.vel);
    // gr_Robot_Command robot_command = curTactic->execute(*bs, curParam);
    gr_Robot_Command robot_command = Strategy::getRobotCommandMessage(botID, -w.vel.y*1000., w.vel.x*1000., w.w, 0, 0);
    gr_Commands command;
    command.robot_commands = robot_command;
    command.timestamp = ros::Time::now().toSec();
    command.isteamyellow = bs->isteamyellow;
    commandPub.publish(command);

    float angleError, motionError;
    motionError = MotionControl::motionError(path.get(), mc.get(), robot_pos).mag();
    angleError = MotionControl::angleError(rc.get(), robot_pos, robot_angle);
    // printf("motionError, angleError = (%f, %f)\n", motionError, angleError);
    // motion max = 0.099, angle max = 0.61
    motionError = motionError/0.099 > 1? 1: motionError/0.099;
    angleError = angleError/0.61 > 1? 1: angleError/0.61;

    float color = Point(motionError, angleError).mag()/sqrt(2.);
    // make the debug lines and circles
    sslDebug_Data msg;
    msg.id = ros::this_node::getName();

    // bot pos circle
    sslDebug_Circle c_start;
    c_start.x = robot_pos.x*1000.;
    c_start.y = robot_pos.y*1000.;
    c_start.radius = BOT_RADIUS*1.1;
    c_start.color = color;
    msg.circle.push_back(c_start);

    // destination circle
    sslDebug_Circle c_dest;
    c_dest.x = dpoint.x*1000.;
    c_dest.y = dpoint.y*1000.;
    c_dest.radius = BOT_RADIUS*1.1;
    c_dest.color = color;
    msg.circle.push_back(c_dest);


    // lines for waypoints
    // Get the closest step size to a desired value that is divisible into the
    //     // duration
    const float duration = path->getDuration();
    const float desiredStep =
        0.25;  // draw the path by interpolating every x seconds
    const float segmentCount = roundf(duration / desiredStep);
    const float step = duration / segmentCount;
    Point prevPoint = path->start().pos;
    // Draw points along the path except the last one
    for (int i = 0; i < segmentCount; ++i) {
        float t = i * step;
        Point pt = path->evaluate(t)->pos;
        sslDebug_Line l;
        l.x1 = prevPoint.x*1000.;
        l.y1 = prevPoint.y*1000.;
        l.x2 = pt.x*1000.;
        l.y2 = pt.y*1000.;
        l.color = color;
        msg.line.push_back(l);
        prevPoint = pt;
    }

    // Draw the last point of the path
    {
      Point pt = path->end().pos;
      sslDebug_Line l;
      l.x1 = prevPoint.x*1000.;
      l.y1 = prevPoint.y*1000.;
      l.x2 = pt.x*1000.;
      l.y2 = pt.y*1000.;
      l.color = color;
      msg.line.push_back(l);
      prevPoint = pt;
    }
    debugPub.publish(msg);
  }
  void RJRobot::tacticPacketCallback(const krssg_ssl_msgs::TacticPacket::ConstPtr& tp) {
  //   printf("got tactic packet for bot (%d), tactic = (%s)\n", botID, tp->tID.c_str());
  //   tParamJSON = tp->tParamJSON;
  //   tID = tp->tID;
  //   gotTacticPacket =  true;
  }

}

