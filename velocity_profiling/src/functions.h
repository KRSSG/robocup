#include <bits/stdc++.h>
#include "ros/ros.h"
#include "krssg_ssl_msgs/planner_path.h"
#include "krssg_ssl_msgs/point_2d.h"
#include "krssg_ssl_msgs/point_SF.h"
#include "krssg_ssl_msgs/gr_Commands.h"
#include "krssg_ssl_msgs/gr_Robot_Command.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include "krssg_ssl_msgs/pid_message.h"
#include "krssg_ssl_msgs/replan.h"
using namespace std;

#define point krssg_ssl_msgs::point_2d
vector<point> home_pos(6);
vector<point> away_pos(6);
vector<point> path_points;
// krssg_ssl_msgs::point_2d myLastPos;
vector<double> vel_angle;
vector<double> home_pos_theta(6);
double path_length, distance_traversed, out_speed;
point start_point, goal_point;
int ExpectedPosIndex;
double start_time, curr_time, ExpectedTraverseTime;
const double MAX_SPEED = 4000, MAX_ACC = 1000, START_SPEED = 1000, FINAL_SPEED = 0, PI = 3.14159265359;
bool PATH_RECEIVED = false;

double dist(point A, point B)
{
  double x = B.x - A.x; double y = B.y - A.y;
  // x *= 160/13;
  // y *= 40/3;
  return sqrt(x*x + y*y);
}

int GetExpectedPositionIndex(double distance_traversed)
{
  double distance = 0;
  if (distance_traversed==0)
  {
    return 0;
  }
  for(int i=1;i<path_points.size();i++)
  {
    distance += dist(path_points[i], path_points[i-1]);
    if(distance>distance_traversed)
      return i;
  }
  return -1;
}

double GetPathLength()
{
  double path_length = 0;
  for(int i=1;i<path_points.size();i++)
  {
    path_length += dist(path_points[i], path_points[i-1]);
  }
  cout<<"in GetPathLength, pathLength = "<<path_length<<endl;
  return path_length;
}

double getTime(double distance, double pathLength, double maxSpeed,
    double maxAcc, double startSpeed,
    double finalSpeed) {
  cout<<"pathLength = "<<pathLength<<endl;
  startSpeed = fmin(startSpeed, maxSpeed);
  finalSpeed = fmin(finalSpeed, maxSpeed);
  double rampUpTime = (maxSpeed - startSpeed) / maxAcc;
  double plateauTime;
  double rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;

  double rampUpDist = rampUpTime * (startSpeed + maxSpeed) / 2.0;
  double plateauDist;
  double rampDownDist = rampDownTime * (maxSpeed + finalSpeed) / 2.0;

  if (rampUpDist + rampDownDist > pathLength) {
    // triangle case: we don't ever hit full speed
    maxSpeed = sqrt((2 * maxAcc * pathLength + powf(startSpeed, 2) +
          powf(finalSpeed, 2)) /
        2.0);

    rampUpTime = (maxSpeed - startSpeed) / maxAcc;
    rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;
    rampUpDist = (startSpeed + maxSpeed) / 2.0 * rampUpTime;
    rampDownDist = (finalSpeed + maxSpeed) / 2.0 * rampDownTime;

    // no plateau
    plateauTime = 0;
    plateauDist = 0;

  } else {
    // trapezoid case: there's a time where we go at maxSpeed for a bit
    plateauDist = pathLength - (rampUpDist + rampDownDist);
    plateauTime = plateauDist / maxSpeed;
  }

  if (distance <= 0) {
    return 0;
  }

  if (abs(distance - (rampUpDist + plateauDist + rampDownDist)) < 0.00001) {
    return rampUpTime + plateauTime + rampDownTime;
  }
  if (distance < rampUpDist) {
    // time calculations
    /*
       1/2*a*t^2 + t*v0 - d = 0
       t = -b +- sqrt(b^2 - 4*a*c)/(2*a)
       */
    double b = startSpeed;
    double a = maxAcc / 2.0;
    double c = -distance;
    double root = sqrt(b * b - 4 * a * c);
    double temp1 = (-b + root) / (2 * a);
    double temp2 = (-b - root) / (2 * a);
    if (std::isnan(root)) {
      throw std::invalid_argument("TrapezoidalMotion failed. Solution is imaginary");
      // Handle
      // this
      return rampUpTime;
    }
    if (temp1 > 0 && temp1 < rampUpTime) {
      return temp1;
    } else {
      return temp2;
    }
  } else if (distance <= rampUpDist + plateauDist) {
    double position = distance - rampUpDist;
    return rampUpTime + position / maxSpeed;
  } else if (distance < rampUpDist + plateauDist + rampDownDist) {
    // time calculations
    /*
       1/2*a*t^2 + t*v0 - d = 0
       t = -b +- sqrt(b^2 - 4*a*c)/(2*a)
       */
    double position = distance - rampUpDist - plateauDist;
    double b = maxSpeed;
    double a = -maxAcc / 2.0;
    double c = -position;
    double root = sqrt(b * b - 4 * a * c);
    double temp1 = (-b + root) / (2 * a);
    double temp2 = (-b - root) / (2 * a);
    if (std::isnan(root)) {
      throw std::invalid_argument("TrapezoidalMotion failed. Solution is imaginary");
      // Handle
      // this, AB KYA KRU !!
      return rampUpTime + plateauTime + rampDownTime;
    }
    if (temp1 > 0 && temp1 < rampDownTime) {
      return rampUpTime + plateauTime + temp1;
    } else {
      return rampUpTime + plateauTime + temp2;
    }
  } else {
    return rampUpTime + plateauTime + rampDownTime;
  }
}

bool TrapezoidalMotion(double pathLength, double maxSpeed, double maxAcc,
    double timeIntoLap, double startSpeed, double finalSpeed,
    double& posOut, double& speedOut) {

  startSpeed = fmin(startSpeed, maxSpeed);
  finalSpeed = fmin(finalSpeed, maxSpeed);
  double rampUpTime = (maxSpeed - startSpeed) / maxAcc;
  double plateauTime;
  double rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;

  double rampUpDist = rampUpTime * (startSpeed + maxSpeed) / 2.0;
  double plateauDist;
  double rampDownDist = rampDownTime * (maxSpeed + finalSpeed) / 2.0;

  if (rampUpDist + rampDownDist > pathLength) {
    // triangle case: we don't ever hit full speed
    maxSpeed = sqrt((2 * maxAcc * pathLength + powf(startSpeed, 2) +
          powf(finalSpeed, 2)) /
        2.0);

    rampUpTime = (maxSpeed - startSpeed) / maxAcc;
    rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;
    rampUpDist = (startSpeed + maxSpeed) / 2.0 * rampUpTime;
    rampDownDist = (finalSpeed + maxSpeed) / 2.0 * rampDownTime;

    // no plateau
    plateauTime = 0;
    plateauDist = 0;
  } else {
    // trapezoid case: there's a time where we go at maxSpeed for a bit
    plateauDist = pathLength - (rampUpDist + rampDownDist);
    plateauTime = plateauDist / maxSpeed;
  }

  if (timeIntoLap < 0) {
    /// not even started on the path yet
    posOut = 0;
    speedOut = startSpeed;
    return false;
  } else if (timeIntoLap < rampUpTime) {
    /// on the ramp-up, we're accelerating at @maxAcc
    posOut =
      0.5 * maxAcc * timeIntoLap * timeIntoLap + startSpeed * timeIntoLap;
    speedOut = startSpeed + maxAcc * timeIntoLap;
    return true;
  } else if (timeIntoLap < rampUpTime + plateauTime) {
    /// we're on the plateau
    posOut = rampUpDist + (timeIntoLap - rampUpTime) * maxSpeed;
    speedOut = maxSpeed;
    return true;
  } else if (timeIntoLap < rampUpTime + plateauTime + rampDownTime) {
    /// we're on the ramp down
    double timeIntoRampDown = timeIntoLap - (rampUpTime + plateauTime);
    posOut = 0.5 * (-maxAcc) * timeIntoRampDown * timeIntoRampDown +
      maxSpeed * timeIntoRampDown + (rampUpDist + plateauDist);
    speedOut = maxSpeed - maxAcc * timeIntoRampDown;
    return true;
  } else {
    /// past the end of the path
    posOut = pathLength;
    speedOut = finalSpeed;
    return false;
  }
}

bool trapezoid(double t, double& posOut, double& speedOut){

  // cout<<"t = "<<t<<" MAX_SPEED = "<<MAX_SPEED<<endl;
  bool valid = TrapezoidalMotion(path_length, MAX_SPEED, MAX_ACC, t, START_SPEED, FINAL_SPEED, posOut, speedOut);

  // if(valid){
  //   auto time = getTime(posOut, pathLength, maxSpeed, maxAcc,
  //       startSpeed, finalSpeed);
  // }
  return valid;
}
