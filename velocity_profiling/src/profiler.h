#ifndef _PROFILER_
#define _PROFILER_

#include <cmath>
#include "krssg_ssl_msgs/point_2d.h"
#include "krssg_ssl_msgs/profilerOutput.h"
#include <ssl_common/config.h>


using namespace std;
class profiler
{
public:
	/**
	 * @brief      Constructor for velocity profiling
	 *
	 * @param[in]  pathArray       Vector of points on path
	 * @param[in]  startTime  Starting Time
	 * @param[in]  currPos    Current Position of kub
	 */
	profiler(std::vector<std::vector<krssg_ssl_msgs::point_2d> > pathArray, 
			 double startTime, krssg_ssl_msgs::point_2d currPos){
		/**
		 * Number of path used for replanning
		 */
		numPath = 3;
		startTime = startTime;
		pathArray = pathArray;
		currPos = currPos;
		distanceTraversed = 0;
		vMax = MAX_BOT_SPEED;
		aMax = MAX_BOT_LINEAR_ACC;
		updatePathLength();
		updateAngle();
		// TODO
		// startSpeed;
		// finalSpeed;
	};
	~profiler(){};
	krssg_ssl_msgs::profilerOutput sendStop(){
		krssg_ssl_msgs::profilerOutput output;
		output.velX = 0;
		output.velY = 0;
		output.errorX = 0;
		output.errorY = 0;
		return output;
	}

	krssg_ssl_msgs::profilerOutput sendVelocity(int pathIndex){
		double v = velocity/1000.0;
		krssg_ssl_msgs::profilerOutput output;
		output.velX = v*cos(motionAngle[positionIndex]);
		output.velY = v*sin(motionAngle[positionIndex]);
		output.errorX = pathArray[pathIndex][positionIndex].x - currPos.x;
		output.errorY = pathArray[pathIndex][positionIndex].y - currPos.y;

		return output;
	}
	// TODO return True/False and pathIndex
	bool shouldReplan(){

	};
private:
	double startTime;
	int numPath;
	std::vector<std::vector<krssg_ssl_msgs::point_2d> > pathArray;
	/**
	 * @brief stores current position of kub
	 * @see trapezoidalMotion()
	 */
	krssg_ssl_msgs::point_2d currPos;
	/**
	 * @brief distance travelled by path
	 * @see trapezoidalMotion()
	 */
	double distanceTraversed;
	/**
	 * @brief velocity needed, updated by trapezoidalMotion()
	 */
	double velocity;
	/**
	 * @brief      length of different available paths
	 * @see 	   numPath
	 */		
	std::vector<double> pathLength(numPath);

	/**
	 * @brief      angle at each index on path having index pathIndex
	 * @see updateAngle()
	 */
	std::vector<std::vector<double> > motionAngle(numPath);
	/**
	 * @brief Max allowed speed,defined in config.h
	 */
	double vMax;
	/**
	 * @brief max allowed acceleration, defined in config.h
	 */
	double aMax;
	/**
	 * Initial speed
	 */
	double u;
	/**
	 * Final Speed
	 */
	double v;

	double dist(point A, point B)
	{
		double x = B.x - A.x; double y = B.y - A.y;
		return sqrt(x*x + y*y);
	}

	int GetExpectedPositionIndex(int pathIndex)
	{
		double distance = 0;
		for(int i=1;i<pathArray[pathIndex].size();i++)
		{
			distance += dist(pathArray[pathIndex][i], pathArray[pathIndex][i-1]);
			if(distance > distanceTraversed)
				return i;
		}
			return -1;
	}

	/**
	 * @brief      Update length of all path
	 *
	 */
	double updatePathLength()
	{
		for (int i = 0; i < numPath; ++i)
		{
			for(int j=1;j<pathArray[i].size();j++)
			{
				pathLength[i] += dist(pathArray[i][j], pathArray[i][j-1]);
			}
		}
	}

	/**
	 * @brief      get Time to travell "distance" on path of index
	 * pathIndex.
	 *
	 * @param[in]  pathIndex  Index of path to check
	 *
	 * @return     time needed
	 */
	double getTime(double distance, int pathIndex) {
		double startSpeed = fmin(u, maxSpeed);
		double finalSpeed = fmin(v, maxSpeed);
		double maxSpeed = vMax;
		double maxAcc = aMax;
		double rampUpTime = (maxSpeed - startSpeed) / maxAcc;
		double plateauTime;
		double rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;

		double rampUpDist = rampUpTime * (startSpeed + maxSpeed) / 2.0;
		double plateauDist;
		double rampDownDist = rampDownTime * (maxSpeed + finalSpeed) / 2.0;

		if (rampUpDist + rampDownDist > pathLength[pathIndex]) {
			maxSpeed = sqrt((2 * maxAcc * pathLength[pathIndex] + powf(startSpeed, 2) +
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
			plateauDist = pathLength[pathIndex] - (rampUpDist + rampDownDist);
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

	/**
	 * @brief      updates angle needed 
	 * @see   motionAngle
	 */
	void updateAngle(){
		for (int n = 0; n < numPath; ++n)
		{
			for (int i = 0; i < pathArray[n].size(); ++i)
			{
				if(i < pathArray[n].size() && i>0)
				{
					double dx = pathArray[n][i+1].x - pathArray[n][i-1].x;
					double dy = pathArray[n][i+1].y - pathArray[n][i-1].y;
					motionAngle[n].push_back(atan2(dy, dx));
				}
				else if(i==0)
				{
					double dx = pathArray[n][i+1].x - pathArray[n][i].x;
					double dy = pathArray[n][i+1].y - pathArray[n][i].y;
					motionAngle[n].push_back(atan2(dy, dx));
				}
				else
				{
					double dx = pathArray[n][i].x - pathArray[n][i-1].x;
					double dy = pathArray[n][i].y - pathArray[n][i-1].y;
					motionAngle[n].push_back(atan2(dy, dx));
				}
			}
		}
	}

	/**
	 * @brief      check if motion is possible.
	 *
	 * @param[in]  pathIndex    index of path in patharray
	 * @param[in]  timeIntoLap  time spent
	 *
	 * Also updates private varible velocity and distanceTraversed.
	 * @see velocity
	 * @see distanceTraversed
	 */
	bool trapezoidalMotion(int pathIndex,double timeIntoLap) {
		double startSpeed = fmin(u, vMax);
		double finalSpeed = fmin(v, vMax);
		double maxSpeed = vMax;
		double maxAcc = aMax;
		double rampUpTime = (maxSpeed - startSpeed) / maxAcc;
		double plateauTime;
		double rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;

		double rampUpDist = rampUpTime * (startSpeed + maxSpeed) / 2.0;
		double plateauDist;
		double rampDownDist = rampDownTime * (maxSpeed + finalSpeed) / 2.0;

		if (rampUpDist + rampDownDist > pathLength[pathIndex]) {
			// triangle case: we don't ever hit full speed
			maxSpeed = sqrt((2 * maxAcc * pathLength[pathIndex] + powf(startSpeed, 2) +
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
			plateauDist = pathLength[pathIndex] - (rampUpDist + rampDownDist);
			plateauTime = plateauDist / maxSpeed;
		}

		if (timeIntoLap < 0) {
			// not even started on the path yet
			distanceTraversed = 0;
			velocity = startSpeed;
			return false;
		} else if (timeIntoLap < rampUpTime) {
			// on the ramp-up, we're accelerating at @maxAcc
			distanceTraversed =
				0.5 * maxAcc * timeIntoLap * timeIntoLap + startSpeed * timeIntoLap;
			velocity = startSpeed + maxAcc * timeIntoLap;
			return true;
		} else if (timeIntoLap < rampUpTime + plateauTime) {
			// we're on the plateau
			distanceTraversed = rampUpDist + (timeIntoLap - rampUpTime) * maxSpeed;
			velocity = maxSpeed;
			return true;
		} else if (timeIntoLap < rampUpTime + plateauTime + rampDownTime) {
			// we're on the ramp down
			double timeIntoRampDown = timeIntoLap - (rampUpTime + plateauTime);
			distanceTraversed = 0.5 * (-maxAcc) * timeIntoRampDown * timeIntoRampDown +
				maxSpeed * timeIntoRampDown + (rampUpDist + plateauDist);
			velocity = maxSpeed - maxAcc * timeIntoRampDown;
			return true;
		} else {
			// past the end of the path
			distanceTraversed = pathLength[pathIndex];
			velocity = finalSpeed;
			return false;
		}
	}
};

#endif
