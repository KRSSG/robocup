#pragma once

#include <robojackets/Configuration.hpp>
#include <robojackets/Geometry2d/Point.hpp>
#include <robojackets/Pid.hpp>
#include <robojackets/Geometry2d/Point.hpp>
#include <robojackets/planning/SingleRobotPathPlanner.hpp>
#include <robojackets/planning/RotationCommand.hpp>
#include <robojackets/planning/MotionCommand.hpp>
#include <robojackets/planning/MotionConstraints.hpp>
#include <robojackets/planning/RotationConstraints.hpp>
#include <robojackets/RobotConfig.hpp>
class OurRobot;

/**
 * @brief Handles computer-side motion control
 * @details It is responsible for most of what gets sent out in a RadioTx
 *     packet. The MotionControl object is given an OurRobot at initialization
 *     and from then on will set the values in that robot's RadioTx packet
 *     directly whenever run() or stopped() is called.
 */

 struct MotionWrapper {
    Geometry2d::Point vel;
    float w;
 };
class MotionControl {
public:
    MotionControl();

    /**
     * Stops the robot.
     * The robot will decelerate at max acceleration until it stops.
     */
    // void stopped();

    /**
     * This runs PID control on the position and angle of the robot 
     */
    MotionWrapper run(Planning::Path* path,
                      Planning::RotationCommand* rotationCommand,
                      Planning::MotionCommand *motionCommand,
                      Geometry2d::Point pos,
                      float angle,
                      const MotionConstraints &motionConstraints,
                      const RotationConstraints &rotationConstraints);

    static void createConfiguration(Configuration* cfg);
    static Geometry2d::Point motionError(Planning::Path* path,
                             Planning::MotionCommand *motionCommand,
                             Geometry2d::Point pos);
    static float angleError(Planning::RotationCommand* rotationCommand,
                            Geometry2d::Point pos,
                            float angle);
private:
    // sets the target velocity in the robot's radio packet
    // this method is used by both run() and stopped() and does the
    // velocity and acceleration limiting and conversion to robot velocity
    //"units"
    Geometry2d::Point _targetBodyVel(Geometry2d::Point targetVel);

    /// sets the target angle velocity in the robot's radio packet
    /// does velocity limiting and conversion to robot velocity "units"
    float _targetAngleVel(float angleVel);

    // OurRobot* _robot;

    /// The last velocity (in m/s, not the radioTx value) command that we sent
    /// to the robot
    Geometry2d::Point _lastVelCmd;

    /// the time in microseconds when the last velocity command was sent
    long _lastCmdTime;

    Pid _positionXController;
    Pid _positionYController;
    Pid _angleController;

    static ConfigDouble* _max_acceleration;
    static ConfigDouble* _max_velocity;
};
