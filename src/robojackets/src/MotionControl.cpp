#include "MotionControl.hpp"
// #include <SystemState.hpp>
// #include <RobotConfig.hpp>
// #include <Robot.hpp>
#include <Utils.hpp>
// #include "TrapezoidalMotion.hpp"
#include <Geometry2d/Util.hpp>
#include <planning/MotionInstant.hpp>

#include <cmath>
#include <stdio.h>
#include <algorithm>
#include <Geometry2d/Point.hpp>
#include <planning/SingleRobotPathPlanner.hpp>
#include <iostream>
using namespace std;
using namespace Geometry2d;
using namespace Planning;

#pragma mark Config Variables

const float PIVOT_VEL_MULTIPLIER = 0.12;
const float ACCEL_MULTIPLIER = 0; // ?
const float ANGLE_VEL_MULTIPLIER = 0.15;
const float MIN_EFFECTIVE_ANGULAR_SPEED = 0.7;
const float MIN_EFFECTIVE_VEL = 0.3;
const float VEL_MULTIPLIER = 1.3;
REGISTER_CONFIGURABLE(MotionControl);

ConfigDouble* MotionControl::_max_acceleration;
ConfigDouble* MotionControl::_max_velocity;

void MotionControl::createConfiguration(Configuration* cfg) {
    printf("Mtion control config created!\n");
    _max_acceleration =
        new ConfigDouble(cfg, "MotionControl/Max Acceleration", 1.5);
    _max_velocity = new ConfigDouble(cfg, "MotionControl/Max Velocity", 2.0);
}

#pragma mark MotionControl

MotionControl::MotionControl() : _angleController(0, 0, 0, 50) {
    // _robot = robot;

    // _robot->radioTx.set_robot_id(_robot->shell());
    _lastCmdTime = -1;
}

Point MotionControl::motionError(Planning::Path* path,
                             Planning::MotionCommand *motionCommand,
                             Geometry2d::Point pos) {
    if (!path)
        return Point(0,0);
    MotionInstant target;

   // convert from microseconds to seconds
    float timeIntoPath =
        ((float)(RJ::timestamp() - path->startTime())) *
            TimestampToSecs +
        1.0 / 60.0;

    // evaluate path - where should we be right now?
    boost::optional<MotionInstant> optTarget =
        path->evaluate(timeIntoPath);
    if (!optTarget) {
        // use the path end if our timeIntoPath is greater than the duration
        target.vel = Point();
        target.pos = path->end().pos;
    } else {
        target = *optTarget;
    }
    // tracking error
    Point posError = target.pos - pos;
    return posError;
}
float MotionControl::angleError(Planning::RotationCommand* rotationCommand,
                         Geometry2d::Point pos,
                         float angle) {
    boost::optional<Geometry2d::Point> targetPt;
    switch (rotationCommand->getCommandType()) {
        case RotationCommand::FacePoint:
            targetPt = static_cast<const Planning::FacePointCommand*>(
                           rotationCommand)
                           ->targetPos;
                           //printf("got target pt from FacePoint = %f, %f!\n", targetPt->x, targetPt->y);
            break;
        case RotationCommand::None:
            // do nothing
            break;
        default:
            debugThrow("RotationCommand Not implemented");
            break;
    }

    if (targetPt) {
        // fixing the angle ensures that we don't go the long way around to get
        // to our final angle
        float targetAngleFinal = (*targetPt - pos).angle();
        float angleError = fixAngleRadians(targetAngleFinal - angle);
        return angleError;
    }
    return 0;

}

MotionWrapper MotionControl::run(  Planning::Path* path,
                                   Planning::RotationCommand* rotationCommand,
                                   Planning::MotionCommand *motionCommand,
                                   Geometry2d::Point pos,
                                   float angle,
                                   const MotionConstraints &motionConstraints,
                                   const RotationConstraints &rotationConstraints) {
    // if (!_robot) return;

    const MotionConstraints& constraints = motionConstraints;

    // update PID parameters
    _positionXController.kp = 5.0;
    _positionXController.ki = 0;
    _positionXController.setWindup(0);
    _positionXController.kd = 0;
    _positionYController.kp = 5;
    _positionYController.ki = 0;
    _positionYController.setWindup(0);
    _positionYController.kd = 0;
    _angleController.kp = 0.2;
    _angleController.ki = 0.002;
    _angleController.kd = 0.15;

    // Angle control //////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    MotionWrapper wrapper;
    float targetW = 0;

    boost::optional<Geometry2d::Point> targetPt;
    if (motionCommand->getCommandType() == MotionCommand::Pivot) {
        PivotCommand command = *static_cast<PivotCommand*>(motionCommand);
        targetPt = command.pivotTarget;
    }

    switch (rotationCommand->getCommandType()) {
        case RotationCommand::FacePoint:
            targetPt = static_cast<const Planning::FacePointCommand*>(
                           rotationCommand)
                           ->targetPos;
            break;
        case RotationCommand::None:
            // do nothing
            break;
        default:
            debugThrow("RotationCommand Not implemented");
            break;
    }

    if (targetPt) {
        // fixing the angle ensures that we don't go the long way around to get
        // to our final angle
        float targetAngleFinal = (*targetPt - pos).angle();
        float angleError = fixAngleRadians(targetAngleFinal - angle);

        targetW = _angleController.run(angleError);

        // limit W
        if (abs(targetW) > (rotationConstraints.maxSpeed)) {
            if (targetW > 0) {
                targetW = (rotationConstraints.maxSpeed);
            } else {
                targetW = -(rotationConstraints.maxSpeed);
            }
        }

        /*
        _robot->addText(QString("targetW: %1").arg(targetW));
        _robot->addText(QString("angleError: %1").arg(angleError));
        _robot->addText(QString("targetGlobalAngle: %1").arg(targetAngleFinal));
        _robot->addText(QString("angle: %1").arg(angle));
        */
    }

    wrapper.w = _targetAngleVel(targetW);

    // handle body velocity for pivot command
    if (motionCommand->getCommandType() == MotionCommand::Pivot) {
        float r = Robot_Radius;
        const float FudgeFactor = PIVOT_VEL_MULTIPLIER;
        float speed = r * targetW * RadiansToDegrees * FudgeFactor;
        Point vel(speed, 0);

        // the robot body coordinate system is wierd...
        vel.rotate(-M_PI_2);

        wrapper.vel = _targetBodyVel(vel);

        return wrapper;  // pivot handles both angle and position
    }

    // Position control ///////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////

    MotionInstant target;
    // debugging variablese
    Geometry2d::Point refVel, pidVel, clampedVel;
    // if no target position is given, we don't have a path to follow
    if (!path) {
        wrapper.vel = _targetBodyVel(Point(0, 0));
        return wrapper;
    } else {
        //
        // Path following
        //

        // convert from microseconds to seconds
        float timeIntoPath =
            ((float)(RJ::timestamp() - path->startTime())) *
                TimestampToSecs +
            1.0 / 60.0;

        // evaluate path - where should we be right now?
        boost::optional<MotionInstant> optTarget =
            path->evaluate(timeIntoPath);
        if (!optTarget) {
            // use the path end if our timeIntoPath is greater than the duration
            target.vel = Point();
            target.pos = path->end().pos;
        } else {
            target = *optTarget;
        }
        // tracking error
        Point posError = target.pos - pos;

        // acceleration factor
        Point acceleration;
        boost::optional<MotionInstant> nextTarget =
            path->evaluate(timeIntoPath + 1.0 / 60.0);
        if (nextTarget) {
            acceleration = (nextTarget->vel - target.vel) / 60.0f;
        } else {
            acceleration = {0, 0};
        }
        Point accelFactor =
            acceleration * 60.0f * (ACCEL_MULTIPLIER);

        target.vel += accelFactor;
        refVel = target.vel;
        // PID on position
        pidVel.x = _positionXController.run(posError.x);
        pidVel.y = _positionYController.run(posError.y);
        target.vel.x += pidVel.x;
        target.vel.y += pidVel.y;

        // draw target pt
        // _robot->state()->drawCircle(target.pos, .04, Qt::red, "MotionControl");
        // _robot->state()->drawLine(target.pos, target.pos + target.vel, Qt::blue,
        //                           "MotionControl");

        // convert from world to body coordinates
        target.vel = target.vel.rotated(-angle);
    }    
    // DEBUG: try removing the clapmping part and just set wrapper.vel to target.vel?
    // wrapper.vel = this->_targetBodyVel(target.vel);
    wrapper.vel = target.vel;
    clampedVel = wrapper.vel;
    // std::cout << "ref vel = " << refVel << " pid vel = " << pidVel << " clamped vel = " << clampedVel ;
    return wrapper;
}

// void MotionControl::stopped() {
//     _targetBodyVel(Point(0, 0));
//     _targetAngleVel(0);
// }

float MotionControl::_targetAngleVel(float angleVel) {
    // velocity multiplier
    angleVel *= ANGLE_VEL_MULTIPLIER;

    // convert units
    angleVel *= RadiansToDegrees;

    // If the angular speed is very low, it won't make the robot move at all, so
    // we make sure it's above a threshold value
    float minEffectiveAngularSpeed = MIN_EFFECTIVE_ANGULAR_SPEED;
    if (std::abs(angleVel) < minEffectiveAngularSpeed &&
        std::abs(angleVel) > 0.2) {
        angleVel =
            angleVel > 0 ? minEffectiveAngularSpeed : -minEffectiveAngularSpeed;
    }

    // the robot firmware still speaks degrees, so that's how we send it over
    return angleVel;
}

Point MotionControl::_targetBodyVel(Point targetVel) {
    // Limit Velocity
    targetVel.clamp(*_max_velocity);

    // Limit Acceleration
    if (_lastCmdTime == -1) {
        targetVel.clamp(*_max_acceleration);
    } else {
        float dt = (float)((RJ::timestamp() - _lastCmdTime) / 1000000.0f);
        Point targetAccel = (targetVel - _lastVelCmd) / dt;
        targetAccel.clamp(*_max_acceleration);

        targetVel = _lastVelCmd + targetAccel * dt;
    }

    // make sure we don't send any bad values
    if (isnan(targetVel.x) || isnan(targetVel.y)) {
        targetVel = Point(0, 0);
    }

    // track these values so we can limit acceleration
    _lastVelCmd = targetVel;
    _lastCmdTime = RJ::timestamp();

    // velocity multiplier
    targetVel *= VEL_MULTIPLIER;

    // if the velocity is nonzero, make sure it's not so small that the robot
    // doesn't even move
    float minEffectiveVelocity = MIN_EFFECTIVE_VEL;
    if (targetVel.mag() < minEffectiveVelocity && targetVel.mag() > 0.05) {
        targetVel = targetVel.normalized() * minEffectiveVelocity;
    }

    // set radioTx values
    return targetVel;
}
