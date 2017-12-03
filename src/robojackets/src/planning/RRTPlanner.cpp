#include "planning/RRTPlanner.hpp"
// #include "EscapeObstaclesPathPlanner.hpp"
#include <Constants.hpp>
#include <Utils.hpp>
// #include <protobuf/LogFrame.pb.h>
#include "TrapezoidalMotion.hpp"
#include "planning/Util.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>

#include <ros/console.h>

using namespace std;
using namespace Eigen;

namespace Planning {

RRTPlanner::RRTPlanner(int maxIterations) : _maxIterations(maxIterations) {}

bool RRTPlanner::shouldReplan(MotionInstant start, MotionInstant goal,
                              const MotionConstraints& motionConstraints,
                              const Geometry2d::ShapeSet* obstacles,
                              const Path* prevPath) const {
    if (SingleRobotPathPlanner::shouldReplan(start, motionConstraints,
                                             obstacles, prevPath)) {
        return true;
    }

    // if the destination of the current path is greater than X m away
    // from the target destination, we invalidate the path. This
    // situation could arise if the path destination changed.
    float goalPosDiff = (prevPath->end().pos - goal.pos).mag();
    float goalVelDiff = (prevPath->end().vel - goal.vel).mag();
    if (goalPosDiff > goalChangeThreshold() ||
        goalVelDiff > goalChangeThreshold()) {
        // FIXME: goalChangeThreshold shouldn't be used for velocities as it
        // is above
        return true;
    }

    return false;
}

std::unique_ptr<Path> RRTPlanner::run(
    MotionInstant start, const MotionCommand* cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, std::unique_ptr<Path> prevPath) {
    // This planner only works with commands of type 'PathTarget'
    assert(cmd->getCommandType() == Planning::MotionCommand::PathTarget);
    Planning::PathTargetCommand target =
        *static_cast<const Planning::PathTargetCommand*>(cmd);

    MotionInstant goal = target.pathGoal;

    // Simple case: no path
    if (start.pos == goal.pos) {
        InterpolatedPath* path = new InterpolatedPath();
        path->setStartTime(RJ::timestamp());
        path->waypoints.emplace_back(
            MotionInstant(start.pos, Geometry2d::Point()), 0);
        return unique_ptr<Path>(path);
    }

    // Locate a goal point that is obstacle-free
    // boost::optional<Geometry2d::Point> prevGoal;
    // if (prevPath) prevGoal = prevPath->end().pos;
    // goal.pos = EscapeObstaclesPathPlanner::findNonBlockedGoal(
    //     goal.pos, prevGoal, *obstacles);

    // Replan if needed, otherwise return the previous path unmodified
    if (shouldReplan(start, goal, motionConstraints, obstacles,
                     prevPath.get())) {
        // Run bi-directional RRT to generate a path.
        long long tstart = RJ::timestamp();
        InterpolatedPath* path =
            runRRT(start, goal, motionConstraints, obstacles);
        long long timetaken = RJ::timestamp() - tstart;
        float pathlength = path->length();
        float pathtime = path->getDuration();
        // just apend to a file...
        FILE *f = fopen("/tmp/log-rrt.txt", "a");
        fprintf(f, "%lld %f %f\n", timetaken, pathlength, pathtime);
        fclose(f);
        // If RRT failed, the path will be empty, so we need to add a single
        // point to make it valid.
        if (path && path->waypoints.empty()) {
            ROS_WARN("new path has no waypoints!!");
            path->waypoints.emplace_back(
                MotionInstant(start.pos, Geometry2d::Point()), 0);
        }
        return unique_ptr<Path>(path);
    } else {
        return prevPath;
    }
}

InterpolatedPath* RRTPlanner::runRRT(MotionInstant start, MotionInstant goal,
                                     const MotionConstraints& motionConstraints,
                                     const Geometry2d::ShapeSet* obstacles) {
    InterpolatedPath* path = new InterpolatedPath();
    path->setStartTime(RJ::timestamp());

    // Initialize two RRT trees
    FixedStepTree startTree;
    FixedStepTree goalTree;
    startTree.init(start.pos, obstacles);
    goalTree.init(goal.pos, obstacles);
    startTree.step = goalTree.step = .15f;

    // Run bi-directional RRT algorithm
    Tree* ta = &startTree;
    Tree* tb = &goalTree;
    for (unsigned int i = 0; i < _maxIterations; ++i) {
        Geometry2d::Point r = RandomFieldLocation();

        Tree::Point* newPoint = ta->extend(r);

        if (newPoint) {
            // try to connect the other tree to this point
            if (tb->connect(newPoint->pos)) {
                // trees connected
                // done with global path finding
                // the path is from start to goal
                // runRRT will handle the rest
                break;
            }
        }

        swap(ta, tb);
    }

    Tree::Point* p0 = startTree.last();
    Tree::Point* p1 = goalTree.last();

    // sanity check
    if (!p0 || !p1 || p0->pos != p1->pos) {
        return path;
    }

    // extract path from RRTs
    // add the start tree first...normal order (aka from root to p0)
    startTree.addPath(*path, p0);
    // add the goal tree in reverse (aka p1 to root)
    goalTree.addPath(*path, p1, true);
    ROS_INFO_NAMED("path", "non optimized path size = %lu", path->size());
    path = optimize(*path, obstacles, motionConstraints, start.vel, goal.vel);
    // ROS_INFO_NAMED("path", "optimized path size = %lu", path->size());
    return path;
}



float getTime(InterpolatedPath& path, int index,
              const MotionConstraints& motionConstraints, float startSpeed,
              float endSpeed) {
    return Trapezoidal::getTime(
        path.length(0, index), path.length(), motionConstraints.maxSpeed,
        motionConstraints.maxAcceleration, startSpeed, endSpeed);
}



}  // namespace Planning
