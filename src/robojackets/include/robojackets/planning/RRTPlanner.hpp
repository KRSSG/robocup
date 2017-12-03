#pragma once

#include "robojackets/planning/SingleRobotBezierPathPlanner.hpp"
#include "robojackets/planning/Tree.hpp"
#include <robojackets/Geometry2d/ShapeSet.hpp>
#include <robojackets/Geometry2d/Point.hpp>
#include <robojackets/planning/InterpolatedPath.hpp>
#include <robojackets/planning/MotionCommand.hpp>
#include <robojackets/planning/MotionConstraints.hpp>
#include <robojackets/planning/MotionInstant.hpp>

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <list>

namespace Planning {

/**
 * @brief Given a start point and an end point and some conditions, plans a path
 * for a robot to get there.
 *
 * @details There are many ways to plan paths.  This planner uses bidirectional
 * [RRTs](http://en.wikipedia.org/wiki/Rapidly-exploring_random_tree).
 * You can check out our interactive RRT applet on GitHub here:
 * https://github.com/RoboJackets/rrt.
 */
class RRTPlanner : public SingleRobotBezierPathPlanner {
public:
    RRTPlanner(int maxIterations);
    /**
     * gets the maximum number of iterations for the RRT algorithm
     */
    int maxIterations() const { return _maxIterations; }
    /**
     * sets the maximum number of iterations for th RRT algorithm
     */
    void maxIterations(int value) { _maxIterations = value; }

    MotionCommand::CommandType commandType() const override {
        return MotionCommand::PathTarget;
    }

    /// run the path RRTplanner
    /// this will always populate path to be the path we need to travel
    std::unique_ptr<Path> run(
        MotionInstant start, const MotionCommand* cmd,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles,
        std::unique_ptr<Path> prevPath = nullptr) override;

protected:
    /// maximum number of rrt iterations to run
    /// this does not include connect attempts
    unsigned int _maxIterations;

    /// Check to see if the previous path (if any) should be discarded and
    /// replaced with a newly-planned one
    bool shouldReplan(MotionInstant start, MotionInstant goal,
                      const MotionConstraints& motionConstraints,
                      const Geometry2d::ShapeSet* obstacles,
                      const Path* prevPath) const;

    /// Runs a bi-directional RRT to attempt to join the start and end states.
    Planning::InterpolatedPath* runRRT(
        MotionInstant start, MotionInstant goal,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles);
    
};

}  // namespace Planning
