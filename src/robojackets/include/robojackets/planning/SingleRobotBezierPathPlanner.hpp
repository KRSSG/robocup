#pragma once
#include <robojackets/planning/SingleRobotPathPlanner.hpp>
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

// Defines a base class for path planners that use the same
// bezier curve and velocity profiling functions, just to reduce
// duplication.
// Right now it just contains the path planning functions, rest of the functionality is 
// in ther respective classes.
// Should think of having a seperate class for velocity planning (as done by robojackets now)

namespace Planning {


class SingleRobotBezierPathPlanner : public SingleRobotPathPlanner {
public:


    // These planners will always run on path target
    MotionCommand::CommandType commandType() const override {
        return MotionCommand::PathTarget;
    }

    
protected:
    /** optimize the path
     *  Calls the cubicBezier optimization function.
     */
    Planning::InterpolatedPath* optimize(
        Planning::InterpolatedPath& path, const Geometry2d::ShapeSet* obstacles,
        const MotionConstraints& motionConstraints, Geometry2d::Point vi,
        Geometry2d::Point vf);

    /**
     * Uses a cubicBezier to interpolate between the points on the path and add
     * velocity planning
     */
    Planning::InterpolatedPath* cubicBezier(
        Planning::InterpolatedPath& path, const Geometry2d::ShapeSet* obstacles,
        const MotionConstraints& motionConstraints, Geometry2d::Point vi,
        Geometry2d::Point vf);

    /**
     * Helper function for cubicBezier() which uses Eigen matrices to solve for
     * the
     * cubic bezier equations.
     */
    Eigen::VectorXd cubicBezierCalc(double vi, double vf,
                                    std::vector<double>& points,
                                    std::vector<double>& ks,
                                    std::vector<double>& ks2);
};
  
}