#pragma once
#include <stdio.h>
#include <bits/stdc++.h>
#include <robojackets/Geometry2d/Polygon.hpp>
#include "robojackets/planning/InterpolatedPath.hpp"


#include "robojackets/planning/SingleRobotBezierPathPlanner.hpp"
#include "robojackets/planning/Tree.hpp"
#include <robojackets/Geometry2d/ShapeSet.hpp>
#include <robojackets/Geometry2d/Point.hpp>
#include <robojackets/planning/InterpolatedPath.hpp>
#include <robojackets/planning/MotionCommand.hpp>
#include <robojackets/planning/MotionConstraints.hpp>
#include <robojackets/planning/MotionInstant.hpp>
#include "robojackets/planning/dijkstra.hpp"

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <list>
using namespace std;

using namespace Geometry2d;


namespace Planning {


class RecursivePlanner : public SingleRobotBezierPathPlanner {
  int maxItr_;
public:
  MotionCommand::CommandType commandType() const override {
      return MotionCommand::PathTarget;
  }
  RecursivePlanner(int maxItr);


public:
  // the recursive search fucntion
  vector<Point> recursivePlanning(const ShapeSet &obstacles, Segment seg, int depth);

  // RRT based implementation of finding a non-blocked point.
  Point findNonBlockedGoalRRT(Point goal, const ShapeSet& obstacles, int maxItr);
  // find a non-blocked point by drawing tangent to the obstacle. assumes obstacle is a circle (ie robot)
  Point findNonBlockedGoalTangent(const ShapeSet& obstacles, Segment seg, const Circle& circle);
  // find a non-blcoked point by drawing a perpendicular line
  Point findNonBlockedGoalPerpendicular(const ShapeSet &obstacles, Segment seg, const Circle& circle);
  // use a vertex of the obstacle (assuming its a polyong! or circle maybe) to find a non-blocked point.
  Point findNonBlockedGoalVertex(Point goal, const ShapeSet &obstacles, std::shared_ptr<Shape> obs);
    // inherited
  std::unique_ptr<Path> run(
        MotionInstant start, const MotionCommand* cmd,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles,
        std::unique_ptr<Path> prevPath = nullptr) override;


  // helper functions
  /// Runs a bi-directional RRT to attempt to join the start and end states.
    Planning::InterpolatedPath* runRecursivePlanner(
      MotionInstant start, MotionInstant goal,
      const MotionConstraints& motionConstraints,
      const Geometry2d::ShapeSet* obstacles);

  };
}