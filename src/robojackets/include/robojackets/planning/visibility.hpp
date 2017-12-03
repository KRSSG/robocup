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
// create a visibility graph of all the vertices (robot and walls).
// run shortest path query on that graph
// should not take too much time to build!
namespace Planning {

class VisibilityGraph : public SingleRobotBezierPathPlanner { 
protected:
  vector<Segment> segments; // all the line segments
  vector<Point> vertices; // all the vertices in the graph
  vector<vector<int> > graph;
  bool LineSegmentIntersection(Segment s1, Segment s2);
  void addVertex(Point p, int numOldVertices);

  Planning::InterpolatedPath* runVG(
        MotionInstant start, MotionInstant goal,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles);
public:
  MotionCommand::CommandType commandType() const override {
        return MotionCommand::PathTarget;
    }
  VisibilityGraph();
  // all things in metres
  void addBotObstacle(Point bot, float radius);
  void addBallObstacle(Point ball, float radius);
  void addWallObstacle(Segment seg);
  std::list<vertex_t> getPathUnoptimized(Point p1, Point p2);

  // inherited
  std::unique_ptr<Path> run(
        MotionInstant start, const MotionCommand* cmd,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles,
        std::unique_ptr<Path> prevPath = nullptr) override;



};
}