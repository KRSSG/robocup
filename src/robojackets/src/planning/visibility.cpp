#include "robojackets/planning/visibility.hpp"  
#include "robojackets/planning/dijkstra.hpp"

#include <ros/console.h>
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

using namespace Geometry2d;
namespace Planning {

VisibilityGraph::VisibilityGraph() {

}
void VisibilityGraph::addBotObstacle(Point bot, float radius) {
  // add the bounding box of the bot as an obstacle.
  // form the set of line segments that are the bot.
  vector<Segment> botSegments;
  vector<Point> botPoints;
  int xArr[] = {-1,+1,+1,-1}, yArr[] = {+1, +1, -1, -1};
  Point prev = bot+Point(xArr[3], yArr[3])*radius;
  for (int i= 0; i < 4; i++) {
    Point p = bot+Point(xArr[i], yArr[i])*radius;    
    botSegments.push_back(Segment(p, prev));
    prev = p;
    botPoints.push_back(p);
  }
  // add all the bot's segments
  segments.insert(segments.end(), botSegments.begin(), botSegments.end());
  // add each vertex to the graph.
  int numOldVertices = vertices.size();
  for (int i = 0; i < botPoints.size(); i++) {
    addVertex(botPoints[i], numOldVertices);
  }
}

void VisibilityGraph::addVertex(Point p, int numOldVertices) {
  int me = vertices.size();
  graph.push_back(vector<int>());
  vertices.push_back(p);
  for (int j = 0; j < numOldVertices; j++) {
    // try edge from me to j
    Segment curSeg(vertices[me], vertices[j]);
    // make sure this edge intersects NONE of the segments, including bot's own segments.
    bool intersects = false;
    for (int k = 0; k < segments.size(); k++) {
      if (LineSegmentIntersection(curSeg, segments[k])) {
        intersects = true;
        break;
      }
    }
    if (!intersects) {
      graph[j].push_back(me);
      graph[me].push_back(j);
    }
  }
}

void VisibilityGraph::addBallObstacle(Point ball, float radius) {

}
void VisibilityGraph::addWallObstacle(Segment seg) {

}
std::list<vertex_t> VisibilityGraph::getPathUnoptimized(Point p1, Point p2) {
  // need to add p1 and p2 to the graph!
  // right now, NOT correcting the graph aftwerwards!
  // i.e. this function can be called only once, then VisibilityGraph object should not be used again.

 
  // add the 2 vertices;
  int idx1 = vertices.size(), idx2 = idx1+1;
  addVertex(p1, vertices.size());
  addVertex(p2, vertices.size());
  // construct dijkstra object.
  using namespace Dijkstra;
  assert(graph.size() == vertices.size());
  adjacency_list_t list(vertices.size());
  for (int i = 0; i < graph.size(); i++) {
    for (int j = 0; j < graph[i].size(); j++) {
      list[i].push_back(neighbor(graph[i][j], (vertices[i]-vertices[graph[i][j]]).mag()));
    }
  }
  // print the graph
  // for (int i = 0 ; i < graph.size(); i++) {
  //   printf("%d: ", i);
  //   for (int j = 0; j < graph[i].size(); j++) {
  //     printf("%d (%lf) ", graph[i][j], (vertices[i]-vertices[graph[i][j]]).mag());
  //   }
  //   printf("\n");
  // }
  std::vector<weight_t> min_distance;
  std::vector<vertex_t> previous;
  DijkstraComputePaths(idx1, list, min_distance, previous);
  // std::cout << "Distance from 0 to 4: " << min_distance[4] << std::endl;
  std::list<vertex_t> pth = DijkstraGetShortestPathTo(idx2, previous);
  // cout << "total number of vertices: " << vertices.size();
  // std::cout << "Path : ";
  // std::copy(pth.begin(), pth.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
  

  return pth;
}

bool VisibilityGraph::LineSegmentIntersection(Segment s1, Segment s2) {
  // check if they share an end point; if they do, then no intersect4ion!
  if (s1.pt[0] == s2.pt[0] || s1.pt[0] == s2.pt[1] ||
      s1.pt[1] == s2.pt[0] || s1.pt[1] == s2.pt[1])
    return false;
  return s1.intersects(s2);
}

Planning::InterpolatedPath* VisibilityGraph::runVG(
        MotionInstant start, MotionInstant goal,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles) {
  // printf("run vg called!! <>< >< >< > <> < >< >< ><\n");
  InterpolatedPath *path = new InterpolatedPath();
  path->setStartTime(RJ::timestamp());
  // clean all data structures... probably should not have done this way
  segments.clear();
  graph.clear();
  vertices.clear();

  // HACK: assume each obstacle is a circle, and is a bot.
  for (const auto& shape : obstacles->shapes()) {
    Circle *bot=static_cast<Circle*>(shape.get());
    // increase radius a lil bit here so that replanning is not done that often.
    addBotObstacle(bot->center, bot->radius()*2);
  }
  // convert to robojeckets path object;
  // add usign this weird meethod, copied from addPath in Tree.cpp
  std::list<vertex_t> pth = getPathUnoptimized(start.pos, goal.pos);
  {
    path->waypoints.reserve(pth.size());
    for (auto v : pth) {
      path->waypoints.emplace_back(MotionInstant(vertices[v], Geometry2d::Point()),
                                    0);
    }    
  }
  ROS_INFO_NAMED("path", "non optimized path size = %lu", path->size());
  path = optimize(*path, obstacles, motionConstraints, start.vel, goal.vel);
  return path;
}

std::unique_ptr<Path> VisibilityGraph::run(
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
    if (SingleRobotPathPlanner::shouldReplan(start, motionConstraints,
                                             obstacles, prevPath.get())) {        
        // Run bi-directional RRT to generate a path.
        long long tstart = RJ::timestamp();
        InterpolatedPath* path =
            runVG(start, goal, motionConstraints, obstacles);
        // debugging and logging stuf : make logs of timetaken, path lenght, path time.
        long long timetaken = RJ::timestamp() - tstart;
        float pathlength = path->length();
        float pathtime = path->getDuration();
        // just apend to a file...
        FILE *f = fopen("/tmp/log-vis.txt", "a");
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


using namespace Eigen;










// already defined in RRTPlanner, we just declare here...
float getTime(InterpolatedPath& path, int index,
              const MotionConstraints& motionConstraints, float startSpeed,
              float endSpeed);/* {
    return Trapezoidal::getTime(
        path.length(0, index), path.length(), motionConstraints.maxSpeed,
        motionConstraints.maxAcceleration, startSpeed, endSpeed);
}
*/

}
