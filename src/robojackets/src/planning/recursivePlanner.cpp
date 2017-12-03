#include "robojackets/planning/recursivePlanner.hpp"

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

RecursivePlanner::RecursivePlanner(int maxItr): maxItr_(maxItr) {}


vector<Point> RecursivePlanner::recursivePlanning(const ShapeSet &obstacles, Segment seg, int depth) {
  std::set<std::shared_ptr<Shape>> hitSet = obstacles.hitSet<Segment>(seg);
  // doesn't intersect, or depth = maxItr, just return current path
  if (hitSet.size() == 0 || depth >= maxItr_) {
    if (depth == maxItr_) {
      printf("reached max depht :!!! wtf\n");
    }
    return vector<Point>{seg.pt[0], seg.pt[1]};
  }
  // find a point in seg that actually intersects with an obstacle!
  // just take the first obstalce in hitset and let its centre point be that point.

  // HACK: converting shape to circle just to get its centre.
  // Point goal = static_cast<Circle*>((*hitSet.begin()).get())->center;
  // goal = findNonBlockedGoalRRT(goal, obstacles, 10);
  // using tangent function to find goal
  Point goal = findNonBlockedGoalTangent(obstacles, seg, *(static_cast<Circle*>((*hitSet.begin()).get())));
  Segment seg1(seg.pt[0], goal), seg2(goal, seg.pt[1]);
  vector<Point> tr1 = recursivePlanning(obstacles, seg1, depth+1);
  vector<Point> tr2 = recursivePlanning(obstacles, seg2, depth+1);
  tr1.insert(tr1.end(), tr2.begin(), tr2.end());
  return tr1;
}

Point RecursivePlanner::findNonBlockedGoalTangent(const ShapeSet& obstacles, Segment seg, const Circle& circle) {
  Point p1, p2;
  Circle largerCircle(circle.center, circle.radius()+Robot_Radius*1.5);
  if (largerCircle.tangentPoints(seg.pt[0], &p1, &p2)) {
    // HACK: should ideally make sure that these tangent points do not in fact lie in any obstacles.
    // HACK: also, should ideally draw tangents from both start and end points, and consider their intersection points.
    // find out closer point
    if (seg.distTo(p1) > seg.distTo(p2))
      swap(p1, p2);
    if (obstacles.hit(p1)) {
      swap(p1, p2);
    }
    if (obstacles.hit(p1)) {
      printf("for some reason it still hits the obstacle. calling escape obstacles planner.\n");
      return findNonBlockedGoalRRT(p1, obstacles, 10);
    }
    return p1;
  } 
  // soemthign went wrong, use escape obstacles planner
  printf("soemthing went worng. calling escape obstcles planner\n");
  return findNonBlockedGoalRRT(circle.center, obstacles, 10);
}

Point RecursivePlanner::findNonBlockedGoalPerpendicular(const ShapeSet &obstacles, Segment seg, const Circle& circle) {
  Segment l2(circle.center, seg.nearestPoint(circle.center));
  return Point();
}



Point RecursivePlanner::findNonBlockedGoalRRT(Point goal, const ShapeSet& obstacles, int maxItr) {
  // NOTE: maybe should not do this check, since we already know that it does hit the obstacles?
  if (obstacles.hit(goal)) {
    FixedStepTree goalTree;
    goalTree.init(goal, &obstacles);
    // HACK: fixing stepSize to .1, robojackets uses config file for htis too
    goalTree.step = 0.1;

    // The starting point is in an obstacle, extend the tree until we find
    // an unobstructed point
    Point newGoal;
    for (int i = 0; i < maxItr; ++i) {
      // extend towards a random point
      Tree::Point* newPoint = goalTree.extend(RandomFieldLocation());

      // if the new point is not blocked, it becomes the new goal
      if (newPoint && newPoint->hit.empty()) {
        newGoal = newPoint->pos;
        break;
      }
    }

    return newGoal;
  }

  return goal;
}



Planning::InterpolatedPath* RecursivePlanner::runRecursivePlanner(
        MotionInstant start, MotionInstant goal,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles) {
  InterpolatedPath *path = new InterpolatedPath();
  path->setStartTime(RJ::timestamp());
  // clean all data structures... probably should not have done this way
  
  vector<Point> pth = recursivePlanning(*obstacles, Segment(start.pos, goal.pos), 0);
  // convert to robojeckets path object;
  // add usign this weird meethod, copied from addPath in Tree.cpp
  // print the path
  // cout << "Path: ";
  // for (int i= 0; i < pth.size(); i++) {
  //   cout << pth[i] << " " ;
  // }
  // cout << endl;
  {
    path->waypoints.reserve(pth.size());
    for (auto v : pth) {
      path->waypoints.emplace_back(MotionInstant(v, Geometry2d::Point()),
                                    0);
    }    
  }
  ROS_INFO_NAMED("path", "non optimized path size = %lu", path->size());
  path = optimize(*path, obstacles, motionConstraints, start.vel, goal.vel);
  cout << "Done optimization." << endl;
  return path;
}

std::unique_ptr<Path> RecursivePlanner::run(
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
            runRecursivePlanner(start, goal, motionConstraints, obstacles);
        long long timetaken = RJ::timestamp() - tstart;
        float pathlength = path->length();
        float pathtime = path->getDuration();
        // just apend to a file...
        FILE *f = fopen("/tmp/log-rec.txt", "a");
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