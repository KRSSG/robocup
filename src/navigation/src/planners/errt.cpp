#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <cassert>
#include "planners/errt.h"
#include "planners/kdtree.h"
#include <ssl_common/config.h>

using namespace std;

namespace Navigation
{
  ERRT::ERRT(float GoalProb, float WaypointCacheProb) :
    StepLength(200.0f),
    GoalProb(GoalProb),
    WaypointCacheProb(WaypointCacheProb),
    waypointCacheEntries(0)
  { }

  ERRT::~ERRT()
  { }

  Point2D<int> ERRT::randomState(void) const
  {
    int randX = (rand() % HALF_FIELD_MAXX*2) - HALF_FIELD_MAXX ;
    int randY = (rand() % HALF_FIELD_MAXY*2) - HALF_FIELD_MAXY ;
    return Point2D<int>(randX, randY);
  }

  Point2D<int> ERRT::chooseTarget(const Point2D<int>& goal)
  {
    float prob = (float)rand() / RAND_MAX;

    if (prob < GoalProb)
      return goal;
    else if (GoalProb <= prob && prob < GoalProb + WaypointCacheProb && waypointCacheEntries == WAYPOINT_CACHE_SIZE)
    {
      return waypointCache[rand() % WAYPOINT_CACHE_SIZE];
    }
    else
      return randomState();
  }

  Point2D<int> ERRT::getNearest(KDTree<int>& tree, const Point2D<int>& target)
  {
    int t[2] = {target.x, target.y};
    KDNode<int>* nearestNode = tree.find_nearest(t);
    assert(nearestNode != NULL);
    return nearestNode->toVector2D();
  }

  Point2D<int> ERRT::extend(const vector<obstacle>& obs,
                            const Point2D<int>&     nearest,
                            const Point2D<int>&     target,
                            const Point2D<int>&     initial)
  {
    Vector2D<int> dirVect = target - nearest;
    float         dirLen  = dirVect.abs();

    Point2D<int> ext = nearest;  // ext is the nearest point by default
    if (dirLen > StepLength)
    {
      /* If the distance to target is more than one step length,
       * ext is the extension of nearest towards target by one step length
       */
      ext += dirVect * (StepLength / dirLen);
    }
    else if (dirLen < StepLength)
    {
      // If the distance to target is less than one step length, ext is the target
      ext = target;
    }
    
    if (nearest != initial)
    { 
      if (collides(nearest, ext, obs) == false)
        return ext;
      else
        return Point2D<int>::InvalidVector();
    }
    else 
    {
      // Allowing the bot to plan a path successfully when it accidentally is inside an obstacle
      if (collides(ext, obs) == false) // nearest = initial but ext does not collide with any obstacle
        return ext;
      else
        return Point2D<int>::InvalidVector();
    }
  }

  bool ERRT::plan(const Point2D<int>&     initial,
                  const Point2D<int>&     goal,
                  const vector<obstacle>& obs,
                  const int               goalThreshold,
                  list<Point2D<int> >&    waypoints)
  {
    waypoints.clear();

    // Modification on original ERRT algorithm: Checking and returning if a trivial path exists
    if (collides(initial, goal, obs) == false)
    {
      waypoints.push_front(goal);
      return true;
    }

    Vector2D<int> nearest, extended, target;
    KDTree<int> tree;

    nearest = initial;

    if (tree.add(initial) == false)
      return false;

    // Main RRT algorithm loop
    bool pathNotFound = false;
    while ((pathNotFound = Point2D<int>::distSq(nearest, goal) > goalThreshold * goalThreshold))
    {
      target = chooseTarget(goal);
      nearest = getNearest(tree, target);
      extended = extend(obs, nearest, target, initial);
      if (extended.valid())
      {
        if (tree.add(extended, nearest) == false)
          break; // Stop when no more nodes can be added to the tree
      }
    }

    int point[2] = {nearest.x, nearest.y};
    KDNode<int>* node = tree.find_nearest(point);
    while (node != NULL)
    {
      Point2D<int> wp = node->toVector2D();
      
      // Adding the waypoints to the cache only when a path was found
      if (pathNotFound == false)
      {
        // Adding the waypoint to the waypoint cache with random replacement
        if (waypointCacheEntries == WAYPOINT_CACHE_SIZE)
          waypointCache[rand() % WAYPOINT_CACHE_SIZE] = wp; // When the cache is filled, replace randomly
        else
          waypointCache[waypointCacheEntries++] = wp; // When the cache is not filled, replace sequencially
      }
      
      // Adding the waypoint to the path
      waypoints.push_front(wp);
      node = node->errtParent;
    }
    
    // Modification on original ERRT algorithm: Finding the waypoint farthest from initial point such that the straight line joining them is obstacle-free
    smoothPath(waypoints, obs);
    return !pathNotFound;
  }

  void ERRT::smoothPath(list<Point2D<int> >& waypoints, const vector<obstacle>& obs)
  {
    int nodesToDelete = waypoints.size();
    list<Point2D<int> >::reverse_iterator rit;
    for (rit = waypoints.rbegin(); rit != waypoints.rend(); ++rit, --nodesToDelete)
    {
      if (collides(waypoints.front(), *rit, obs) == false)
      {
        --nodesToDelete;
        break;
      }
    }
    for (int i = 0; i < nodesToDelete; ++i)
      waypoints.pop_front();
  }
} // namespace Strategy
