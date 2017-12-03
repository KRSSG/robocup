#ifndef ERRT_H
#define ERRT_H
#include <vector>
#include <list>
#include "ssl_common/geometry.hpp"
#include "common.h"

namespace Navigation {
  // forward declaration
  template <class T> class KDTree;

  class ERRT
  {
  public:
    typedef struct
    {
      Point2D<int> center;
      int radius;
    } obstacle;

  public:
    static const int WAYPOINT_CACHE_SIZE = 50;
    const float StepLength;

    const float GoalProb;
    const float WaypointCacheProb;

    int          waypointCacheEntries;
    Point2D<int> waypointCache[WAYPOINT_CACHE_SIZE];

    Point2D<int> chooseTarget(const Point2D<int>& goal);

    Point2D<int> getNearest(KDTree<int>& tree, const Point2D<int>& target);

    Point2D<int> extend(const std::vector<obstacle>& obs,
                        const Point2D<int>&          nearest,
                        const Point2D<int>&          target,
                        const Point2D<int>&          initial);

    // Returns a random point anywhere on the playing field
    Point2D<int> randomState(void) const;

    // Detects if a point collides with any of the obstacles
    inline bool collides(const Point2D<int>& point, const std::vector<obstacle>& obs)
    {
      int size = obs.size();
      for (int i = 0; i < size; ++i)
      {
        if (intersects(point, obs[i].center, obs[i].radius))
          return true;
      }
      return false;
    }

    // Detects if a line segment whose ends are point1 and point2 collides with any of the obstacles
    inline bool collides(const Point2D<int>&          point1,
                         const Point2D<int>&          point2,
                         const std::vector<obstacle>& obs)
    {
      int size = obs.size();
      for (int i = 0; i < size; ++i)
      {
        if (intersects(point1, point2, obs[i].center, obs[i].radius))
          return true;
      }
      return false;
    }

  public:
    ERRT(float GoalProb, float WaypointCacheProb);

    ~ERRT();

    bool plan(const Point2D<int>&          start,
              const Point2D<int>&          goal,
              const std::vector<obstacle>& obs,
              const int                    goalThreshold,
              std::list<Point2D<int> >&    waypoints);
              
    void smoothPath(std::list<Point2D<int> >& waypoints, const std::vector<obstacle>& obs);
  }; // class ERRT
}
#endif