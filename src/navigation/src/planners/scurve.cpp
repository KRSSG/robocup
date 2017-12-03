#include "planners/scurve.h"

#include <cstdlib>
#include <cstdio>
#include <cmath>


namespace Navigation
{
  SCurve::SCurve() :
    threshold(0),
    startx(0),
    starty(0),
    total_node(0),
    length(0)
  { }

  SCurve::~SCurve()
  { }

  int SCurve::Connect(Vector2D<float> Start, Vector2D<float> End, obstacle* obs, int nObs, int BotID, int *nearest, bool TeamBlue)
  {
    float A, B, C, D, dot, len_sq, param, xx, yy, minParam = 999999;
    int minIndex = 0, i, count = 0;
    for(i = 0; i < nObs; ++i)
    {
      if((TeamBlue && i == BotID) || (!TeamBlue && i == BotID + 5))
        continue;
      A = obs[i].x - Start.x; // vector from one point to test point
      B = obs[i].y - Start.y;
      C = End.x - Start.x; //vector from one endpoint to the other end point
      D = End.y - Start.y;

      dot = A * C + B * D;
      len_sq = C * C + D * D;
      param = dot / len_sq; // length range reduced to (0,1)

      if(param < 0)
      {
        xx = Start.x;
        yy = Start.y;
      }
      else if(param > 1)
      {
        xx = End.x;
        yy = End.y;
      }
      else
      {
        xx = Start.x + param * C;
        yy = Start.y + param * D;
      }

      if((obs[i].x - xx) * (obs[i].x - xx) + (obs[i].y - yy) * (obs[i].y - yy) < obs[i].radius * obs[i].radius)
      {
        count++;
        if(param < minParam)
        {
          minIndex = i;
          minParam = param;
        }
      }
    }
    if(nearest)
    {
      *nearest = minIndex;
      if(count != 0)
      {
        for(i = 0; i < nObs; ++i)
        {
          len_sq = (obs[*nearest].x - obs[i].x) * (obs[*nearest].x - obs[i].x) + (obs[*nearest].y - obs[i].y) * (obs[*nearest].y - obs[i].y);
          if(len_sq < (obs[i].radius * obs[i].radius))
            count++;
        }
      }
    }
    return count;
  }

  int SCurve::ifInObstacle(Vector2D<float> point, obstacle *obs, int nObs, int closest)
  {
    float dist_sq;
    for(int i = 0; i < nObs; ++i)
    {
      if(i != closest)
      {
        dist_sq = (point.x - obs[i].x) * (point.x - obs[i].x) + (point.y - obs[i].y) * (point.y - obs[i].y);
        if(dist_sq < (obs[i].radius * obs[i].radius))
          return i;
      }
    }
    return -1;
  }

  Vector2D<float> SCurve::getNextPt(Vector2D<float> initial, int closest, Vector2D<float> final, obstacle *obstacles, int nObs, int BotID, bool teamBlue)
  {
    float dist_sq = ((initial.x - obstacles[closest].x) * (initial.x - obstacles[closest].x)) + ((initial.y - obstacles[closest].y) * (initial.y - obstacles[closest].y));
    float rad_sq = obstacles[closest].radius * obstacles[closest].radius;
    float x = dist_sq * rad_sq / (dist_sq - rad_sq);
    float xx = (obstacles[closest].x - initial.x) / sqrt(dist_sq);
    float yy = (obstacles[closest].y - initial.y) / sqrt(dist_sq);
    Vector2D<float> V1;
    Vector2D<float> V2;
    /*V1.x = obstacles[closest].x + yy*sqrt(x);
    V1.y = obstacles[closest].y - xx*sqrt(x);
    V2.x = obstacles[closest].x - yy*sqrt(x);
    V2.y = obstacles[closest].y + xx*sqrt(x); */
    V1.x = obstacles[closest].x + yy * 1.2f * sqrt(x);
    V1.y = obstacles[closest].y - xx * 1.2f * sqrt(x);
    V2.x = obstacles[closest].x - yy * 1.2f * sqrt(x);
    V2.y = obstacles[closest].y + xx * 1.2f * sqrt(x);
    Vector2D<float> clearerPoint;

    if((SCurve::Connect(V1, final, obstacles, nObs, BotID, NULL, teamBlue) > (SCurve::Connect(V2, final, obstacles, nObs, BotID, NULL, teamBlue))))
      clearerPoint = V2;
    else
      clearerPoint = V1;
    int nearestObstacle = ifInObstacle(clearerPoint, obstacles, nObs, closest);
    if(nearestObstacle != -1)
      return getNextPtRec(initial, closest, nearestObstacle, obstacles, nObs);
    else
      return clearerPoint;
  }
  Vector2D<float> SCurve::getNextPtRec(Vector2D<float> initial, int lastClosest, int closest, obstacle *obstacles, int nObs)
  {
    // Logger::toStdOut("Entering recursion\n");
    float dist_sq = ((initial.x - obstacles[closest].x) * (initial.x - obstacles[closest].x)) + ((initial.y - obstacles[closest].y) * (initial.y - obstacles[closest].y));
    float rad_sq = obstacles[closest].radius * obstacles[closest].radius;
    float x = dist_sq * rad_sq / (dist_sq - rad_sq);
    float xx = (obstacles[closest].x - initial.x) / sqrt(dist_sq);
    float yy = (obstacles[closest].y - initial.y) / sqrt(dist_sq);
    Vector2D<float> V1;
    Vector2D<float> V2;
    V1.x = obstacles[closest].x + yy * sqrt(x);
    V1.y = obstacles[closest].y - xx * sqrt(x);
    V2.x = obstacles[closest].x - yy * sqrt(x);
    V2.y = obstacles[closest].y + xx * sqrt(x);
    Vector2D<float> clearerPoint;
    float distSq1 = Vector2D<float>::distSq(V1, Vector2D<float>(obstacles[lastClosest].x, obstacles[lastClosest].y));
    float distSq2 = Vector2D<float>::distSq(V2, Vector2D<float>(obstacles[lastClosest].x, obstacles[lastClosest].y));
    if(distSq1 < distSq2)
      clearerPoint = V2;
    else
      clearerPoint = V1;

    int nearestObstacle = ifInObstacle(clearerPoint, obstacles, nObs, closest);

    if(nearestObstacle != -1)
      return getNextPtRec(initial, closest, nearestObstacle, obstacles, nObs);
    else
      return clearerPoint;
  }

  bool SCurve::plan(Vector2D<float> initial, Vector2D<float> final, Vector2D<float> *pt1, Vector2D<float> *pt2, obstacle *obstacles, int obstacle_count, int current_id, bool teamBlue)
  {
    int closest;
    Vector2D<float> TangentPt;
    if(!SCurve::Connect(initial, final, obstacles, obstacle_count, current_id, &closest, teamBlue))
    {
      *pt1 = final;
      return true;
    }

    *pt1 = SCurve::getNextPt(initial, closest, final, obstacles, obstacle_count, current_id, teamBlue);

    if(!SCurve::Connect(*pt1, final, obstacles, obstacle_count, current_id, &closest, teamBlue))
    {
      *pt2 = final;
    }
    else
    {
      *pt2 = SCurve::getNextPt(*pt1, closest, final, obstacles, obstacle_count, current_id, teamBlue);
    }

    return false;
  }
}
