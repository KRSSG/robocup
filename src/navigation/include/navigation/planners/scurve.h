#ifndef S_CURVE_H
#define S_CURVE_H =
#include <vector>
#include <list>
#include "ssl_common/geometry.hpp"
#include "common.h"

namespace Navigation{
  
class SCurve
{
private:
  static const float x_min;
  static const float x_max;
  static const float y_min;
  static const float y_max;

  float              threshold;
  float              startx, starty;
  int                total_node;
  float              length;

public:
  SCurve();

  ~SCurve();

  bool plan(Vector2D<float> initial, Vector2D<float> final, Vector2D<float> *pt1, Vector2D<float> *pt2, obstacle *obstacles, int obstacle_count, int current_id, bool teamBlue);

private:
  int ifInObstacle(Vector2D<float> point, obstacle *obs, int nObs, int closest);
  int Connect(Vector2D<float> Start, Vector2D<float> End, obstacle* obs, int nObs, int BotID, int *nearest, bool TeamBlue);
  Vector2D<float> getNextPt(Vector2D<float> initial, int nClosest, Vector2D<float> final, obstacle *obstacles, int nObs, int BotID, bool teamBlue);
  Vector2D<float> getNextPtRec(Vector2D<float> initial, int lastClosest, int closest, obstacle *obstacles, int nObs);
}; // class SCurve
}
#endif