#ifndef NAIVE_PATH_PLANNER_H
#define NAIVE_PATH_PLANNER_H

#include <vector>
#include <list>
#include "ssl_common/geometry.hpp"
#include "common.h"
namespace Navigation{
class NaivePathPlanner
{
private:
  static const float x_min;
  static const float x_max;
  static const float y_min;
  static const float y_max;

  float            threshold;
  float            startx, starty;
  int              total_node;
  float length;


public:
  NaivePathPlanner();
  ~NaivePathPlanner();

  bool plan(Vector2D<float> initial, Vector2D<float> final, Vector2D<float> *pt1, Vector2D<float> *pt2, obstacle *obstacles, int nObs, int current_id, bool teamBlue);


private:
  int Connect(Vector2D<float> Start, Vector2D<float> End, obstacle* obs, int nObs, int BotID, int *nearest, bool TeamBlue);

}; // class NaivePathPlanner
}
#endif