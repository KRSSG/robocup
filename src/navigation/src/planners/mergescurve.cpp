#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
// #include "comdef.h"
#include "planners/mergescurve.h"
using namespace std;
namespace Navigation
{
  MergeSCurve::MergeSCurve() :
    Obstacles(NULL),
    nObstacles(0),
    nBotID(-1)
  { }

  MergeSCurve::~MergeSCurve()
  { }

  int MergeSCurve::Connect(Vector2D<int> Start,
                           Vector2D<int> End,
                           std::vector<obstacle>       obs,
                           int             nObs,
                           int             BotID,
                           int*            nearest,
                           bool            TeamBlue)
  {
    float A, B, C, D, dot, len_sq, param, xx, yy, minParam = 999999;
    int minIndex = 0, i, count = 0;

    // std::cout<<"nObstacles ::::::::"<<nObs<<std::endl;

    nObs = obs.size();
    // cout<<" obs size ================ "<<obs.size()<<endl;
    for(i = 0; i < nObs; ++i)
    {
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
      /*      if(count !=0)
            {
                for(i = 0;i<nObs;++i)
                {
                    len_sq = (obs[*nearest].x-obs[i].x)*(obs[*nearest].x-obs[i].x) + (obs[*nearest].y-obs[i].y)*(obs[*nearest].y-obs[i].y);
                    if(len_sq < (obs[i].radius*obs[i].radius))
                        count++;
                }
            }*/
    }


    // std::cout<<"nobs : "<<nObs<<std::endl;
    // std::cout<<"count : "<<count<<std::endl;
    return count;
  }

  int MergeSCurve::ifInObstacle(Vector2D<int> point, std::vector<obstacle> obs, int nObs, int closest)
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

  Vector2D<int> MergeSCurve::getNextPt(Vector2D<int> initial, int closest, Vector2D<int> final, std::vector<obstacle> obstacles, int nObs, int BotID, bool teamBlue)
  {
    //Logger::toStdOut("Getting point\n");
    float dist_sq = ((initial.x - obstacles[closest].x) * (initial.x - obstacles[closest].x)) + ((initial.y - obstacles[closest].y) * (initial.y - obstacles[closest].y));
    float rad_sq = obstacles[closest].radius * obstacles[closest].radius;
    float  x, xx, yy, dx, dy;
    Vector2D<int> clearerPoint, V1, V2;
    int cV1, cV2;

    xx = (obstacles[closest].x - initial.x) / sqrt(dist_sq);
    yy = (obstacles[closest].y - initial.y) / sqrt(dist_sq);

    dx = (final.x - initial.x) / sqrt(dist_sq);
    dy = (final.y - initial.y) / sqrt(dist_sq);

    if(dist_sq - rad_sq < 50)
    {
      V1.x = initial.x + yy * 5;
      V1.y = initial.y - xx * 5;
      V2.x = initial.x - yy * 5;
      V2.y = initial.y + xx * 5;

      if(yy * dx - xx * dy > 0)
        return V1;
      else
        return V2;
    }
    else
    {
      x = dist_sq * rad_sq / (dist_sq - rad_sq);
      x = sqrt(x) * 1.2f;

      V1.x = obstacles[closest].x + yy * x;
      V1.y = obstacles[closest].y - xx * x;
      V2.x = obstacles[closest].x - yy * x;
      V2.y = obstacles[closest].y + xx * x;

      cV1 = Connect(V1, final, obstacles, nObs, BotID, NULL, teamBlue);
      cV2 = Connect(V2, final, obstacles, nObs, BotID, NULL, teamBlue);
//          if(cV1 == cV2)
//          {
      if(yy * dx - xx * dy > 0)
        return V1;
      else
        return V2;
//          }
//          else if(cV1 > cV2)
//              return V2;
//          else
//              return V1;
    }
  }
  Vector2D<int> MergeSCurve::getNextPtRec(Vector2D<int> initial, int lastClosest, int closest, std::vector<obstacle> obstacles, int nObs)
  {
    float dist_sq = ((initial.x - obstacles[closest].x) * (initial.x - obstacles[closest].x)) + ((initial.y - obstacles[closest].y) * (initial.y - obstacles[closest].y));
    float rad_sq = obstacles[closest].radius * obstacles[closest].radius;
    float x = dist_sq * rad_sq / (dist_sq - rad_sq);
    float xx = (obstacles[closest].x - initial.x) / sqrt(dist_sq);
    float yy = (obstacles[closest].y - initial.y) / sqrt(dist_sq);
    Vector2D<int> V1;
    Vector2D<int> V2;
    V1.x = obstacles[closest].x + yy * sqrt(x);
    V1.y = obstacles[closest].y - xx * sqrt(x);
    V2.x = obstacles[closest].x - yy * sqrt(x);
    V2.y = obstacles[closest].y + xx * sqrt(x);
    Vector2D<int> clearerPoint;
    float distSq1 = Vector2D<int>::distSq(V1, Vector2D<int>(obstacles[lastClosest].x, obstacles[lastClosest].y));
    float distSq2 = Vector2D<int>::distSq(V2, Vector2D<int>(obstacles[lastClosest].x, obstacles[lastClosest].y));
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

  void MergeSCurve::ConnectObstacles(bool *Connected, int nCurrentObstacle)
  {
    int nCounterI;
    for(nCounterI = 0; nCounterI < nObstacles; ++nCounterI)
    {
      if(nCounterI == nBotID || Connected[nCounterI])
        continue;
      if( (Obstacles[nCounterI].x - Obstacles[nCurrentObstacle].x) *
          (Obstacles[nCounterI].x - Obstacles[nCurrentObstacle].x) +
          (Obstacles[nCounterI].y - Obstacles[nCurrentObstacle].y) *
          (Obstacles[nCounterI].y - Obstacles[nCurrentObstacle].y) <
          4 * Obstacles[nCounterI].radius * Obstacles[nCounterI].radius)
      {
        Connected[nCounterI] = true;
        ConnectObstacles(Connected, nCounterI);
      }
    }
  }

  int MergeSCurve::MergeObstacles()
  {

    if(Obstacles.size()==0)
      return 0;

    bool bChecked[MAX_OBSTACLES], bConnected[MAX_OBSTACLES];
    obstacle Merged[MAX_OBSTACLES];

    int nMaxMerged = 0;
    float minx, miny, maxx, maxy, dRadius = Obstacles[0].radius;

    int nCounterI, nCounterJ, nMergedObstacles = nObstacles;

    for(nCounterI = 0; nCounterI < nObstacles; ++nCounterI)
    {
      bChecked[nCounterI] = false;
      bConnected[nCounterI] = false;
    }

    for(nCounterI = 0; nCounterI < nObstacles; ++nCounterI)
    {
      if(bChecked[nCounterI] || nBotID == nCounterI)
        continue;
      minx = Obstacles[nCounterI].x;
      miny = Obstacles[nCounterI].y;
      maxx = Obstacles[nCounterI].x;
      maxy = Obstacles[nCounterI].y;
      bConnected[nCounterI] = true;
      ConnectObstacles(bConnected, nCounterI);

      for(nCounterJ = 0; nCounterJ < nObstacles; ++nCounterJ)
      {
        if(bConnected[nCounterJ])
        {
          if(Obstacles[nCounterJ].x < minx)
            minx = Obstacles[nCounterJ].x ;
          if(Obstacles[nCounterJ].y < miny)
            miny = Obstacles[nCounterJ].y ;
          if(Obstacles[nCounterJ].x > maxx)
            maxx = Obstacles[nCounterJ].x ;
          if(Obstacles[nCounterJ].y > maxy)
            maxy = Obstacles[nCounterJ].y ;
          bConnected[nCounterJ] = false;
          bChecked[nCounterJ] = true;
        }
      }

      Merged[nMaxMerged].x = (maxx + minx) / 2;
      Merged[nMaxMerged].y = (maxy + miny) / 2;
      Merged[nMaxMerged].radius = sqrt((maxx - Merged[nMaxMerged].x) *
                                       (maxx - Merged[nMaxMerged].x) +
                                       (maxy - Merged[nMaxMerged].y) *
                                       (maxy - Merged[nMaxMerged].y))
                                  + dRadius;
      nMaxMerged++;
    }
    for(nCounterI = 0; nCounterI < nMaxMerged; ++nCounterI)
    {
      Obstacles[nCounterI] = Merged[nCounterI];
    }
    return nMaxMerged;
  }

  bool MergeSCurve::plan(const Vector2D<int>& initial,
                         const Vector2D<int>& final,
                         Vector2D<int> *pt1,
                         Vector2D<int> *pt2,
                         std::vector<obstacle> obstacles,
                         int obstacle_count,
                         int current_id,
                         bool teamBlue)
  {
    int closest;
    nObstacles = obstacle_count;
    Obstacles = vector<obstacle>(obstacles);
    nBotID = current_id;
    nObstacles = MergeObstacles();
    //     fstream file;
    // file.open("/home/kgpkubs/ssl/debuger.txt", fstream::app);
    Vector2D<float> TangentPt;
    *pt1 = final;


    if(!Connect(initial, final, obstacles, nObstacles, current_id, &closest, teamBlue))
    {
      
      // file<<"connect 1111111111111111111111"<<std::endl;
      // file.close();
      return true;
    }
    *pt1 = getNextPt(initial, closest, final, obstacles, nObstacles, current_id, teamBlue);
    if(!Connect(*pt1, final, obstacles, obstacle_count, current_id, &closest, teamBlue))
    {
      
      // file<<"connect 222222222222222222222"<<std::endl;      
      *pt2 = final;
    }
    else
    {
      
      // file<<"connect 333333333333333333"<<std::endl;


      *pt2 = getNextPt(*pt1, closest, final, obstacles, obstacle_count, current_id, teamBlue);
    }

    // file.close();
    return false;
  }
}
