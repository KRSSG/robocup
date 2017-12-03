#include <cstdlib> 
#include <cstdio>
#include <cmath>
#include "ssl_common/geometry.hpp"
#include "planners/naivepp.h"

namespace Navigation
{
    NaivePathPlanner::NaivePathPlanner() : 
      threshold(0),
      startx(0),
      starty(0),
      total_node(0),
      length(0)
    { }

    NaivePathPlanner::~NaivePathPlanner()
    { }
    
    int NaivePathPlanner::Connect(Vector2D<float> Start,
                                  Vector2D<float> End,
                                  obstacle*       obs,
                                  int             nObs,
                                  int             BotID,
                                  int*            nearest,
                                  bool            TeamBlue)
    {
      float A,B,C,D,dot,len_sq,param,xx,yy,minParam=999999;
      int minIndex = 0, i,count=0;
      for(i = 0; i < nObs;++i)
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
          xx = Start.x + param*C;
          yy = Start.y + param*D;
        }

        if((obs[i].x - xx)*(obs[i].x - xx) + (obs[i].y - yy)*(obs[i].y - yy) < obs[i].radius*obs[i].radius)
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
            *nearest = minIndex;
        return count;
    }
 /*   int NaivePathPlanner::returnSide(Vector2D<float> initial,)
    {
        
    }*/
    bool NaivePathPlanner::plan(Vector2D<float> initial,Vector2D<float> final,Vector2D<float> *pt1,Vector2D<float> *pt2,obstacle *obstacles,int nObs,int current_id,bool teamBlue)
     {
        int nearest,i;
        float maxdist = 0,dist;
        const float XCONSTANT = 3,YCONSTANT = 10;
        float vx,vy,obsDist,destDist,destSlope;
        if(Connect(initial,final,obstacles,nObs,current_id,&nearest,teamBlue)!=0)
        {
            for(i = 0;i < nObs;++i)
            {
                if(i == current_id)
                    continue;
                if((dist = 
                    (obstacles[i].x - obstacles[nearest].x) *
                    (obstacles[i].x - obstacles[nearest].x) +
                    (obstacles[i].y - obstacles[nearest].y) *
                    (obstacles[i].y - obstacles[nearest].y) )
                    < 4.5*obstacles[i].radius*obstacles[i].radius)
                    if(dist > maxdist)
                        maxdist = dist;
            }
            maxdist = sqrt(maxdist);
            maxdist += obstacles[nearest].radius;
            obsDist = Vector2D<float>::dist(initial,Vector2D<float>(obstacles[nearest].x,obstacles[nearest].y)) - maxdist;

            destDist = Vector2D<float>::dist(initial,final);
            destSlope = Vector2D<float>::angle(final,initial);
            vx = XCONSTANT * obsDist/destDist;
            vy = YCONSTANT / obsDist;
            if(vy < 0)
                vy = -vy;
//            Logger::toStdOut("%f  %f %f\n",vx,vy,maxdist);
            pt1->x = initial.x + vx *cos(destSlope) + vy*sin(destSlope);
            pt1->y = initial.y + vx *sin(destSlope) + vy*cos(destSlope);
            return false;
        }
        else
        {
            pt1 = &final;
            return true;
        }
       
     }
}


