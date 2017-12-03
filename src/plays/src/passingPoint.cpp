#include <iostream>
#include <utility>
#include "play.hpp"
#include "krssg_ssl_msgs/BeliefState.h"
#include "tactics/tactic.h"
#include <ssl_common/config.h>
#include <ssl_common/geometry.hpp>
#include "pPassTest.hpp"

using namespace Strategy;

 bool checkPointInField(Vector2D<int> point)
{
  if(fabs(point.x)<fabs(HALF_FIELD_MAXX) && fabs(point.y)<fabs(HALF_FIELD_MAXY)) return true;
  return false;
}

Vector2D<int> findPointForPassing(int passrer_id,int receiver_id,int marker_id,krssg_ssl_msgs::BeliefState state)
{
  Vector2D<int> passPoint,Point1,Point2;
    bool intersecting1=true,intersecting2=true;
    double x1,x2,y1,y2,x3,y3,x4,y4,temp;
    
   //  //#############this is for intersection of ball/receiver conic and receiver marker line
    double c1,a1=c1=(u*u-w*w);
    double b1=-2*(state.homePos[passrer_id].x*u*u-state.homePos[receiver_id].x*w*w);
    double d1=-2*(state.homePos[passrer_id].y*u*u-state.homePos[receiver_id].y*w*w);
    double e1=(state.homePos[passrer_id].x*state.homePos[passrer_id].x+state.homePos[passrer_id].y*state.homePos[passrer_id].y)*u*u-(state.homePos[receiver_id].x*state.homePos[receiver_id].x+state.homePos[receiver_id].y*state.homePos[receiver_id].y)*w*w;
   
    double a2=-2*(state.awayPos[marker_id].x*u*u-state.homePos[receiver_id].x*v*v);
    double b2=-2*(state.awayPos[marker_id].y*u*u-state.homePos[receiver_id].y*v*v);
    double c2=(state.awayPos[marker_id].x*state.awayPos[marker_id].x+state.awayPos[marker_id].y*state.awayPos[marker_id].y)*u*u-(state.homePos[receiver_id].x*state.homePos[receiver_id].x+state.homePos[receiver_id].y*state.homePos[receiver_id].y)*v*v;
    temp=pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2);
    if(((b1*pow(a2,2) + a1*pow(b2,2))!=0 || a2!=0)&&temp>=0)
    {
       x1=-(c2 - (b2*(pow(a2,2)*d1 + a2*pow(temp,(0.5)) + 2*a1*b2*c2 - a2*b2*c1))/(2*(b1*pow(a2,2) + a1*pow(b2,2))))/a2 ;
       y1=-(pow(a2,2)*d1 + a2*pow(temp,(0.5)) + 2*a1*b2*c2 - a2*b2*c1)/(2*(b1*pow(a2,2) + a1*pow(b2,2))) ;
      
       x2=-(c2 - (b2*(pow(a2,2)*d1 - a2*pow(temp,(0.5)) + 2*a1*b2*c2 - a2*b2*c1))/(2*(b1*pow(a2,2) + a1*pow(b2,2))))/a2 ;
       y2=-(pow(a2,2)*d1 - a2*pow(temp,(0.5)) + 2*a1*b2*c2 - a2*b2*c1)/(2*(b1*pow(a2,2) + a1*pow(b2,2))) ;
    }
    else 
    {
      intersecting1=false;
    }
    
     //#############this is for intersection of ball/marker conic and receiver marker line
      a1=c1=v*v-w*w ;
      b1=-2*(state.homePos[passrer_id].x*v*v-state.awayPos[marker_id].x*w*w);
      d1=-2*(state.homePos[passrer_id].y*v*v-state.awayPos[marker_id].y*w*w);
      e1=(state.homePos[passrer_id].x*state.homePos[passrer_id].x+state.homePos[passrer_id].y*state.homePos[passrer_id].y)*v*v-(state.awayPos[marker_id].x*state.awayPos[marker_id].x+state.homePos[receiver_id].y*state.homePos[receiver_id].y)*w*w;
    temp=pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2);
    if(((b1*pow(a2,2) + a1*pow(b2,2))!=0 || a2!=0)&&temp>=0)
    {
       x3=-(c2 - (b2*(pow(a2,2)*d1 + a2*pow(temp,(0.5)) + 2*a1*b2*c2 - a2*b2*c1))/(2*(b1*pow(a2,2) + a1*pow(b2,2))))/a2 ;
       y3=-(pow(a2,2)*d1 + a2*pow(temp,(0.5)) + 2*a1*b2*c2 - a2*b2*c1)/(2*(b1*pow(a2,2) + a1*pow(b2,2))) ;
      
       x4=-(c2 - (b2*(pow(a2,2)*d1 - a2*pow(temp,(0.5)) + 2*a1*b2*c2 - a2*b2*c1))/(2*(b1*pow(a2,2) + a1*pow(b2,2))))/a2 ;
       y4=-(pow(a2,2)*d1 - a2*pow(temp,(0.5)) + 2*a1*b2*c2 - a2*b2*c1)/(2*(b1*pow(a2,2) + a1*pow(b2,2))) ;
    }
    else 
    {
      intersecting2=false;
    }

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!selecting point!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    double max_theta=0,maxX,maxY;
    Vector2D<int> GoalPoint(OPP_GOAL_X,OPP_GOAL_Y);
    Vector2D<int> GoalPole;
    
    if(Vector2D<int>::dist(Vector2D<int>(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y),Vector2D<int>(OPP_GOAL_X,OPP_GOAL_MAXY)) > Vector2D<int>::dist(Vector2D<int>(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y),Vector2D<int>(OPP_GOAL_X,OPP_GOAL_MINY)))
    {
      GoalPole.x=OPP_GOAL_X;
      GoalPole.y=OPP_GOAL_MAXY;
    }
    else
    {
      GoalPole.x=OPP_GOAL_X;
      GoalPole.y=OPP_GOAL_MINY;
    }//finding the goal pole further from the goalkeeper

    if(intersecting2==true && intersecting1==true && ((x1<x3<x2 && y1<y3<y2)||(x2<x3<x1 && y2<y3<y1)||(x1<x4<x2 && y1<y4<y2)||(x2<x4<x1 && y2<y4<y1))) // case 1 : both ellipses interect and one intersectin pt of marker's ellipse lies between those of the receiver's
    {
      if((x1<x3<x2 && y1<y3<y2)||(x2<x3<x1 && y2<y3<y1)) //X3 lies between X1 and X2 
      {
        if((x3<x2<x4 && y3<y2<y4)||(x4<x2<x3 && y4<y2<y3)) //X2 lies between X3 and X4
        {
          Point1.x=x2;
          Point1.y=y2;
          Point2.x=x4;
          Point2.y=y4;
          
          if(Point1.x<Point2.x)
          {
              for (int x= Point1.x; x < Point2.x; ++x)
              {
                if(b2!=0)
                {
                   int y=-(a2*x+c2)/b2;
                  if(checkPointInField( Vector2D<int> (x,y)))
                  {
                    double theta=fabs(normalizeAngle(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y))-Vector2D<int>::angle(Vector2D<int>(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y),GoalPole)));
                    if(theta>max_theta) 
                      {
                        max_theta=theta;
                        maxX=x;
                        maxY=y;
                      }  
                  }
                  
                }
               
              }
          }
          else
          {
              for (int x= Point2.x; x < Point1.x; ++x)
              {
                if(b2!=0)
                {
                   int y=-(a2*x+c2)/b2;
                  if(checkPointInField( Vector2D<int> (x,y)))
                  {
                     double theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
                    if(theta>max_theta) 
                      {
                        max_theta=theta;
                        maxX=x;
                        maxY=y;
                      }

                  }
                 
                }
               
              }
          }

          passPoint.x=maxX;
          passPoint.y=maxY;
        }
        else if((x3<x1<x4 && y3<y1<y4)||(x4<x1<x3 && y4<y1<y3)) //X1 lies between X3 and X4
        {
          Point1.x=x1;
          Point1.y=y1;
          Point2.x=x4;
          Point2.y=y4;

          if(Point1.x<Point2.x)
          {
              for (int x= Point1.x; x < Point2.x; ++x)
              {
                if(b2!=0)
                {
                  int y=-(a2*x+c2)/b2;
                  if(checkPointInField( Vector2D<int> (x,y)))
                  {
                     double theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
                    if(theta>max_theta) 
                      {
                        max_theta=theta;
                        maxX=x;
                        maxY=y;
                      }

                  }
                 
                }
              }
          }
           else
          {
              for (int x= Point2.x; x < Point1.x; ++x)
              {
                if(b2!=0)
                {
                   int y=-(a2*x+c2)/b2;
                  if(checkPointInField( Vector2D<int> (x,y)))
                  {
                      double theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
                    if(theta>max_theta) 
                      {
                        max_theta=theta;
                        maxX=x;
                        maxY=y;
                      }
                  }
                  
                }
               
              }
          }

          passPoint.x=maxX;
          passPoint.y=maxY;
        }
        
      }
      else if((x1<x4<x2 && y1<y4<y2)||(x2<x4<x1 && y2<y4<y1)) //X4 lies between X1 and X2 
      {
         if((x3<x2<x4 && y3<y2<y4)||(x4<x2<x3 && y4<y2<y3)) //X2 lies between X3 and X4
        {
          Point1.x=x2;
          Point1.y=y2;
          Point2.x=x3;
          Point2.y=y3;

          if(Point1.x<Point2.x)
          {
              for (int x= Point1.x; x < Point2.x; ++x)
              {
                if(b2!=0)
               {
                int y=-(a2*x+c2)/b2;
                if(checkPointInField( Vector2D<int> (x,y)))
                {
                  double theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
                  if(theta>max_theta) 
                    {
                      max_theta=theta;
                      maxX=x;
                      maxY=y;
                    }
                }
                
               }
              }
          }
          else
          {
              for (int x= Point2.x; x < Point1.x; ++x)
              {
                if(b2!=0)
                {
                   int y=-(a2*x+c2)/b2;
                  if(checkPointInField( Vector2D<int> (x,y)))
                  {
                    double theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
                    if(theta>max_theta) 
                      {
                        max_theta=theta;
                        maxX=x;
                        maxY=y;
                      }
                  }
                  
                }
               
              }
          }
          passPoint.x=maxX;
          passPoint.y=maxY;
        }
        else if((x3<x1<x4 && y3<y1<y4)||(x4<x1<x3 && y4<y1<y3)) //X1 lies between X3 and X4
        {
          Point1.x=x1;
          Point1.y=y1;
          Point2.x=x3;
          Point2.y=y3;

          if(Point1.x<Point2.x)
          {
              for (int x= Point1.x; x < Point2.x; ++x)
              {
                if(b2!=0)
                {
                  int y=-(a2*x+c2)/b2;
                  if(checkPointInField( Vector2D<int> (x,y)))
                  {
                    double theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
                    if(theta>max_theta) 
                      {
                        max_theta=theta;
                        maxX=x;
                        maxY=y;
                    }
                  }
                  
                }
              }
          }
          else
          {
              for (int x= Point2.x; x < Point1.x; ++x)
              {
                if(b2!=0)
                {
                   int y=-(a2*x+c2)/b2;
                  if(checkPointInField( Vector2D<int> (x,y)))
                  {
                    double theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
                    if(theta>max_theta) 
                    {
                      max_theta=theta;
                      maxX=x;
                      maxY=y;
                    }
                  }
                  
                }
               
              }
          }
          passPoint.x=maxX;
          passPoint.y=maxY;
        }
      }

    }
    else if(intersecting1==true) //case 2 both ellipses intersect the line but have no pt of intersection between themselves
    {
      Point1.x=x1;
      Point1.y=y1;
      Point2.x=x2;
      Point2.y=y2;

      if(Point1.x<Point2.x)
          {
              for (int x= Point1.x; x < Point2.x; ++x)
              {
                if(b2!=0)
                {
                  int y=-(a2*x+c2)/b2;
                  if(checkPointInField( Vector2D<int> (x,y)))
                  {
                    double theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
                   if(theta>max_theta) 
                    {
                      max_theta=theta;
                      maxX=x;
                      maxY=y;
                    }
                  }
                  
                }
              }
          }
        else
          {
              for (int x= Point2.x; x < Point1.x; ++x)
              {
                if(b2!=0)
                {
                   int y=-(a2*x+c2)/b2;
                  if(checkPointInField( Vector2D<int> (x,y)))
                  {
                    double theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
                    if(theta>max_theta) 
                      {
                        max_theta=theta;
                        maxX=x;
                        maxY=y;
                      }
                  }
                  
                }
               
              }
          }
          passPoint.x=maxX;
          passPoint.y=maxY;

    }
    else  // case 3 when the ellipse of ball/receiver interaction does not intersect the line of receiver/marker interaction 
    {
      c1,a1=c1=(u*u-w*w);
      b1=-2*(state.homePos[passrer_id].x*u*u-state.homePos[receiver_id].x*w*w);
      d1=-2*(state.homePos[passrer_id].y*u*u-state.homePos[receiver_id].y*w*w);
      e1=(state.homePos[passrer_id].x*state.homePos[passrer_id].x+state.homePos[passrer_id].y*state.homePos[passrer_id].y)*u*u-(state.homePos[receiver_id].x*state.homePos[receiver_id].x+state.homePos[receiver_id].y*state.homePos[receiver_id].y)*w*w;

      a2=(OPP_GOAL_Y-state.homePos[receiver_id].y);
      b2=(OPP_GOAL_X-state.homePos[receiver_id].x);
      c2=(OPP_GOAL_X-state.homePos[receiver_id].x)*OPP_GOAL_Y-(OPP_GOAL_Y-state.homePos[receiver_id].y)*OPP_GOAL_X;
      temp=pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2);
       if(temp>=0)
      {
       x1=-(c2 - (b2*(pow(a2,2)*d1 + a2*pow(temp,(0.5)) + 2*a1*b2*c2 - a2*b2*c1))/(2*(b1*pow(a2,2) + a1*pow(b2,2))))/a2 ;
       y1=-(pow(a2,2)*d1 + a2*pow(temp,(0.5)) + 2*a1*b2*c2 - a2*b2*c1)/(2*(b1*pow(a2,2) + a1*pow(b2,2))) ;
      
       x2=-(c2 - (b2*(pow(a2,2)*d1 - a2*pow(temp,(0.5)) + 2*a1*b2*c2 - a2*b2*c1))/(2*(b1*pow(a2,2) + a1*pow(b2,2))))/a2 ;
       y2=-(pow(a2,2)*d1 - a2*pow(temp,(0.5)) + 2*a1*b2*c2 - a2*b2*c1)/(2*(b1*pow(a2,2) + a1*pow(b2,2))) ;
      }
      else
      {
        x1=x2=y1=y2=0;
      }  
      if(fabs(x1-OPP_GOAL_X)+fabs(y1-OPP_GOAL_Y)>fabs(x2-OPP_GOAL_X)+fabs(y2-OPP_GOAL_Y))
       {
        passPoint.x=x2;
        passPoint.y=y2;
       }
       else
       {
        passPoint.x=x1;
        passPoint.y=y1;
       }
    }  // passPoint selected for passing 
  return passPoint;
}

Vector2D<int> findPointForPassingNaive(int passrer_id,int receiver_id,int marker_id,krssg_ssl_msgs::BeliefState state)
{
  return Vector2D<int>(state.homePos[receiver_id].x,state.homePos[receiver_id].y);
}