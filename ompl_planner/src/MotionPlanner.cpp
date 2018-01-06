/**
 * \file MotionPlanner.cpp
 * @brief      Source Code for planner.
 * 
 * @see listener.cpp
 * @see MotionPlanner.h 
 */

#include "MotionPlanner.h"
#include <boost/make_shared.hpp>
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;


std::vector<krssg_ssl_msgs::point_2d> vect,vect1;

/**
 * \brief Describing Length of Window used for Smooth Path
 */
int windowSize = 50;
// windowSize = 50;  

std::vector<krssg_ssl_msgs::point_2d> publishingPoint;


Planning::Planning(vector<krssg_ssl_msgs::point_2d> &v,int n, krssg_ssl_msgs::point_SF gui_msgs){
  init(v,n,gui_msgs);
}


/**
 * @brief      Smooth Path using neighbour points.
 *
 *Smooth path using "windowSize" points, half of which are behind
 *current point and half after it
 *@see        windowSize
 * @param      v     Vector of path 
 * @return     Vector of smooth path
 */
void simplifyPoint(std::vector<krssg_ssl_msgs::point_2d>& v){
  std::vector<krssg_ssl_msgs::point_2d> newPoints;

  windowSize = windowSize/2 > v.size()?v.size():windowSize;
  for (int i = 0; i < windowSize/2; ++i)
  {
    newPoints.push_back(v[i]);
  }
  for (int i = windowSize/2; i < v.size() - windowSize/2; ++i)
  {
    float X = 0;
    float Y = 0;
    for (int j = i - windowSize/2; j <= i + windowSize/2; ++j)
    {
      X += v[j].x;
      Y += v[j].y;
    }
    krssg_ssl_msgs::point_2d p;
    p.x = X/(windowSize + 1);
    p.y = Y/(windowSize + 1);
    newPoints.push_back(p);
  }
  float deltaX = (newPoints[windowSize/2].x - newPoints[0].x)*2/windowSize;
  float deltaY = (newPoints[windowSize/2].y - newPoints[0].y)*2/windowSize;
  for (int i = 0; i < windowSize/2; ++i)
  {
    newPoints[i].x = newPoints[0].x + i*deltaX;
    newPoints[i].y = newPoints[0].y + i*deltaY;
  }
  deltaY = (v[v.size() - 1].y - newPoints[v.size() - windowSize/2 - 1].y)*2/windowSize;
  deltaX = (v[v.size() - 1].x - newPoints[v.size() - windowSize/2 - 1].x)*2/windowSize;
  for (int i = 0; i < windowSize/2; ++i)
  {
    krssg_ssl_msgs::point_2d p;
    p.x = newPoints[v.size() - windowSize/2 - 1].x + (i+1)*deltaX;
    p.y = newPoints[v.size() - windowSize/2 - 1].y + (i+1)*deltaY; 
    newPoints.push_back(p);
  }
  v = newPoints;
}

/**
 * @brief      Function for converting path into smooth path.
 * @see        windowSize
 * @param      v     Vector of path
 * Generate new points using next "windowSize" point to smooth path
 */
void simplifyWindow(vector<krssg_ssl_msgs::point_2d> & v){
  std::vector<krssg_ssl_msgs::point_2d> newPoints;
  for (int i = 0; i <= v.size(); i += windowSize)
  {
    int lastPtr = i + windowSize - 1;
    if (lastPtr > v.size() - 1)
    {
      lastPtr = v.size() - 1;
    }
    int currentWindowSize = lastPtr - i + 1;
    float deltaX = (v[lastPtr].x - v[i].x)/currentWindowSize;
    float deltaY = (v[lastPtr].y - v[i].y)/currentWindowSize;
    for (int j = i; j <= lastPtr; j++)
    {
      krssg_ssl_msgs::point_2d p;
      p.x = v[i].x + (j - i + 1)*deltaX;
      p.y = v[i].y + (j - i + 1)*deltaY;
      newPoints.push_back(p);
    }
  }
  v = newPoints;
}

void Planning::init(vector<krssg_ssl_msgs::point_2d> &v,int n, krssg_ssl_msgs::point_SF gui_msgs)
{

  // stepSize=gui_msgs.step_size;
  // Boundaries of the space
  xLeft=-HALF_FIELD_MAXX_OMPL;
  xRight=HALF_FIELD_MAXX_OMPL;
  yTop=HALF_FIELD_MAXY_OMPL;
  yBottom=-HALF_FIELD_MAXY_OMPL;
  selector=gui_msgs.max_iteration;

  n--;
  numObstacles = n;
  xc = new double[numObstacles];
  yc = new double[numObstacles];

  for(int i=0;i<n;i++){
    xc[i] = v[i+1].x;
    yc[i] = v[i+1].y;
  }

}




bool Planning::plan(int start_row, int start_col, int goal_row, int goal_col){
  if (!ss_)
    return false;

  ob::ScopedState<> start(ss_->getStateSpace());
  start[0] = start_row;
  xstart = start_row;
  ystart = start_col;
  start[1] = start_col;
  ob::ScopedState<> goal(ss_->getStateSpace());
  goal[0] = goal_row;
  goal[1] = goal_col;
  selector = 3;
  ss_->setStartAndGoalStates(start, goal);
  for (int i = 0 ; i < 1 ; ++i){
        if(selector == 1){
        ob::PlannerPtr planner(new og::PRM(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 2){
        ob::PlannerPtr planner(new og::RRT(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 3){
        ob::PlannerPtr planner(new og::RRTConnect(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 4){
        ob::PlannerPtr planner(new og::RRTstar(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 5){
        ob::PlannerPtr planner(new og::LBTRRT(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 6){
        ob::PlannerPtr planner(new og::LazyRRT(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 7){
        ob::PlannerPtr planner(new og::TRRT(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 8){
        ob::PlannerPtr planner(new og::pRRT(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 9){
        ob::PlannerPtr planner(new og::EST(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }
      else{
        ob::PlannerPtr planner(new og::RRTConnect(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }
    ss_->solve();
  }
  const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
  if (ss_->haveSolutionPath()){
    cout<<"__HERE__\n"<<endl;
    ss_->simplifySolution();
    og::PathGeometric &p = ss_->getSolutionPath();
    ss_->getPathSimplifier()->simplifyMax(p);
    ss_->getPathSimplifier()->smoothBSpline(p);
    cout<<"__FUNC_END__"<<endl;
    return true;
  }
  return false;
}



vector<krssg_ssl_msgs::point_2d> Planning::recordSolution(){
  vect1.clear();
  cout<<"recordSolution start\n";
  if (!ss_ || !ss_->haveSolutionPath())
    {
      vect1.clear();
      return vect1;
    }
  og::PathGeometric &p = ss_->getSolutionPath();
  p.interpolate();
  for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
  {
    const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
    const int h = std::min(maxHeight_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
   krssg_ssl_msgs::point_2d s;
    s.x=w;
    s.y=h;
    vect1.push_back(s);
  }
  return vect1;
}

bool Planning::isStateValid(const ob::State *state) const {

  const int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
  const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);
  if(sqrt(pow((xstart-w),2)+pow((ystart-h),2))<=radius)
    return true;
  for (int i = 0; i < numObstacles; ++i){
    if (sqrt(pow((xc[i]-w),2)+pow((yc[i]-h),2))<=radius){
      return false;
    }
  }
  return true;
}

/**
 * xLeft, xRight for x Dimension
 * 
 * yLeft, yRight for y Dimension
 */
void Planning::planSimple(){
  //if throwing an error related to shared pointer conflict b/w boost and std, uncomment the follwing line and comment the 2nd one.
  // boost::shared_ptr<ob::RealVectorStateSpace> space(boost::make_shared<ob::RealVectorStateSpace>());
  auto space(std::make_shared<ob::RealVectorStateSpace>());
  space->addDimension(xLeft, xRight);
  space->addDimension(yBottom, yTop);
  maxWidth_ = xRight - xLeft;
  maxHeight_ = yTop - yBottom;
  // ss_ = make_shared_ptr(ss_);
  std::shared_ptr<og::SimpleSetup> std_ss_;
  //do the same here too.
  ss_ = std::make_shared<og::SimpleSetup>(space);

  ss_->setStateValidityChecker([this](const ob::State *state) {return isStateValid(state);});
  space->setup();
  ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
}

