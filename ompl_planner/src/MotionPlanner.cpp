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


Planning::Planning(vector<krssg_ssl_msgs::point_2d> &v,int n, krssg_ssl_msgs::point_SF gui_msgs, int BOT_ID){
  init(v,n,gui_msgs, BOT_ID);
}

void Planning::init(vector<krssg_ssl_msgs::point_2d> &v,int n, krssg_ssl_msgs::point_SF gui_msgs, int BOT_ID)
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


  for(int i=0, j=0;i<n;i++, j++){
    if(j==BOT_ID)
    {
      i--;
      continue;
    }
    xc[i] = v[j].x;
    yc[i] = v[j].y;
  }

}




bool Planning::plan(int start_row, int start_col, int goal_row, int goal_col){
  if (!ss_)
    return false;

  ob::ScopedState<> start(ss_->getStateSpace());
  start[0] = start_row;
  xstart = start_row;
  ystart = start_col;
  xgoal = goal_row;
  ygoal = goal_col;
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
  return false ;
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
  if(sqrt(pow((xstart-w),2)+pow((ystart-h),2))<=self_radius)
    return true;
  if(sqrt(pow((xgoal-w),2)+pow((ygoal-h),2))<=self_radius)
    return true;
  for (int i = 0; i < numObstacles; ++i){
    if (sqrt(pow((xc[i]-w),2)+pow((yc[i]-h),2))<=obs_radius){
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

