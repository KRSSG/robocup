#ifndef _MOTION_PLANNER_H_
#define _MOTION_PLANNER_H_
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <boost/bind.hpp>
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerData.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;
using namespace cv;

template<typename T>
boost::shared_ptr<T> make_shared_ptr(std::shared_ptr<T>& ptr)
{
    return boost::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset();});
}

template<typename T>
std::shared_ptr<T> make_shared_ptr(boost::shared_ptr<T>& ptr)
{
    return std::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset();});
}

typedef struct 
{
  double x;
  double y;
}point;

typedef struct 
{
  double stepSize;
  double maxIteration;
  double biasParam;
  int planner_selector;
}gui_msg;

typedef struct {
  double xrange[2];
  double yrange[2];
} RANGE;

typedef struct
{
    double x;
    double y;
}pos;

class Planning{
  public:
    Planning(std::vector<pos> &v,int n, gui_msg gui_msgs);
    void init(std::vector<pos> &v,int n, gui_msg gui_msgs);
    void CreateCircle();
    //void PlannerSelector();
    bool isStateValid(const ob::State *state) const;
    void planWithSimpleSetup();
    void drawPath();
    void output();
    void planSimple();
    bool plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col);
    vector<point> recordSolution();
    bool isStateValid1(const ob::State *state);
    void drw();

  private:
    double* xc;
    double* yc;
    double* r;
    // Number of obstacles in space.
    int numObstacles;
    // Start position in space
    double xStart;
    double yStart;
    // Goal position in space
    double xGoal;
    double yGoal;
    // Max. distance toward each sampled position we
    // should grow our tree
    double stepSize;
    // Boundaries of the space
    double xLeft;
    double xRight;
    double yTop;
    double yBottom;
    int selector;


    int maxWidth_;
    int maxHeight_;
    og::SimpleSetupPtr ss_;

};
#endif
