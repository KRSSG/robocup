/**
 * \file MotionPlanner.h
 * \brief Header File for Planner
 */

#ifndef _MOTION_PLANNER_H_
#define _MOTION_PLANNER_H_

#include "krssg_ssl_msgs/BeliefState.h"
#include "krssg_ssl_msgs/planner_path.h"
#include "krssg_ssl_msgs/point_2d.h"
#include "krssg_ssl_msgs/point_SF.h"
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
#include <ostream>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;

#define BS_TO_OMPL 0.1
#define OMPL_TO_BS 10
#define HALF_FIELD_MAXX 6000
#define HALF_FIELD_MAXY 4500
#define HALF_FIELD_MAXX_OMPL HALF_FIELD_MAXX*BS_TO_OMPL
#define HALF_FIELD_MAXY_OMPL HALF_FIELD_MAXY*BS_TO_OMPL
#define self_radius 270*BS_TO_OMPL
#define obs_radius 257*BS_TO_OMPL

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
/**
 * @brief      Class for planning.
 */
class Planning{
  public:
    /**
     * @brief      Constructor for planning class.
     *
     * @param      v         Position of all bots
     * @param[in]  n         Total Number of Bots
     * @param[in]  gui_msgs  msg from gui node
     */
    Planning(std::vector<krssg_ssl_msgs::point_2d> &v,int n, krssg_ssl_msgs::point_SF gui_msgs, int BOT_ID);
    void init(std::vector<krssg_ssl_msgs::point_2d> &v,int n, krssg_ssl_msgs::point_SF gui_msgs, int BOT_ID);
    
    /**
 * @brief      Determines if ompl state is valid.
 *
 * @param[in]  state  ompl state
 *
     *Check if obstacle is not on path
 * @return     True if state valid 1, False otherwise.
 */
    bool isStateValid(const ob::State *state) const;
    // void planWithSimpleSetup();
    // void drawPath();
    // void output();

/**
 * @brief      Set Dimensions for ompl space
 * 
 * Boundaries of space
 */
    void planSimple();


    /**
 * @brief      Initialise ompl state with path.
 *
 * @param[in]  start_row  start point coordinate
 * @param[in]  start_col  start point coordinate
 * @param[in]  goal_row   end point coordinate
 * @param[in]  goal_col   end point coordinate
 *
 *Find path from start point to end point.
 *Simplify Path
 *
 * @return     return True if path is found
 */
    bool plan(int start_row, int start_col, int goal_row, int goal_col);
/**
 * @brief      Get vector of points on path 
 *
 *Path after Interpolation
 *
 * @return     Vector of points on path
 */
    vector<krssg_ssl_msgs::point_2d> recordSolution();
    // bool isStateValid1(const ob::State *state);

  private:
    double* xc;
    double* yc;
    double* r;
    double xstart, ystart, xgoal, ygoal;
    /**
     * Number of obstacles in space
     */

    int numObstacles;
    /**
     * @brief Left coordinate of space
     */
    double xLeft;
    /**
     * @brief Right coordinate of space
     * @see planSimple()
     */
    double xRight;
    /**
     * @brief Top coordinate of space
     * @see planSimple()
     */
    double yTop;
    /**
     * @brief Bottom coordinate of space
     * @see planSimple()
     */
    double yBottom;
    /**
     * @brief Planner Selector
     * @see planSimple()
     */
    int selector;

    /**
     * @brief Width of space
     */
    int maxWidth_;
    /**
     * @brief Height of space
     * @see planSimple()
     */
    int maxHeight_;

    /**
     * Ompl space ptr
     * @see planSimple()
     */
    og::SimpleSetupPtr ss_;

};
#endif
