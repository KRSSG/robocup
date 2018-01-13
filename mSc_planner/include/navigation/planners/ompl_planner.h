#ifndef _OMPL_MOTION_PLANNER_H_
#define _OMPL_MOTION_PLANNER_H_
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

#include "geometry_msgs/Pose2D.h"
#include "krssg_ssl_msgs/path_data.h"
#include "krssg_ssl_msgs/obstacle.h"
#include "ssl_common/geometry.hpp"
#include "ssl_common/config.h"
#include "common.h"


namespace Navigation {
	class OMPL_Planner {
		public:
			OMPL_Planner(krssg_ssl_msgs::path_data::Request &request);
			~OMPL_Planner();

			bool isStateValid(const ompl::base::State *state);
			krssg_ssl_msgs::path_data::Response& plan();
			krssg_ssl_msgs::path_data::Response response;

		private:
			geometry_msgs::Pose2D start_point;
			geometry_msgs::Pose2D final_point;
			std::vector<krssg_ssl_msgs::obstacle> obstacles;

			int botID;

			double step_size;
			double goal_bias;

			int numObstacles;
	};
}
#endif
