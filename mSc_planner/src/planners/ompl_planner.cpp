#include "ros/ros.h"
#include "planners/ompl_planner.h"

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;
using namespace krssg_ssl_msgs;

namespace Navigation {
	OMPL_Planner::OMPL_Planner(krssg_ssl_msgs::path_data::Request &request) {
		start_point  = request.botPos;
		final_point  = request.targetPos;
		numObstacles = request.obs_size;

        step_size = 100.0;
        goal_bias = 0.01;

		for(int obs=0; obs<numObstacles; ++obs) {
			obstacles.push_back(request.obs_vec[obs]);
		}
	}

	OMPL_Planner::~OMPL_Planner() {

	}

	bool OMPL_Planner::isStateValid(const ob::State *state) {
	  const ob::SE2StateSpace::StateType *state_2d= state->as<ob::SE2StateSpace::StateType>();
	  const double &x(state_2d->getX()), &y(state_2d->getY());

	  for (int i = 0; i < numObstacles; ++i){
	    if (sqrt(pow((obstacles[i].x - x), 2)+pow((obstacles[i].y - y), 2)) <= 10.0){
	      return false;
	    }
	  }
		return true;
	}

	krssg_ssl_msgs::path_data::Response& OMPL_Planner::plan() {
		ob::StateSpacePtr space(new ob::SE2StateSpace());
		ob::RealVectorBounds bounds(2);
		bounds.setLow(0, -HALF_FIELD_MAXX);
		bounds.setHigh(0, HALF_FIELD_MAXX);
		bounds.setLow(1, -HALF_FIELD_MAXY);
		bounds.setHigh(1, HALF_FIELD_MAXY);
		space->as<ob::SE2StateSpace>()->setBounds(bounds);

		og::SimpleSetup setup(space);

		setup.setStateValidityChecker(boost::bind(&OMPL_Planner::isStateValid, this, _1));


		ob::ScopedState<ob::SE2StateSpace> start(space);
	  	start->setXY(start_point.x, start_point.y);

		ob::ScopedState<ob::SE2StateSpace> final(space);
	  	final->setXY(final_point.x, final_point.y);

	  	setup.setStartAndGoalStates(start, final);


	  	ob::PlannerPtr rrtc_planner(new og::RRTConnect(setup.getSpaceInformation()));
	  	// ob::PlannerPtr rrtc_planner(new og::RRTstar(setup.getSpaceInformation()));

	  	rrtc_planner->as<og::RRTConnect>()->setRange(step_size);
	  	// rrtc_planner->as<og::RRTstar>()->setRange(step_size);
	  	// rrtc_planner->as<og::RRTstar>()->setGoalBias(goal_bias);


       	        setup.setPlanner(rrtc_planner);

	        ob::PlannerStatus solved = setup.solve();

	   if (solved) {
	   	ompl::geometric::PathGeometric final_path = setup.getSolutionPath();

	   	std::vector<ompl::base::State *> inter_states = final_path.getStates();

	   	geometry_msgs::Pose2D temp_point;
	   	int i=0;
	   	response.control_points.resize(50);
	   	for(std::vector<ompl::base::State *>::iterator itr =
	   		inter_states.begin(); itr != inter_states.end()&&i!=50; ++itr,i++) {
	   		const ob::SE2StateSpace::StateType *inter_instance = (*itr)->as<ob::SE2StateSpace::StateType>();
	   		temp_point.x = inter_instance->getX();
	   		temp_point.y = inter_instance->getY();
	   		response.control_points[i]=(temp_point);
	   	}
	   } else {
	   	ROS_INFO("No path exists !");
	   	response.control_points.resize(0);
	   }

	   cout<<"Here4"<<response.control_points.size()<<endl;
	   return response;
	}
}
