#include "refPlays.hpp"

namespace Strategy{
	// Halt Referee play function starts
	HaltRP::HaltRP(const krssg_ssl_msgs::BeliefState& state) : RefereePlay(state, HALT) {
		
		name = "Halt RefereePlay";
		assert(HomeTeam::SIZE == 6);

		Tactic::Param param;
		for(int i=0;i<6;i++){
			roleList[i].push_back(std::make_pair("TStop", param));
		}
	}

	void HaltRP::updateParam() {
		Tactic::Param param;
		for(int i=0;i<6;i++){
			roleList[i].clear();
			roleList[i].push_back(std::make_pair("TStop", param));
		}
	}
	// Halt Referee play functions end

	// Add the remaining class functions here 
}