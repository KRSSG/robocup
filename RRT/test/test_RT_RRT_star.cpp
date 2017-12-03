#include "ros/ros.h"
#include "RT_RRT_star.hpp"



int main(int argc, char **argv) {
    srand(time(NULL));
    Utils::Point<int> start,finish;
    start.x=10;
    start.y=10;
    finish.x=6000;
    finish.y=4000;

    RT_RRT<int> test(start,finish);
    // test.setEndPoints(start,finish);
    // test.setCheckPointFunction(*(check));
    // test.setStepLength(200);
    // test.setHalfDimensions(3000.0,2000.0);
    // test.setBiasParameter(100);
    // test.setOrigin(origin);
    // test.setMaxIterations(10000);
    // test.plan();
    std::cout<<"#################################################"<<std::endl;
    while(1)
    {
        Utils::Point<int> now;
        now.x = rand()%6000;
        now.y = rand()%4000;
        std::pair<Utils::Point<int>,Utils::Point<int> > came_back = test.add_node_to_tree(now);
        // test.add_node_to_tree(now);
        std::cout<<"Generated a point "<<came_back.first.x<<","<<came_back.first.y<<" with parent "<<came_back.second.x<<","<<came_back.second.y<<std::endl;
        // std::cout<<"Generated point\n";
    }
    // for(int i=0;i<path.size();i++)
    //     cout<<path[i].x<<","<<path[i].y<<endl;
  //ros::init(argc, argv, "RT_RRT_star");

  ros::NodeHandle nh;

}