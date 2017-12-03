//#include "ros/ros.h"
#include "RT_RRT_star.cpp"
#include "ros/ros.h"
#include "krssg_ssl_msgs/planner_path.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include "krssg_ssl_msgs/point_2d.h"
#include "krssg_ssl_msgs/point_SF.h"
using namespace rrt;
using namespace std;

int p=0;
ros::Publisher pub;

void send_points(std::pair<Utils::Point<int>,Utils::Point<int> > child_parent)
{
    krssg_ssl_msgs::planner_path points;
     
    krssg_ssl_msgs::point_2d point;
    
     
        point.x=child_parent.first.x;
        point.y=child_parent.first.y;
        points.point_array.push_back(point);

        point.x=child_parent.second.x;
        point.y=child_parent.second.y;
        points.point_array.push_back(point);
       
      pub.publish(points); cout<<"data published "<<p++<<endl;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "self_rrt");
    ros::NodeHandle n;
    pub = n.advertise<krssg_ssl_msgs::planner_path>("self_rrt_pair", 1000);
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
    int x=1000;
    while(x--)
    {
        Utils::Point<int> now;
        now.x = rand()%6000;
        now.y = rand()%4000;
        std::pair<Utils::Point<int>,Utils::Point<int> > came_back = test.add_node_to_tree(now);
        // test.add_node_to_tree(now);
        send_points(came_back);
        std::cout<<"Generated a point "<<came_back.first.x<<","<<came_back.first.y<<" with parent "<<came_back.second.x<<","<<came_back.second.y<<std::endl;
        ros::Duration(0.10).sleep();
        // std::cout<<"Generated point\n";
    }
    // for(int i=0;i<path.size();i++)
    //     cout<<path[i].x<<","<<path[i].y<<endl;
  //ros::init(argc, argv, "RT_RRT_star");

  //ros::NodeHandle nh;

}
