#include "ros/ros.h"
#include <krssg_ssl_msgs/BeliefState.h>
#include <krssg_ssl_msgs/SSL_DetectionFrame.h>
#include <krssg_ssl_msgs/SSL_DetectionRobot.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>

#include <string.h>
#include <math.h>
#include <vector>
#include <queue>
#include <deque>
#include <fstream>

using namespace std;
using geometry_msgs::Pose2D;
using geometry_msgs::Point32;


/**
 * @brief      Core class for extracting belief state information
 * @param      
 */
class BeliefState {

   BeliefState *prev_msg;

   //Time
   ros::Time stamp;

   //Technical
   bool isteamyellow;
   int frame_number;
   double t_capture, t_sent;

   //Positions
   Pose2D ballPos;
   vector<Pose2D> awayPos, homePos;

   //Velocities
   Pose2D ballVel;
   vector<Pose2D> awayVel, homeVel;

   //Detections
   bool ballDetected;
   vector<bool> awayDetected, homeDetected;

public:

   /**
    * @brief      Default Constructer
    */
   BeliefState();

   /**
    * @brief      Default Constructer
    * 
    * @param[in]  isteamyellow  True if our team is yellow
    */
   BeliefState(bool isteamyellow);

   /**
    * @brief      Copy constructer
    *
    * @param[in]  msg  Instance of belief state
    */
   BeliefState(const BeliefState& msg);

   /**
    * @brief      Copy Constructer for frame data
    *
    * @param[in]  vmsg  packet of frame data
    */
   void update_frame(const krssg_ssl_msgs::SSL_DetectionFrame *vmsg);

   /**
    * @brief      Copy `bf` to current instance
    *
    * @param[in]  bf  Instance of belief state
    */
   void copy(const BeliefState& );

   /**
    * @brief      { function_description }
    */
   void initialise();

   /**
    * @brief      Print one instance of the belief state data
    */
   void print(string str);

   /**
    * @brief      Gets the beliefstate message of the current instance.
    *
    * @return     The beliefstate message.
    */
   krssg_ssl_msgs::BeliefState get_beliefstate_msg();

};
