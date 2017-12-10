#include "ros/ros.h"

#include "krssg_ssl_msgs/Vector2f.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include "krssg_ssl_msgs/SSL_DetectionFrame.h"
#include "krssg_ssl_msgs/SSL_DetectionBall.h"
#include "krssg_ssl_msgs/SSL_DetectionRobot.h"
#include "krssg_ssl_msgs/SSL_GeometryData.h"
#include "krssg_ssl_msgs/SSL_GeometryCameraCalibration.h"
#include "krssg_ssl_msgs/SSL_GeometryFieldSize.h"
#include "krssg_ssl_msgs/SSL_WrapperPacket.h"
#include "krssg_ssl_msgs/SSL_FieldLineSegment.h"
#include "krssg_ssl_msgs/SSL_FieldCircularArc.h"

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

   // Geometry Data
   int field_length,
       field_width,
       goal_width,
       goal_depth,
       boundary_width;

   vector<krssg_ssl_msgs::SSL_FieldLineSegment> lines;
   vector<krssg_ssl_msgs::SSL_FieldCircularArc> arcs;

   vector<krssg_ssl_msgs::SSL_GeometryCameraCalibration> cam_params;

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
    * @param[in]  vmsg  packet of data
    */
   void update_frame(const krssg_ssl_msgs::SSL_WrapperPacket *pkt);

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
   void print();

   /**
    * @brief      Gets the beliefstate message of the current instance.
    *
    * @return     The beliefstate message.
    */
   krssg_ssl_msgs::BeliefState get_beliefstate_msg();

   /**
    * @brief      Updates the geometry data
    *
    * @param[in]  geo  Instance of Geometry data
    */
   void update_geometry_data(const krssg_ssl_msgs::SSL_GeometryData *geo);

protected:
  /**
   * @brief      Update field parameters
   *
   * @param[in]  field  Instance of field data
   */
   void update_field_params(const krssg_ssl_msgs::SSL_GeometryFieldSize *field);

   /**
    * @brief      Update camera clibration paramters
    *
    * @param      camera_params  The calib
    */
   void update_camera_calib(const vector<
    krssg_ssl_msgs::SSL_GeometryCameraCalibration> &cam_params);
};
