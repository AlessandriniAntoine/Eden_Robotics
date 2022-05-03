#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "axes";

// For Block axes/1 2/Deplacement/Subscribe
SimulinkSubscriber<sensor_msgs::Joy, SL_Bus_axes_sensor_msgs_Joy> Sub_axes_7;

// For Block axes/1 2/Deplacement/angles_reference/Subscribe
SimulinkSubscriber<geometry_msgs::Quaternion, SL_Bus_axes_geometry_msgs_Quaternion> Sub_axes_47;

// For Block axes/Subscribe1
SimulinkSubscriber<std_msgs::Int16, SL_Bus_axes_std_msgs_Int16> Sub_axes_64;

// For Block axes/1 2/publisher_position_robot/Publish
SimulinkPublisher<geometry_msgs::Point, SL_Bus_axes_geometry_msgs_Point> Pub_axes_120;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

