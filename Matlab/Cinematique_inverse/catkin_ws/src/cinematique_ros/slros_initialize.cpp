#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "Cinematique_ROS";

// For Block Cinematique_ROS/Subscribe
SimulinkSubscriber<geometry_msgs::Point, SL_Bus_Cinematique_ROS_geometry_msgs_Point> Sub_Cinematique_ROS_1224;

// For Block Cinematique_ROS/Enabled Subsystem/Bassin/Publish
SimulinkPublisher<geometry_msgs::Point, SL_Bus_Cinematique_ROS_geometry_msgs_Point> Pub_Cinematique_ROS_1271;

// For Block Cinematique_ROS/Enabled Subsystem/Coude/Publish
SimulinkPublisher<geometry_msgs::Point, SL_Bus_Cinematique_ROS_geometry_msgs_Point> Pub_Cinematique_ROS_1319;

// For Block Cinematique_ROS/Enabled Subsystem/Epaule/Publish
SimulinkPublisher<geometry_msgs::Point, SL_Bus_Cinematique_ROS_geometry_msgs_Point> Pub_Cinematique_ROS_1314;

// For Block Cinematique_ROS/Enabled Subsystem/Poignet/Publish
SimulinkPublisher<geometry_msgs::Point, SL_Bus_Cinematique_ROS_geometry_msgs_Point> Pub_Cinematique_ROS_1324;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

