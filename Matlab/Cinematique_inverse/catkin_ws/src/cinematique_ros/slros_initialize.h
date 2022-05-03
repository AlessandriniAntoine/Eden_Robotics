#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block Cinematique_ROS/Subscribe
extern SimulinkSubscriber<geometry_msgs::Point, SL_Bus_Cinematique_ROS_geometry_msgs_Point> Sub_Cinematique_ROS_1224;

// For Block Cinematique_ROS/Enabled Subsystem/Bassin/Publish
extern SimulinkPublisher<geometry_msgs::Point, SL_Bus_Cinematique_ROS_geometry_msgs_Point> Pub_Cinematique_ROS_1271;

// For Block Cinematique_ROS/Enabled Subsystem/Coude/Publish
extern SimulinkPublisher<geometry_msgs::Point, SL_Bus_Cinematique_ROS_geometry_msgs_Point> Pub_Cinematique_ROS_1319;

// For Block Cinematique_ROS/Enabled Subsystem/Epaule/Publish
extern SimulinkPublisher<geometry_msgs::Point, SL_Bus_Cinematique_ROS_geometry_msgs_Point> Pub_Cinematique_ROS_1314;

// For Block Cinematique_ROS/Enabled Subsystem/Poignet/Publish
extern SimulinkPublisher<geometry_msgs::Point, SL_Bus_Cinematique_ROS_geometry_msgs_Point> Pub_Cinematique_ROS_1324;

void slros_node_init(int argc, char** argv);

#endif
