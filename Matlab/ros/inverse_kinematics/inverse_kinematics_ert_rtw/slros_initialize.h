#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"
#include "inverse_kinematics_types.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block inverse_kinematics/Subscriber/Subscribe
extern SimulinkSubscriber<geometry_msgs::Point, SL_Bus_inverse_kinematics_geometry_msgs_Point> Sub_inverse_kinematics_482;

// For Block inverse_kinematics/Publisher/Publish
extern SimulinkPublisher<geometry_msgs::Quaternion, SL_Bus_inverse_kinematics_geometry_msgs_Quaternion> Pub_inverse_kinematics_487;

void slros_node_init(int argc, char** argv);

#endif
