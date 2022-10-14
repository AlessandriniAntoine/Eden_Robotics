#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "inverse_kinematics";

// For Block inverse_kinematics/Command/Subscribe
SimulinkSubscriber<geometry_msgs::Point, SL_Bus_inverse_kinematics_geometry_msgs_Point> Sub_inverse_kinematics_482;

// For Block inverse_kinematics/config/Publish
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_inverse_kinematics_sensor_msgs_JointState> Pub_inverse_kinematics_487;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

