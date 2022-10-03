#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "inverse_kinematics_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_inverse_kinematics_geometry_msgs_Point const* busPtr);
void convertToBus(SL_Bus_inverse_kinematics_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr);

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_inverse_kinematics_geometry_msgs_Quaternion const* busPtr);
void convertToBus(SL_Bus_inverse_kinematics_geometry_msgs_Quaternion* busPtr, geometry_msgs::Quaternion const* msgPtr);


#endif
