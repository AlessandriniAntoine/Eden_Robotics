#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include "inverse_kinematics_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_inverse_kinematics_geometry_msgs_Point const* busPtr);
void convertToBus(SL_Bus_inverse_kinematics_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_inverse_kinematics_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_inverse_kinematics_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(sensor_msgs::JointState* msgPtr, SL_Bus_inverse_kinematics_sensor_msgs_JointState const* busPtr);
void convertToBus(SL_Bus_inverse_kinematics_sensor_msgs_JointState* busPtr, sensor_msgs::JointState const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_inverse_kinematics_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_inverse_kinematics_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);


#endif
