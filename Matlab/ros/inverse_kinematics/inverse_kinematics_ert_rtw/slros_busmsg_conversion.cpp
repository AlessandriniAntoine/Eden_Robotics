#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_inverse_kinematics_geometry_msgs_Point and geometry_msgs::Point

void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_inverse_kinematics_geometry_msgs_Point const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_inverse_kinematics_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_inverse_kinematics_ros_time_Time and ros::Time

void convertFromBus(ros::Time* msgPtr, SL_Bus_inverse_kinematics_ros_time_Time const* busPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->nsec =  busPtr->Nsec;
  msgPtr->sec =  busPtr->Sec;
}

void convertToBus(SL_Bus_inverse_kinematics_ros_time_Time* busPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  busPtr->Nsec =  msgPtr->nsec;
  busPtr->Sec =  msgPtr->sec;
}


// Conversions between SL_Bus_inverse_kinematics_sensor_msgs_JointState and sensor_msgs::JointState

void convertFromBus(sensor_msgs::JointState* msgPtr, SL_Bus_inverse_kinematics_sensor_msgs_JointState const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/JointState");

  convertFromBusVariablePrimitiveArray(msgPtr->effort, busPtr->Effort, busPtr->Effort_SL_Info);
  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBusVariableStringArray(msgPtr->name, busPtr->Name, busPtr->Name_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->position, busPtr->Position, busPtr->Position_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->velocity, busPtr->Velocity, busPtr->Velocity_SL_Info);
}

void convertToBus(SL_Bus_inverse_kinematics_sensor_msgs_JointState* busPtr, sensor_msgs::JointState const* msgPtr)
{
  const std::string rosMessageType("sensor_msgs/JointState");

  convertToBusVariablePrimitiveArray(busPtr->Effort, busPtr->Effort_SL_Info, msgPtr->effort, slros::EnabledWarning(rosMessageType, "effort"));
  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBusVariableStringArray(busPtr->Name, busPtr->Name_SL_Info, msgPtr->name, slros::EnabledWarning(rosMessageType, "name"));
  convertToBusVariablePrimitiveArray(busPtr->Position, busPtr->Position_SL_Info, msgPtr->position, slros::EnabledWarning(rosMessageType, "position"));
  convertToBusVariablePrimitiveArray(busPtr->Velocity, busPtr->Velocity_SL_Info, msgPtr->velocity, slros::EnabledWarning(rosMessageType, "velocity"));
}


// Conversions between SL_Bus_inverse_kinematics_std_msgs_Header and std_msgs::Header

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_inverse_kinematics_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr->frame_id, busPtr->FrameId, busPtr->FrameId_SL_Info);
  msgPtr->seq =  busPtr->Seq;
  convertFromBus(&msgPtr->stamp, &busPtr->Stamp);
}

void convertToBus(SL_Bus_inverse_kinematics_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->FrameId, busPtr->FrameId_SL_Info, msgPtr->frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  busPtr->Seq =  msgPtr->seq;
  convertToBus(&busPtr->Stamp, &msgPtr->stamp);
}

