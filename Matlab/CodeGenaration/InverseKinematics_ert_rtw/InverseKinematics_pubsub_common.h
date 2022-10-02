
//
// Code generated for Simulink model 'InverseKinematics'.
//
// Model version                  : 4.20
// Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
// C/C++ source code generated on : Wed Jul 27 23:36:14 2022
//
#ifndef _SLROS2_COMMON_H_
#define _SLROS2_COMMON_H_
#include "InverseKinematics_types.h"
#ifndef SET_QOS_VALUES
#define SET_QOS_VALUES(qosStruct, hist, dep, dur, rel)  \
    {                                                   \
        qosStruct.history = hist;                       \
        qosStruct.depth = dep;                          \
        qosStruct.durability = dur;                     \
        qosStruct.reliability = rel;                    \
    }
#endif
namespace ros2 {
    namespace matlab {
        // InverseKinematics/Publisher/Publish
        extern void create_Pub_InverseKinematics_466(const char *topicName, const rmw_qos_profile_t& qosProfile);
        extern void publish_Pub_InverseKinematics_466(const SL_Bus_geometry_msgs_Quaternion* inBus);
        // InverseKinematics/Subscribe
        extern void create_Sub_InverseKinematics_562(const char *topicName, const rmw_qos_profile_t& qosProfile);
        extern bool getLatestMessage_Sub_InverseKinematics_562(SL_Bus_geometry_msgs_Point* outBus);
    }
}
#endif // _SLROS2_COMMON_H_
