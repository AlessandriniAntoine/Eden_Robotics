
//
// File ros2nodeinterface.h
//
// Code generated for Simulink model 'InverseKinematics'.
//
// Model version                  : 4.20
// Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
// C/C++ source code generated on : Wed Jul 27 23:36:13 2022
//
#ifndef _ROS2_MATLAB_NODEINTERFACE_
#define _ROS2_MATLAB_NODEINTERFACE_
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "rclcpp/rclcpp.hpp"
class InverseKinematics;
#include "rtwtypes.h"
#include "InverseKinematics_types.h"
#include "slros_busmsg_conversion.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
namespace ros2 {
namespace matlab {
  //Semaphore using std::CV and std::mutex
  class Semaphore {
  public:
    std::mutex mMutex;
    std::condition_variable mCV;
    std::atomic_uint mCount;
    //
    Semaphore(int count = 0)
      : mCount(count) {
      }
    //
    inline void notify() {
      std::unique_lock<std::mutex> lock(mMutex);
      mCount++;
      mCV.notify_all();
    }
    //
    inline void wait() {
      std::unique_lock<std::mutex> lock(mMutex);
      while (mCount == 0) {
        mCV.wait(lock);
      }
      if (mCount)
        mCount--;
    }
  };
  //NodeInterface
  class NodeInterface {
    NodeInterface(const NodeInterface& );
    NodeInterface& operator=(const NodeInterface& );
    //
    rclcpp::Node::SharedPtr mNode;
    std::shared_ptr<InverseKinematics> mModel;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr mExec;
    //
    Semaphore mBaseRateSem;
    std::shared_ptr<std::thread> mBaseRateThread;
    std::shared_ptr<std::thread> mSchedulerThread;
    //
    //
    Semaphore mStopSem;
    volatile boolean_T mRunModel;
    //
    // InverseKinematics/Publisher/Publish
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr mPub_InverseKinematics_466;
    //
    // InverseKinematics/Subscribe
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr mSub_InverseKinematics_562;
    geometry_msgs::msg::Point::SharedPtr mLatestMsg_Sub_InverseKinematics_562;
    std::mutex mtx_Sub_InverseKinematics_562;
    //
  public:
    NodeInterface();
    ~NodeInterface();
    //
    void initialize(int argc, char * const argv[]);
    int run();
    void stop(void);
    void terminate(void);
    //
    boolean_T getStopRequestedFlag(void);
    void schedulerThreadCallback(void);
    void baseRateTask(void);
    // InverseKinematics/Publisher/Publish
    void create_Pub_InverseKinematics_466(const char *topicName, const rmw_qos_profile_t& qosProfile);
    void publish_Pub_InverseKinematics_466(const SL_Bus_geometry_msgs_Quaternion* inBus);
    // InverseKinematics/Subscribe
    void create_Sub_InverseKinematics_562(const char *topicName, const rmw_qos_profile_t& qosProfile);
    bool getLatestMessage_Sub_InverseKinematics_562(SL_Bus_geometry_msgs_Point* outBus);
    //
    rclcpp::Node::SharedPtr getNode() {
      return mNode;
    }
    //
    std::shared_ptr<InverseKinematics> getModel() {
      return mModel;
    }
  }; //class NodeInterface
  //
  std::shared_ptr<ros2::matlab::NodeInterface> getNodeInterface();
  // Helper for InverseKinematics/Publisher/Publish
  void create_Pub_InverseKinematics_466(const char *topicName, const rmw_qos_profile_t& qosProfile = rmw_qos_profile_default);
  void publish_Pub_InverseKinematics_466(const SL_Bus_geometry_msgs_Quaternion* inBus);
  // Helper for InverseKinematics/Subscribe
  void create_Sub_InverseKinematics_562(const char *topicName, const rmw_qos_profile_t& qosProfile = rmw_qos_profile_default);
  bool getLatestMessage_Sub_InverseKinematics_562(SL_Bus_geometry_msgs_Point* outBus);
  // Get QoS Settings from RMW
  inline rclcpp::QoS getQOSSettingsFromRMW(const rmw_qos_profile_t& qosProfile) {
      rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(qosProfile));
      if (RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL == qosProfile.durability) {
          qos.transient_local();
      } else {
          qos.durability_volatile();
      }
      if (RMW_QOS_POLICY_RELIABILITY_RELIABLE == qosProfile.reliability) {
          qos.reliable();
      } else {
          qos.best_effort();
      }
      return qos;
  }
}//namespace matlab
}//namespace ros2
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
#endif //_ROS2_MATLAB_ROS2CGEN_MULTIRATE_
