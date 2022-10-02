
//
// File ros2nodeinterface.cpp
//
// Code generated for Simulink model 'InverseKinematics'.
//
// Model version                  : 4.21
// Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
// C/C++ source code generated on : Fri Jul 29 16:24:07 2022
//
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "rclcpp/rclcpp.hpp"
#include "InverseKinematics.h"
#include "ros2nodeinterface.h"
#include <thread>
#include <chrono>
#include <utility>
const std::string SLROSNodeName("InverseKinematics");
namespace ros2 {
namespace matlab {
NodeInterface::NodeInterface()
    : mNode()
    , mModel()
    , mExec()
    , mBaseRateSem()
    , mBaseRateThread()
    , mSchedulerThread()
    , mStopSem()
    , mRunModel(true){
  }
NodeInterface::~NodeInterface() {
    terminate();
  }
void NodeInterface::initialize(int argc, char * const argv[]) {
    try {
        //initialize ros2
        std::vector<char *> args(argv, argv + argc);
        rclcpp::init(static_cast<int>(args.size()), args.data());
        //create the Node specified in Model
        std::string NodeName("InverseKinematics");
        mNode = std::make_shared<rclcpp::Node>(NodeName);
        mExec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        mExec->add_node(mNode);
        //initialize the model which will initialize the publishers and subscribers
        mModel = std::make_shared<InverseKinematics>(
        );
		rtmSetErrorStatus(mModel->getRTM(), (NULL));
        mModel->initialize();
        //create the threads for the rates in the Model
        mBaseRateThread = std::make_shared<std::thread>(&NodeInterface::baseRateTask, this);
		mSchedulerThread = std::make_shared<std::thread>(&NodeInterface::schedulerThreadCallback, this);
    }
    catch (std::exception& ex) {
        std::cout << ex.what() << std::endl;
        throw ex;
    }
    catch (...) {
        std::cout << "Unknown exception" << std::endl;
        throw;
    }
}
int NodeInterface::run() {
  if (mExec) {
    mExec->spin();
  }
  mRunModel = false;
  return 0;
}
boolean_T NodeInterface::getStopRequestedFlag(void) {
    #ifndef rtmGetStopRequested
    return (!(rtmGetErrorStatus(mModel->getRTM())
        == (NULL)));
    #else
    return (!(rtmGetErrorStatus(mModel->getRTM())
        == (NULL)) || rtmGetStopRequested(mModel->getRTM()));
    #endif
}
void NodeInterface::stop(void) {
  if (mExec.get()) {
    mExec->cancel();
    if (mNode) {
      mExec->remove_node(mNode);
    }
    while (mExec.use_count() > 1);
  }
}
void NodeInterface::terminate(void) {
    if (mBaseRateThread.get()) {
        mRunModel = false;
        mBaseRateSem.notify(); // break out wait
        mBaseRateThread->join();
        if (mSchedulerThread.get()) {
            mSchedulerThread->join();
            mSchedulerThread.reset();
        }
        mBaseRateThread.reset();
        if (mModel.get()) {
            mModel->terminate();
        }
        // Release publisher InverseKinematics/Publisher/Publish
        mPub_InverseKinematics_466.reset();
        // Release subscriber InverseKinematics/Subscribe
        mSub_InverseKinematics_562.reset();
        mModel.reset();
        mExec.reset();
        mNode.reset();
        rclcpp::shutdown();
    }
}
//
void NodeInterface::schedulerThreadCallback(void)
{
  while (mRunModel) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(10000000));
        mBaseRateSem.notify();
    }
}
//Model specific
void NodeInterface::baseRateTask(void) {
  mRunModel = (rtmGetErrorStatus(mModel->getRTM()) ==
              (NULL));
  while (mRunModel) {
    mBaseRateSem.wait();
    if (!mRunModel) break;
    mModel->step();
    mRunModel &= !NodeInterface::getStopRequestedFlag(); //If RunModel and not stop requested
  }
  NodeInterface::stop();
}
// InverseKinematics/Publisher/Publish
void NodeInterface::create_Pub_InverseKinematics_466(const char *topicName, const rmw_qos_profile_t& qosProfile){
  mPub_InverseKinematics_466 = mNode->create_publisher<geometry_msgs::msg::Quaternion>(topicName, ros2::matlab::getQOSSettingsFromRMW(qosProfile));
}
void NodeInterface::publish_Pub_InverseKinematics_466(const SL_Bus_geometry_msgs_Quaternion* inBus) {
  auto msg = std::make_unique<geometry_msgs::msg::Quaternion>();
  convertFromBus(*msg, inBus);
  mPub_InverseKinematics_466->publish(std::move(msg));
}
// InverseKinematics/Subscribe
void NodeInterface::create_Sub_InverseKinematics_562(const char *topicName, const rmw_qos_profile_t& qosProfile){
    auto callback = [this](geometry_msgs::msg::Point::SharedPtr msg) {
        std::lock_guard<std::mutex> lockMsg(mtx_Sub_InverseKinematics_562);
        mLatestMsg_Sub_InverseKinematics_562 = msg;
    };
    mSub_InverseKinematics_562 = mNode->create_subscription<geometry_msgs::msg::Point>(topicName, ros2::matlab::getQOSSettingsFromRMW(qosProfile), callback);
}
bool NodeInterface::getLatestMessage_Sub_InverseKinematics_562(SL_Bus_geometry_msgs_Point* outBus) {
    if (mLatestMsg_Sub_InverseKinematics_562.get()) {
        std::lock_guard<std::mutex> lockMsg(mtx_Sub_InverseKinematics_562);
        convertToBus(outBus, *mLatestMsg_Sub_InverseKinematics_562);
        mLatestMsg_Sub_InverseKinematics_562.reset();
        return true;
    }
    return false;
}
// Helper for InverseKinematics/Publisher/Publish
void create_Pub_InverseKinematics_466(const char *topicName, const rmw_qos_profile_t& qosProfile){
  ros2::matlab::getNodeInterface()->create_Pub_InverseKinematics_466(topicName, qosProfile);
}
void publish_Pub_InverseKinematics_466(const SL_Bus_geometry_msgs_Quaternion* inBus) {
  ros2::matlab::getNodeInterface()->publish_Pub_InverseKinematics_466(inBus);
}
// Helper for InverseKinematics/Subscribe
void create_Sub_InverseKinematics_562(const char *topicName, const rmw_qos_profile_t& qosProfile){
  ros2::matlab::getNodeInterface()->create_Sub_InverseKinematics_562(topicName, qosProfile);
}
bool getLatestMessage_Sub_InverseKinematics_562(SL_Bus_geometry_msgs_Point* outBus) {
  return ros2::matlab::getNodeInterface()->getLatestMessage_Sub_InverseKinematics_562(outBus);
}
}//namespace matlab
}//namespace ros2
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
