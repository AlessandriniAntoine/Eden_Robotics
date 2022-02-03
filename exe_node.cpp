#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"

//#include "/home/alessandrini/Documents/Eden_Robotics/motors_ws/src/dynamixel-workbench/dynamixel_workbench_toolbox/include/dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "/home/alessandrini/Documents/Eden_Robotics/motors_ws/src/DynamixelSDK/ros/dynamixel_sdk/include/dynamixel_sdk/dynamixel_sdk.h"

#include <cmath>
#include <inttypes.h>


using namespace dynamixel;

// Control table address
#define OPERATING_MODE        11
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132

#define MOVING_SPEED        			 32
#define HAND_ADDR_TORQUE_ENABLE    24
#define HAND_ADDR_GOAL_POSITION    30
#define HAND_ADDR_PRESENT_POSITION 36
#define HAND_PRESENT_LOAD          40

// Protocol version
#define PROTOCOL_VERSION      1.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define DXL2_ID               2               // DXL2 ID
#define DXL3_ID               3               // DXL3 ID
#define DXL4_ID               4               // DXL4 ID
#define HAND_ID               5               // HAND ID
#define HAND_MAX_LOAD         7000
#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command



// Rapport de reduction
float r_bassin = 80/20; // ou 193/25
float r_epaule = 110/25;
float r_coude = 105/21;
float r_poignet = 50/25;

// Correction
float x_correction = (180/M_PI)*r_bassin*4095/360;
float y_correction = (180/M_PI)*r_epaule*4095/360;
float z_correction = (180/M_PI)*r_coude*4095/360;
float w_correction = (180/M_PI)*r_poignet*4095/360;

// Variables for initial motor positions
u_int32_t DXL1, DXL2, DXL3, DXL4;


PortHandler * portHandler;
PacketHandler * packetHandler;



void anglesCallback(const geometry_msgs::Quaternion& msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  
  // angles en radian
  float xf_angle = (float)msg.x; // Convert int32 -> uint32
  float yf_angle = (float)msg.y;
  float zf_angle = (float)msg.z;
  float wf_angle = (float)msg.w;


  // angles en degres 
  u_int32_t x_angle = DXL1 + x_correction * xf_angle; 
  u_int32_t y_angle = DXL2 + y_correction * yf_angle;
  u_int32_t z_angle = DXL3 + z_correction * zf_angle;
  u_int32_t w_angle = DXL4 + w_correction * wf_angle;


  // ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", 1, unsigned(x_angle));
  // ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", 2, unsigned(y_angle));
  // ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", 3, unsigned(z_angle));
  // ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", 4, unsigned(w_angle));

  ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", 1, unsigned(DXL1));
  ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", 2, unsigned(DXL2));
  ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", 3, unsigned(DXL3));
  ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", 4, unsigned(DXL4));


  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t) DXL1_ID, ADDR_GOAL_POSITION, x_angle, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    // ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", 1, x_angle);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }

  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t) DXL2_ID, ADDR_GOAL_POSITION, y_angle, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    // ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", 2, y_angle);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }

  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t) DXL3_ID, ADDR_GOAL_POSITION, z_angle, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    // ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", 3, z_angle);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }

  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t) 4, ADDR_GOAL_POSITION, w_angle, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    // ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", 4, w_angle);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }
}

void handCallback(const std_msgs::Bool& msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  u_int16_t move_speed;
  u_int16_t stop;
  
  if(msg.data){
      move_speed = 2047;
      stop = 0;

      ROS_INFO("setMovingSpeed : [ID:%d] [SPEED:%d]", HAND_ID, unsigned(move_speed));

      dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler, (uint8_t) HAND_ID, MOVING_SPEED, move_speed, &dxl_error);
      if (dxl_comm_result == COMM_SUCCESS) {} 
      else {
        ROS_ERROR("Failed to set moving speed! Result: %d", dxl_comm_result);
      }

      sleep(1);

      ROS_INFO("setMovingSpeed : [ID:%d] [SPEED:%d]", HAND_ID, unsigned(stop));

      dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler, (uint8_t) HAND_ID, MOVING_SPEED, stop, &dxl_error);
      if (dxl_comm_result == COMM_SUCCESS) {} 
      else {
        ROS_ERROR("Failed to set moving speed! Result: %d", dxl_comm_result);
      }
  }
  else{
      move_speed = 500;
      stop = 0;

      u_int16_t load = 0;

      ROS_INFO("setMovingSpeed : [ID:%d] [SPEED:%d]", HAND_ID, unsigned(move_speed));

      dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler, (uint8_t) HAND_ID, MOVING_SPEED, move_speed, &dxl_error);
      if (dxl_comm_result == COMM_SUCCESS) {} 
      else {
        ROS_ERROR("Failed to set moving speed! Result: %d", dxl_comm_result);
      }

      while(load<HAND_MAX_LOAD){
        dxl_comm_result = packetHandler->read2ByteTxRx(
          portHandler, (uint8_t)HAND_ID, HAND_PRESENT_LOAD, (u_int16_t *)&load, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
          ROS_INFO("getLoad : [ID:%d] -> [POSITION:%d]", HAND_ID, load);
        } else {
          ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
        }
      }
      ROS_INFO("setMovingSpeed : [ID:%d] [SPEED:%d]", HAND_ID, unsigned(stop));

      dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler, (uint8_t) HAND_ID, MOVING_SPEED, stop, &dxl_error);
      if (dxl_comm_result == COMM_SUCCESS) {}      
      else {
        ROS_ERROR("Failed to set moving speed! Result: %d", dxl_comm_result);
      }
  }
}

int main(int argc, char **argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  int IDS [5] = {DXL1_ID,DXL2_ID,DXL3_ID,DXL4_ID,HAND_ID};
  int i;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

//Open port and baudrate

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }


  //Torque ON

  for(i=0; i<5; i++){
    dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, IDS[i], ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      ROS_ERROR("Failed to enable torque for Dynamixel ID %d", IDS[i]);
      return -1;
    }
  }

  //Operating Mode

  for(i=0; i<4; i++){
    dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, IDS[i], OPERATING_MODE, 4, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to change operating mode for Dynamixel ID %d", IDS[i]);
    return -1;
    }
  }

  ROS_INFO("ALL GOOD");

  
  ros::init(argc, argv, "motors");

  ros::NodeHandle n;

  ros::Publisher init_pub = n.advertise<geometry_msgs::Quaternion>("motor_init", 10);

  u_int32_t CURRENT_POS [4] = {DXL1, DXL2, DXL3, DXL4};

  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)IDS[0], ADDR_PRESENT_POSITION, (uint32_t *)&DXL1, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", IDS[0], DXL1);
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return -1;
  }
  
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)IDS[1], ADDR_PRESENT_POSITION, (uint32_t *)&DXL2, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", IDS[1], DXL2);
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return -1;
  }

  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)IDS[2], ADDR_PRESENT_POSITION, (uint32_t *)&DXL3, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", IDS[2], DXL3);
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return -1;
  }

  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)IDS[3], ADDR_PRESENT_POSITION, (uint32_t *)&DXL4, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", IDS[3], DXL4);
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return -1;
  }
  
  geometry_msgs::Quaternion msg;
  msg.x = DXL1;
  msg.y = DXL2;
  msg.z = DXL3;
  msg.w = DXL4;


  init_pub.publish(msg);


  ros::Subscriber arm = n.subscribe("angles_liaison", 10, anglesCallback);

	ros::Subscriber hand = n.subscribe("hand", 10, handCallback);

  ros::spin();

  portHandler->closePort();
  return 0;
}