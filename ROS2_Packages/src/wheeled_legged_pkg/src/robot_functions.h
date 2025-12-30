#ifndef ROBOT_FUNCTIONS_H
#define ROBOT_FUNCTIONS_H

#include <vector>
#include <string>
#include <cstdint>
#include "robot_structs.h"


#define Joint_Start_Angle    96.25619525f
#define Motor_KP 1.0f   //0~500
#define Motor_KD 0.5f   //0~5



// 全局變量聲明（在cpp中定義）
extern ROS_body_t ROS_body;
extern int Serial_update_flag;
extern TX_MIT_Data_t TX_MIT_Data;//

void TxdMitData_init(void);
void printMITFeedbackData(const ROS_body_t& ros_body);
void printMitTxdData(const TX_MIT_Data_t& TX_Data);

void printRobotStatus(const ROS_body_t& ros_body);

// CRC16校驗函數
uint16_t crc16(const uint8_t* data, uint32_t length);

float degrees_to_radians(float degrees);
float radians_to_degrees(float radians) ;

// 結構體與字節數組互轉
size_t TXD_structToArr(const ComWheelLegged_t& structData, uint8_t* array, size_t arraySize);
size_t TXD_joint_structToArr(const TX_MIT_Data_t& structData, uint8_t* array, size_t arraySize);

size_t structToArr(const ROS_body_t& structData, uint8_t* array, size_t arraySize);
size_t arrToStruct(const uint8_t* array, size_t arraySize, ROS_body_t& structData);

// 數據打印函數
void printMilemeterData(const milemeter_t& milemeter);
void printImuData(const ImuData_t& imu);

// SerialPort相關函數
std::vector<std::string> listSerialPorts();
int openSerialPort(const std::string &port, speed_t baudRate);
std::vector<uint8_t> hexSend(const std::string &hexStr);
void displaySerialMenu(const std::vector<std::string>& ports);

//odom函數
void update_odom(float dt);
void TransAngleInPI(float angle,float& out_angle);

// 時間與數據接收函數
unsigned long millis();
void SERIAL_RXT(int serialFd);

#endif // ROBOT_FUNCTIONS_H