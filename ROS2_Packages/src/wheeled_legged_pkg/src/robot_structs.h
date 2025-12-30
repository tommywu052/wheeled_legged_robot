#ifndef ROBOT_STRUCTS_H
#define ROBOT_STRUCTS_H

#include <cstdint> // 用于标准整数类型

/**
 * @brief 轮式机器人电机ID枚举
 * 包含左前、左后、右前、右后电机及左轮、右轮电机（适配不同底盘结构）
 */
enum MotorID : uint8_t  
{
    LEFT_FRONT = 0,    // 身体左前电机
    LEFT_REAR = 1,     // 身体左后电机
    RIGHT_FRONT = 2,   // 身体右前电机
    RIGHT_REAR = 3,    // 身体右后电机
    LEFT_WHEEL = 4,    // 左轮电机
    RIGHT_WHEEL = 5,   // 右轮电机

    TOTAL_COUNT        // 电机总数（用于数组大小定义、遍历等）
};



// 里程计数据结构
typedef struct {
    double  x;     
    double  y;    
    float angle;             
    float linear_speed;  // 前进速度 (m/s) 
    float angle_speed;   // 角速度 (rad/s) 
} odom_t;



struct RobotStatus {
    // 状态码联合体（你的定义）
    union {
        uint16_t status_code;  // 32位状态码整体
        struct {
            uint16_t switched_on : 1;           // 机器开启状态(1=开启，0=关闭)
            uint16_t model : 1;                 // 机器工作模式(1=控制电机模式，0=控制移动底盘模式)
            uint16_t PADDING : 14;              // 保留位（必须为0）
        };
    } status;
};




// ==================== 新增结构体定义 ====================
#pragma pack(push, 1)  // 确保1字节对齐，避免填充字节


// ROS通信数据结构体
typedef struct {
    uint8_t header1;          // 帧头1 (123)
    uint8_t header2;          // 帧头2 (45)
    uint16_t length;          // 结构体长度
    float linear_velocity;
    float angle_velocity;
    float leg_length;
    float pitch_angle;
    float roll_angle;
    uint16_t set;             // 設置
    uint8_t count;
    uint16_t crc;             // CRC16校验
    
} ComWheelLegged_t;


// 里程计数据结构
typedef struct {
    float LeftWheelSpeed;     // 左轮速度 (m/s)
    float RightWheelSpeed;    // 右轮速度 (m/s)
    float Speed;              // 前进速度 (m/s)
    float AngularVelocity;    // 角速度 (rad/s)
} milemeter_t;

// IMU傳感器數據結構
typedef struct {
    float accel[3];           // 加速度計數據 [x, y, z] (m/s²)
    float gyro[3];            // 陀螺儀數據 [x, y, z] (rad/s)
    float q0 = 1.0f;          // 四元數分量
    float q1 = 0.0f;          // 四元數分量
    float q2 = 0.0f;          // 四元數分量
    float q3 = 0.0f;          // 四元數分量
} ImuData_t;

// 電機控制參數
typedef struct {
    float position; // 位置指令
    float velocity; // 速度指令
    float kp;           // 比例增益
    float kd;           // 微分增益
    float effort;    // 前馈转矩
} MIT_Command_Data_t;


typedef struct {
    uint8_t header1;          // 帧头1 (123)
    uint8_t header2;          // 帧头2 (45)
    uint16_t length;          // 数据包长度
    MIT_Command_Data_t MIT_Command_Data[6];
    uint16_t count;            // 计数
    uint16_t crc;             // CRC16校验    
} TX_MIT_Data_t;


typedef struct {
    float position; // 电机位置信息
    float velocity; // 电机速度信息
    float torque;   // 力矩
    float Temp;           //回传温度
} MIT_Feedback_Data_t;


// ROS通信数据结构体
typedef struct {
    uint8_t header1;          // 帧头1 (123)
    uint8_t header2;          // 帧头2 (45)
    uint16_t length;          // 数据包长度
    milemeter_t milemeter;    // 里程计数据
    ImuData_t ImuData;        // IMU数据
    MIT_Feedback_Data_t MIT_Feedback_Data[6];//电机反馈
    int16_t SBUS_Channels_Data[10];
    RobotStatus Status;          // 状态标志
    uint16_t count;            // 计数
    uint16_t crc;             // CRC16校验
} ROS_body_t;


// ROS通信数据结构体
typedef struct {
    uint8_t header1;          // 帧头1 (123)
    uint8_t header2;          // 帧头2 (45)
    uint16_t length;          // 数据包长度
    MIT_Feedback_Data_t MIT_Feedback_Data[6];//电机反馈
    int16_t SBUS_Channels_Data[10];
    uint16_t count;            // 计数
    uint16_t crc;             // CRC16校验
} ROS_body_t1;


#pragma pack(pop)  // 恢復默認對齊方式



#endif // ROBOT_STRUCTS_H