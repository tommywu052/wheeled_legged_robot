#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <unistd.h>          // 確保包含此頭文件用於close()
#include <fcntl.h>
#include <termios.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <chrono>            // 確保包含此頭文件
#include <thread>            // 確保包含此頭文件
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include "robot_functions.h"




// 全局變量定義
ROS_body_t ROS_body;
ROS_body_t1 ROS_body1;

TX_MIT_Data_t TX_MIT_Data;//

int Serial_update_flag = 0;
int Serial_update_flag1 = 0;

unsigned long RX_error = 0;
unsigned int RX_error_hz = 0;


void TxdMitData_init(void)
{
    const size_t dataSize = sizeof(TX_MIT_Data_t);
    TX_MIT_Data.header1 = 123;
    TX_MIT_Data.header1 = 45;
    TX_MIT_Data.length = dataSize;

    for (int i = 0; i < 4; i++)
    {
        TX_MIT_Data.MIT_Command_Data[i].kp = Motor_KP;
        TX_MIT_Data.MIT_Command_Data[i].kd = Motor_KD;
        TX_MIT_Data.MIT_Command_Data[i].position = Joint_Start_Angle;
        TX_MIT_Data.MIT_Command_Data[i].velocity = 0;
        TX_MIT_Data.MIT_Command_Data[i].effort = 0;
    }
    TX_MIT_Data.MIT_Command_Data[MotorID::LEFT_WHEEL].kp = 0;
    TX_MIT_Data.MIT_Command_Data[MotorID::LEFT_WHEEL].kd = 0;
    TX_MIT_Data.MIT_Command_Data[MotorID::LEFT_WHEEL].effort = 0;

    TX_MIT_Data.MIT_Command_Data[MotorID::RIGHT_WHEEL].kp = 0;
    TX_MIT_Data.MIT_Command_Data[MotorID::RIGHT_WHEEL].kd = 0;
    TX_MIT_Data.MIT_Command_Data[MotorID::RIGHT_WHEEL].effort = 0;

}


/**
 * @brief 將角度轉換為弧度
 * @param degrees 角度值（單位：度）
 * @return 對應的弧度值（單位：弧度）
 * @note 轉換公式：弧度 = 角度 × π / 180
 */float degrees_to_radians(float degrees) 

{
    return degrees * M_PI / 180.0;
}


/**
 * @brief 將弧度轉換為角度
 * @param radians 弧度值（單位：弧度）
 * @return 對應的角度值（單位：度）
 * @note 轉換公式：角度 = 弧度 × 180 / π
 */
float radians_to_degrees(float radians) 
{
    return radians * 180.0 / M_PI;
}




/**
 * @brief CRC16校驗函數 (Modbus協議常用)
 */
uint16_t crc16(const uint8_t* data, uint32_t length) 
{
    uint16_t crc = 0xFFFF;  // 初始值
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];       // 與當前字節異或
        
        // 按位處理
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) { // 最低位為1
                crc >>= 1;
                crc ^= 0xA001;  // 多項式反轉值 (0x8005)
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}




/**
 * @brief 將結構體數據複製到字節數組
 */
size_t structToArr(const ROS_body_t& structData, uint8_t* array, size_t arraySize) {
    const size_t structSize = sizeof(ROS_body_t);
    
    // 安全檢查：數組不為空且大小足夠
    if (array == nullptr || arraySize < structSize) {
        return 0;
    }
    
    // 內存拷貝（替代循環，效率更高）
    memcpy(array, &structData, structSize);
    return structSize;
}

/**
 * @brief 將結構體數據複製到字節數組
 */
size_t TXD_structToArr(const ComWheelLegged_t& structData, uint8_t* array, size_t arraySize) {
   
    const size_t structSize = sizeof(ComWheelLegged_t);
    
    // 安全檢查：數組不為空且大小足夠
    if (array == nullptr || arraySize < structSize) {
        return 0;
    }
    
    // 內存拷貝（替代循環，效率更高）
    memcpy(array, &structData, structSize);
    return structSize;
}


size_t TXD_joint_structToArr(const TX_MIT_Data_t& structData, uint8_t* array, size_t arraySize) {
   
    const size_t structSize = sizeof(TX_MIT_Data_t);
    
    // 安全檢查：數組不為空且大小足夠
    if (array == nullptr || arraySize < structSize) {
        return 0;
    }
    
    // 內存拷貝（替代循環，效率更高）
    memcpy(array, &structData, structSize);
    return structSize;
}



/**
 * @brief 將字節數組數據複製到結構體
 */
size_t arrToStruct(const uint8_t* array, size_t arraySize, ROS_body_t& structData) {
    const size_t structSize = sizeof(ROS_body_t);
    
    // 安全檢查：數組不為空且大小足夠
    if (array == nullptr || arraySize < structSize) {
        return 0;
    }
    
    // 內存拷貝
    memcpy(&structData, array, structSize);
    return structSize;
}

size_t arrToStruct1(const uint8_t* array, size_t arraySize, ROS_body_t1& structData) {
    const size_t structSize = sizeof(ROS_body_t1);
    
    // 安全檢查：數組不為空且大小足夠
    if (array == nullptr || arraySize < structSize) {
        return 0;
    }
    
    // 內存拷貝
    memcpy(&structData, array, structSize);
    return structSize;
}

void printRobotStatus(const ROS_body_t& ros_body)
{
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "機器人狀態:\n";

    std::cout << "    開關機: " << ros_body.Status.status.switched_on << "\n";
    std::cout << "    機器工作模式: " << ros_body.Status.status.model << "\n";


}


void printMITFeedbackData(const ROS_body_t& ros_body)
{
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "電機反饋數據:\n";
    
    for (int i = 0; i < 6; ++i)
    {
        std::cout << "  電機 " << i+1 << ":\n";
        std::cout << "    位置: " << ros_body.MIT_Feedback_Data[i].position << "\n";
        std::cout << "    速度: " << ros_body.MIT_Feedback_Data[i].velocity << "\n";
        std::cout << "    力矩: " << ros_body.MIT_Feedback_Data[i].torque << "\n";
        std::cout << "    溫度: " << ros_body.MIT_Feedback_Data[i].Temp << "\n";
    }
}


void printMitTxdData(const TX_MIT_Data_t& TX_Data)
{
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "電機電機反饋數據:\n";
    
    for (int i = 0; i < 6; ++i)
    {
        std::cout << "  電機 " << i+1 << ":\n";
        std::cout << "    位置: " << TX_Data.MIT_Command_Data[i].position << "\n";
        std::cout << "    速度: " << TX_Data.MIT_Command_Data[i].velocity << "\n";
        std::cout << "    力矩: " << TX_Data.MIT_Command_Data[i].effort << "\n";
        std::cout << "    kp: " << TX_Data.MIT_Command_Data[i].kp << "\n";
        std::cout << "    kd: " << TX_Data.MIT_Command_Data[i].kd << "\n";
    }
}



void printSbusData(const ROS_body_t& ros_body)
{
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "遙控器通道數據:\n";
    
    for (int i = 0; i < 10; ++i)
    {
        std::cout << "  通道 " << i+1 << ":";
        std::cout << ros_body.SBUS_Channels_Data[i] << "\n";
    }
}



/**
 * @brief 印odom數據
 */
void printMilemeterData(const milemeter_t& milemeter) 
{
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "裡程計數據:\n";
    std::cout << "  左輪速度: " << milemeter.LeftWheelSpeed << " m/s\n";
    std::cout << "  右輪速度: " << milemeter.RightWheelSpeed << " m/s\n";
    std::cout << "  前進速度: " << milemeter.Speed << " m/s\n";
    std::cout << "  角速度: " << milemeter.AngularVelocity << " rad/s\n";
}

/**
 * @brief 印IMU傳感器數據
 */
void printImuData(const ImuData_t& imu) 
{
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "IMU數據:\n";
    std::cout << "  加速度: [" 
              << imu.accel[0] << ", " 
              << imu.accel[1] << ", " 
              << imu.accel[2] << "] m/s²\n";
    std::cout << "  陀螺儀: [" 
              << imu.gyro[0] << ", " 
              << imu.gyro[1] << ", " 
              << imu.gyro[2] << "] rad/s\n";
    std::cout << "  四元數分量: [" 
              << imu.q0 << ", " 
              << imu.q1 << ", " 
              << imu.q2 << ", " 
              << imu.q3 << "] \n";   
    
    // 計算歐拉角（從四元數轉換）
    float roll = atan2(2*(imu.q0*imu.q1 + imu.q2*imu.q3), 1 - 2*(imu.q1*imu.q1 + imu.q2*imu.q2));
    float pitch = asin(2*(imu.q0*imu.q2 - imu.q3*imu.q1));
    float yaw = atan2(2*(imu.q0*imu.q3 + imu.q1*imu.q2), 1 - 2*(imu.q2*imu.q2 + imu.q3*imu.q3));
    
    std::cout << "  姿態 (歐拉角):\n";
    std::cout << "  橫滾(Roll): " << roll * 180/M_PI << "°\n";
    std::cout << "  俯仰(Pitch): " << pitch * 180/M_PI << "°\n";
    std::cout << "  偏航(Yaw): " << yaw * 180/M_PI << "°\n";
}

/**
 * @brief 列舉系統中可用的串口設備
 */
std::vector<std::string> listSerialPorts() {
    std::vector<std::string> ports;
    
    DIR* dir = opendir("/dev");
    if (!dir) return ports;

    dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        
        // 識別常見的串口設備名格式
        if (name.substr(0, 6) == "ttyUSB" || 
            name.substr(0, 6) == "ttyACM" ||
            name.substr(0, 4) == "ttyS"  ||
            name.substr(0, 6) == "ttyAMA") {
            ports.push_back("/dev/" + name);
        }
    }
    closedir(dir);
    return ports;
}

/**
 * @brief 打開並配置串口
 */
int openSerialPort(const std::string &port, speed_t baudRate) {
    // 打開串口設備（讀寫模式，不成為控制終端）
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("無法打開串口");
        return -1;
    }

    // 獲取當前串口配置
    struct termios tty;
    if (tcgetattr(fd, &tty) == -1) {
        perror("獲取串口屬性失敗");
        close(fd);
        return -1;
    }

    // 設置波特率
    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    // 配置數據格式：8位數據位，無校驗位，1個停止位
    tty.c_cflag &= ~PARENB;    // 無校驗位
    tty.c_cflag &= ~CSTOPB;    // 1個停止位
    tty.c_cflag &= ~CSIZE;     // 清除數據位設置
    tty.c_cflag |= CS8;        // 8位數據位

    // 禁用硬件流控，啟用接收
    tty.c_cflag &= ~CRTSCTS;   // 禁用硬件流控
    tty.c_cflag |= (CLOCAL | CREAD);  // 啟用接收，設置本地模式

    // 禁用規範模式和回顯
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 禁用軟件流控和特殊字符處理
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | 
                     INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;     // 禁用輸出處理
    tty.c_oflag &= ~ONLCR;     // 禁用換行轉換

    // 設置超時：1秒超時，最小讀取0字節
    tty.c_cc[VTIME] = 10;      // 10*100ms = 1秒
    tty.c_cc[VMIN] = 0;

    // 應用配置
    if (tcsetattr(fd, TCSANOW, &tty) == -1) {
        perror("設置串口屬性失敗");
        close(fd);
        return -1;
    }

    return fd;
}

/**
 * @brief 十六進制字符串轉字節數組
 */
std::vector<uint8_t> hexSend(const std::string &hexStr) {
    std::vector<uint8_t> bytes;
    
    for (size_t i = 0; i < hexStr.length(); i += 2) {
        std::string byteString = hexStr.substr(i, 2);
        uint8_t byte = (uint8_t)strtol(byteString.c_str(), nullptr, 16);
        bytes.push_back(byte);
    }
    return bytes;
}

/**
 * @brief 顯示串口選擇菜單
 */
void displaySerialMenu(const std::vector<std::string>& ports) {
    std::cout << "\n===== 串口選擇菜單 =====" << std::endl;
    for (size_t i = 0; i < ports.size(); ++i) {
        std::cout << "  [" << i + 1 << "] " << ports[i] << std::endl;
    }
    std::cout << "  [R] 重新掃描串口" << std::endl;
    std::cout << "  [Q] 退出程序" << std::endl;
    std::cout << "=========================" << std::endl;
    std::cout << "請選擇要使用的串口 (1-" << ports.size() << "): ";
}

/**
 * @brief 獲取當前時間戳（毫秒）
 */
unsigned long millis() {
    using namespace std::chrono;
    static const auto start = steady_clock::now();
    return duration_cast<milliseconds>(steady_clock::now() - start).count();
}


void RxtErrorHz(void)
{
    static unsigned long ms = millis();

    unsigned long tt = millis() - ms;
    if(tt>1000)
    {
        ms = millis();   
        RX_error_hz = RX_error;
        RX_error = 0;
    }
}



/**
 * @brief 讀取並解析串口數據
 */
void SERIAL_RXT(int serialFd) 
{
    static unsigned long ms = millis();
    static unsigned long ms1 = millis();
    static unsigned char count = 0;       // 接收字節計數
    static unsigned char recstatu = 0;    // 接收狀態標誌
    const size_t dataSize = sizeof(ROS_body_t);
    const size_t dataSize1 = sizeof(ROS_body_t1);
    static uint8_t dataArray[dataSize];   // 接收數據緩衝區
    uint16_t crc = 0;

    // 檢查串口是否有數據可讀
    int bytesAvailable;
    ioctl(serialFd, FIONREAD, &bytesAvailable);
    RxtErrorHz();
    while (bytesAvailable > 0) 
    {
        uint8_t dat;
        ssize_t bytesRead = read(serialFd, &dat, 1);
        if (bytesRead != 1) {
            perror("讀取串口數據失敗");
            break;
        }
        bytesAvailable--;

        // 保存當前cout狀態（用於恢復）
        std::ios_base::fmtflags old_flags = std::cout.flags();
        std::streamsize old_precision = std::cout.precision();
  
        
        // 幀頭檢測與數據接收邏輯
        if ((count == 0) && (dat == 0x7B)) { // 幀頭1檢測
            dataArray[count] = dat;
            count = 1;
        }
        else if ((count == 1) && (dat == 0x2D)) { // 幀頭2檢測
            dataArray[count] = dat;
            recstatu = 1;  // 標記為已檢測到完整幀頭
            count = 2;
        }
        else if (recstatu == 1) { // 接收數據體
            dataArray[count] = dat;
            count++;
            
            // 檢查是否接收到完整幀（根據長度字段判斷）
            if (count >= dataArray[2]) 
            {
                if(dataArray[2]==dataSize)
                {
                    // 計算CRC（不包括最後的CRC字段）
                    crc = crc16(dataArray, dataSize - 2);
                    
                    ROS_body_t restoredStruct;
                    size_t copied = arrToStruct(dataArray, dataSize, restoredStruct);
                    //std::cout  << " copied:" << static_cast<unsigned int>(copied) << "\n";

                    if (copied == dataArray[2]) 
                    {
                        if (crc == restoredStruct.crc) // CRC校验通过
                        { 
                            unsigned long tt = millis() - ms;
                            ms = millis();
                            if(Serial_update_flag==0)
                            {
                                ROS_body = restoredStruct;
                                Serial_update_flag = 1;

                            }

                            
                            // 打印解析后的数据
                            //printRobotStatus(ROS_body);
                            //printMilemeterData(ROS_body.milemeter);
                            //printMITFeedbackData(ROS_body);
                            //printSbusData(ROS_body);
                            //printImuData(ROS_body.ImuData);
                            //std::cout << "间隔时间: " << std::dec << tt << " ms" << std::endl;
                            //std::cout << "RX_error: " << std::dec << RX_error << " " << std::endl;
                            //std::cout << "RX_error_hz: " << std::dec << RX_error_hz << " hz" << std::endl;
                    
                        }
                        else 
                        { // CRC校验失败

                            RX_error++;
/*

                            for (int i = 0; i < dataArray[2]; i++)
                            {
                                std::cout << " i:" << static_cast<unsigned int>(i) << " dataArray[]:" << static_cast<unsigned int>(dataArray[i]) << "\n";

                            }
                  

                            // 打印16進制（帶格式控制）
                            std::cout << "ROS CRC (16进制): 0x" 
                                    << std::hex                  // 切换为16进制模式
                                    << std::setw(4)              // 固定宽度（16位CRC需要4个16进制位）
                                    << std::setfill('0')         // 不足宽度时用0填充
                                    << std::uppercase            // 大写字母（可选）
                                    << static_cast<unsigned int>(ROS_body.crc)  // 強制轉換為整數類型
                                    << std::dec                  // 恢复十进制模式（避免影响后续输出）
                                    << std::endl;

*/                            


                        }
                    } 
                    else 
                    { // 数组转结构体失败
                        std::cout << "数组转结构体失败！" << std::endl;
                    }


                }
                else if(dataArray[2]==dataSize1)
                {

                    // 计算CRC（不包括最后的CRC字段）
                    crc = crc16(dataArray, dataSize1 - 2);
                    
                    ROS_body_t1 restoredStruct1;
                    size_t copied1 = arrToStruct1(dataArray, dataSize1, restoredStruct1);
                    //std::cout  << " copied1:" << static_cast<unsigned int>(copied1) << "\n";

                    if (copied1 == dataArray[2]) 
                    {
                        if (crc == restoredStruct1.crc) // CRC校验通过
                        { 
                            unsigned long tt = millis() - ms1;
                            ms1 = millis();
                            if(Serial_update_flag==0)
                            {
                                ROS_body1 = restoredStruct1;
                                Serial_update_flag = 1;

                            }

                            
                            // 打印解析后的数据
                            //printMilemeterData(ROS_body.milemeter);
                            //printImuData(ROS_body.ImuData);
                            //std::cout << "间隔时间1: " << std::dec << tt << " ms" << std::endl;
                    
                        }
                        else 
                        { // CRC校验失败
                            for (int i = 0; i < dataArray[2]; i++)
                            {
                                std::cout << " 1i:" << static_cast<unsigned int>(i) << " 1dataArray[]:" << static_cast<unsigned int>(dataArray[i]) << "\n";

                            }


                            // 打印16進制（帶格式控制）
                            std::cout << "1ROS CRC (16进制): 0x" 
                                    << std::hex                  // 切换为16进制模式
                                    << std::setw(4)              // 固定宽度（16位CRC需要4个16进制位）
                                    << std::setfill('0')         // 不足宽度时用0填充
                                    << std::uppercase            // 大写字母（可选）
                                    << static_cast<unsigned int>(ROS_body1.crc)  // 強制轉換為整數類型
                                    << std::dec                  // 恢复十进制模式（避免影响后续输出）
                                    << std::endl;
                        }
                    } 
                    else 
                    { // 数组转结构体失败
                        std::cout << "1数组转结构体失败！" << std::endl;
                    }

                }
                else
                {
                    std::cout << "接收數據 長度錯誤\n";

                }

                // 重置接收狀態
                memset(dataArray, 0, dataSize);
                recstatu = 0;
                count = 0;
            }
        }
        else { // 无效数据，重置状态
            memset(dataArray, 0, dataSize);
            recstatu = 0;
            count = 0;
        }

        // 恢复原始cout状态
        std::cout.flags(old_flags);
        std::cout.precision(old_precision);
    }
}






