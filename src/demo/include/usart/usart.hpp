#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"

class serialport
{
public:
    serial::Serial sp;
    uint8_t buffer[1024];
    uint8_t sum;

public:
    serialport(){};
    ~serialport(){};
    
    bool initSerialPort(std::string serial, int baud, int time);

    // 获取字节中某一位
    // b为传入的字节，i为第几位（范围0-7）
    int getBit(std::uint8_t byte, int i);

    // 置字节中某些（连续）位
    void setByteFlags(std::uint8_t &byte, int start_index, int len, bool flag);

    // 判断帧头
    bool ifHead(uint8_t head, int headnums);

    // 读一个字节，进行移位
    void mvOnce(int headnums);

    // 读一帧数据
    void readOnce(int headnums, int nums);

    // 计算校验和
    // flag为true表示算上帧头，否则不算
    void calSum(int headnums, int nums, bool flag);

    // 完成一次数据的获取
    // head为帧头，headnums为帧头个数，nums为帧头外字节个数，flag为true则校验和算上帧头
    bool getData(uint8_t head, int headnums, int nums, bool flag);
};
