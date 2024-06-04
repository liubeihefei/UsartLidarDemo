#include "usart/usart.hpp"

bool serialport::initSerialPort(std::string serial, int baud, int time)
{
    //串口初始化
    try
    {
        sp.setPort(serial.c_str());
        //115200
        sp.setBaudrate(baud);
        //default:1000
        serial::Timeout to = serial::Timeout::simpleTimeout(time);	
        sp.setTimeout(to);

        sp.open();
    }
    catch(serial::IOException& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("serial"),"unable to open port!");
        RCLCPP_ERROR(rclcpp::get_logger("serial"),"Error:%s",e.what());
        
        return false;
    }
    
    //检查串口
    if(sp.isOpen())
    {
        RCLCPP_INFO(rclcpp::get_logger("serial"),"port is open");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("serial"),"unable to open");
        return false;
    }

    return true;
}

int serialport::getBit(std::uint8_t byte,int i)
{
    int bit =(int)((byte >> i) & 0x01);
    return bit;
}

void serialport::setByteFlags(std::uint8_t &byte, int start_index, int len, bool flag)
{
    auto temp = static_cast<std::uint8_t>((1 << len) - 1);
    if (flag)
        //置1
        byte |= temp << start_index;
    else
        //置0
        byte &= ~(temp << start_index);
}

bool serialport::ifHead(uint8_t head, int headnums)
{
    for(int i = 0;i < headnums;++i)
        if(buffer[i] != head)
            return false;
    return true;
}

void serialport::mvOnce(int headnums)
{
    // 读一个字节
    sp.read(&buffer[headnums], 1);
    // 进行移位
    for(int i = 0;i < headnums;++i)
        buffer[i] = buffer[i+1];
}

void serialport::readOnce(int headnums, int nums)
{
    sp.read(&buffer[headnums], nums);
}

void serialport::calSum(int headnums, int nums, bool flag)
{
    int begin;
    if(flag)
        begin = 0;
    else
        begin = headnums;

    sum = 0;
    for(int i = begin;i < headnums + nums - 1;++i)
        sum += buffer[i];
}

bool serialport::getData(uint8_t head, int headnums, int nums, bool flag)
{
    // 判断帧头
    while(!ifHead(head, headnums))
        mvOnce(head);

    // 读一帧数据
    readOnce(headnums, nums);

    // 判断校验和
    calSum(headnums, nums, flag);
    if(sum == buffer[headnums + nums -1])
        return true;
    else
        return false;
}

