#include "rclcpp/rclcpp.hpp"
#include "usart/usart.hpp"
#include "interfaces/msg/lidar.hpp"

// 节点类
class process : public rclcpp::Node
{
private:
    // 串口对象
    serialport lidar;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer;

    // 发布雷达数据
    rclcpp::Publisher<interfaces::msg::Lidar>::SharedPtr pub;

public:
    // 构造函数
    process(std::string name) : Node(name)
    {
        // 初始化串口
        lidar.initSerialPort("/dev/ttyUSB0", 115200, 1000);
        RCLCPP_INFO(this->get_logger(), "串口初始完毕");

        // 初始化雷达数据发布者
        pub = this->create_publisher<interfaces::msg::Lidar>("lidar_data", 1);

        // 开启定时回调，每1ms进一次回调
        timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&process::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "开启定时回调");
    }

    // 定时回调函数，用于接收雷达数据
    void timer_callback()
    {
        uint8_t head = 0x59;
        int headnums = 2;
        int nums = 7;

        if(lidar.getData(head, headnums, nums, true))
        {
            // 打印数据进行验证
            float distance = float(int(lidar.buffer[2]) + (int(lidar.buffer[3]) * 256));
            float weight = float(int(lidar.buffer[4]) + (int(lidar.buffer[5]) * 256));
            // RCLCPP_INFO(this->get_logger(), "距离为%f", distance);
            // RCLCPP_INFO(this->get_logger(), "强度为%f", weight);

            // 接口定义的雷达数据格式
            interfaces::msg::Lidar temp;
            temp.distance = distance * 10;
            temp.weight = weight;
            pub->publish(temp);
        }

    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<process>("demo");

    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
